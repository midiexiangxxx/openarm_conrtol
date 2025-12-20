#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>

class JointStateController : public rclcpp::Node
{
public:
  JointStateController() : Node("joint_state_controller")
  {
    // Declare parameters
    this->declare_parameter("planning_group", "left_arm");
    this->declare_parameter("execution_timeout", 10.0);
    this->declare_parameter("planning_time", 5.0);
    this->declare_parameter("max_velocity_scaling_factor", 0.1);
    this->declare_parameter("max_acceleration_scaling_factor", 0.1);

    // Get parameters
    planning_group_ = this->get_parameter("planning_group").as_string();
    execution_timeout_ = this->get_parameter("execution_timeout").as_double();
    planning_time_ = this->get_parameter("planning_time").as_double();
    max_velocity_scaling_ = this->get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_ = this->get_parameter("max_acceleration_scaling_factor").as_double();

    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt for planning group: %s", planning_group_.c_str());

    // Create subscriber for joint state goals with topic name based on planning group
    std::string topic_name = planning_group_ + "/joint_goal";
    joint_goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        topic_name, 10,
        std::bind(&JointStateController::jointGoalCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Joint State Controller initialized. Waiting for joint goals on topic '%s'", topic_name.c_str());
  }

  void initialize()
  {
    // Initialize MoveIt MoveGroupInterface after node is fully constructed
    using moveit::planning_interface::MoveGroupInterface;

    RCLCPP_INFO(this->get_logger(), "Initializing MoveGroupInterface...");

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    move_group_node_ = rclcpp::Node::make_shared("move_group_interface_node", node_options);

    move_group_ = std::make_shared<MoveGroupInterface>(move_group_node_, planning_group_);

    // Set planning parameters
    move_group_->setPlanningTime(planning_time_);
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);

    RCLCPP_INFO(this->get_logger(), "MoveIt initialized successfully");
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_->getEndEffectorLink().c_str());
  }

private:
  void jointGoalCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received joint goal with %zu joints", msg->position.size());

    // Validate message
    if (msg->name.empty() || msg->position.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid joint goal message: empty joint names or positions");
      return;
    }

    if (msg->name.size() != msg->position.size())
    {
      RCLCPP_ERROR(this->get_logger(), "Joint names and positions size mismatch");
      return;
    }

    // Create joint value map
    std::map<std::string, double> target_joints;
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      target_joints[msg->name[i]] = msg->position[i];
      RCLCPP_INFO(this->get_logger(), "  %s: %.4f", msg->name[i].c_str(), msg->position[i]);
    }

    // Set joint value target
    try
    {
      move_group_->setJointValueTarget(target_joints);

      RCLCPP_INFO(this->get_logger(), "Planning trajectory...");

      // Plan the motion
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (success)
      {
        RCLCPP_INFO(this->get_logger(), "Planning succeeded! Executing trajectory...");

        // Execute the planned trajectory
        auto result = move_group_->execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
          RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully!");
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", result.val);
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during planning/execution: %s", e.what());
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<rclcpp::Node> move_group_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_goal_sub_;

  std::string planning_group_;
  double execution_timeout_;
  double planning_time_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<JointStateController>();

  // Initialize MoveIt after node is created
  node->initialize();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
