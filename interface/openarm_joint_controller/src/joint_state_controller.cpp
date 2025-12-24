#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <cmath>

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
    this->declare_parameter("goal_tolerance", 0.01);  // 目标容差（弧度）

    // Get parameters
    planning_group_ = this->get_parameter("planning_group").as_string();
    execution_timeout_ = this->get_parameter("execution_timeout").as_double();
    planning_time_ = this->get_parameter("planning_time").as_double();
    max_velocity_scaling_ = this->get_parameter("max_velocity_scaling_factor").as_double();
    max_acceleration_scaling_ = this->get_parameter("max_acceleration_scaling_factor").as_double();
    goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt for planning group: %s", planning_group_.c_str());

    // Create subscriber for joint state goals with topic name based on planning group
    std::string topic_name = planning_group_ + "/joint_goal";
    joint_goal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        topic_name, 10,
        std::bind(&JointStateController::jointGoalCallback, this, std::placeholders::_1));

    // Create publishers for execution status
    std::string status_topic = planning_group_ + "/execution_status";
    std::string result_topic = planning_group_ + "/execution_result";

    execution_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(status_topic, 10);
    execution_result_pub_ = this->create_publisher<std_msgs::msg::String>(result_topic, 10);

    // Subscribe to joint states to get current position
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&JointStateController::jointStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Joint State Controller initialized. Waiting for joint goals on topic '%s'", topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing execution status to '%s' and '%s'", status_topic.c_str(), result_topic.c_str());
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
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Store current joint states
    current_joint_state_ = msg;
  }

  bool checkGoalReached(const std::map<std::string, double>& target_joints)
  {
    if (!current_joint_state_)
    {
      RCLCPP_WARN(this->get_logger(), "No current joint state available");
      return false;
    }

    // Check if all target joints are within tolerance
    double max_error = 0.0;
    for (const auto& target : target_joints)
    {
      const std::string& joint_name = target.first;
      double target_pos = target.second;

      // Find joint in current state
      auto it = std::find(current_joint_state_->name.begin(),
                         current_joint_state_->name.end(),
                         joint_name);

      if (it == current_joint_state_->name.end())
      {
        RCLCPP_WARN(this->get_logger(), "Joint %s not found in current state", joint_name.c_str());
        continue;
      }

      size_t idx = std::distance(current_joint_state_->name.begin(), it);
      double current_pos = current_joint_state_->position[idx];
      double error = std::abs(target_pos - current_pos);

      if (error > max_error)
      {
        max_error = error;
      }

      RCLCPP_INFO(this->get_logger(), "  %s: target=%.4f, current=%.4f, error=%.4f",
                  joint_name.c_str(), target_pos, current_pos, error);
    }

    RCLCPP_INFO(this->get_logger(), "Max joint error: %.4f rad (tolerance: %.4f rad)",
                max_error, goal_tolerance_);

    return max_error <= goal_tolerance_;
  }

  void publishExecutionStatus(bool success, const std::string& message)
  {
    // Publish boolean status
    auto status_msg = std_msgs::msg::Bool();
    status_msg.data = success;
    execution_status_pub_->publish(status_msg);

    // Publish detailed message
    auto result_msg = std_msgs::msg::String();
    result_msg.data = message;
    execution_result_pub_->publish(result_msg);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Status: SUCCESS - %s", message.c_str());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Status: FAILED - %s", message.c_str());
    }
  }

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
          RCLCPP_INFO(this->get_logger(), "Trajectory execution completed!");

          // Wait a bit for joints to settle
          rclcpp::sleep_for(std::chrono::milliseconds(500));

          // Check if goal was actually reached
          RCLCPP_INFO(this->get_logger(), "Checking if goal was reached...");
          
          bool goal_reached = checkGoalReached(target_joints);

          if (goal_reached)
          {
            publishExecutionStatus(true, "Goal reached successfully");
          }
          else
          {
            publishExecutionStatus(false, "Execution completed but goal not reached within tolerance");
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Execution failed with error code: %d", result.val);
          publishExecutionStatus(false, "Execution failed with error code: " + std::to_string(result.val));
        }
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        publishExecutionStatus(false, "Planning failed");
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Exception during planning/execution: %s", e.what());
      publishExecutionStatus(false, std::string("Exception: ") + e.what());
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::shared_ptr<rclcpp::Node> move_group_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr execution_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_result_pub_;

  sensor_msgs::msg::JointState::SharedPtr current_joint_state_;

  std::string planning_group_;
  double execution_timeout_;
  double planning_time_;
  double max_velocity_scaling_;
  double max_acceleration_scaling_;
  double goal_tolerance_;
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
