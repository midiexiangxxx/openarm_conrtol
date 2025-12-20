from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Load MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "openarm", package_name="openarm_bimanual_moveit_config"
    ).to_moveit_configs()

    # Declare launch arguments
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='left_arm',
        description='MoveIt planning group name (left_arm or right_arm)'
    )

    execution_timeout_arg = DeclareLaunchArgument(
        'execution_timeout',
        default_value='10.0',
        description='Trajectory execution timeout in seconds'
    )

    planning_time_arg = DeclareLaunchArgument(
        'planning_time',
        default_value='5.0',
        description='Maximum planning time in seconds'
    )

    max_velocity_scaling_arg = DeclareLaunchArgument(
        'max_velocity_scaling_factor',
        default_value='0.1',
        description='Maximum velocity scaling factor (0.0-1.0)'
    )

    max_acceleration_scaling_arg = DeclareLaunchArgument(
        'max_acceleration_scaling_factor',
        default_value='0.1',
        description='Maximum acceleration scaling factor (0.0-1.0)'
    )

    # Joint state controller node with MoveIt configuration
    joint_controller_node = Node(
        package='openarm_joint_controller',
        executable='joint_state_controller',
        name='joint_state_controller',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'planning_group': LaunchConfiguration('planning_group'),
                'execution_timeout': LaunchConfiguration('execution_timeout'),
                'planning_time': LaunchConfiguration('planning_time'),
                'max_velocity_scaling_factor': LaunchConfiguration('max_velocity_scaling_factor'),
                'max_acceleration_scaling_factor': LaunchConfiguration('max_acceleration_scaling_factor'),
            }
        ]
    )

    return LaunchDescription([
        planning_group_arg,
        execution_timeout_arg,
        planning_time_arg,
        max_velocity_scaling_arg,
        max_acceleration_scaling_arg,
        joint_controller_node,
    ])
