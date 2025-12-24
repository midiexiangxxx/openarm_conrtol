#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
import time


class JointGoalPublisher(Node):
    def __init__(self):
        super().__init__("joint_goal_publisher")
        # Create publishers for both arms
        self.left_publisher = self.create_publisher(
            JointState, "left_arm/joint_goal", 10
        )
        self.right_publisher = self.create_publisher(
            JointState, "right_arm/joint_goal", 10
        )

        # Create action clients for grippers
        self.left_gripper_action = ActionClient(
            self,
            GripperCommand,
            '/left_gripper_controller/gripper_cmd'
        )
        self.right_gripper_action = ActionClient(
            self,
            GripperCommand,
            '/right_gripper_controller/gripper_cmd'
        )

    def send_joint_goal(self, joint_names, joint_positions):
        """发送关节目标"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = joint_positions

        # Determine which arm based on joint names
        if joint_names and "left" in joint_names[0]:
            topic = "left_arm/joint_goal"
            publisher = self.left_publisher
        elif joint_names and "right" in joint_names[0]:
            topic = "right_arm/joint_goal"
            publisher = self.right_publisher
        else:
            self.get_logger().error("Cannot determine arm from joint names!")
            return

        self.get_logger().info(
            f"Sending joint goal to {topic} with {len(joint_names)} joints"
        )
        for name, pos in zip(joint_names, joint_positions):
            self.get_logger().info(f"  {name}: {pos:.4f}")

        publisher.publish(msg)
        self.get_logger().info("Joint goal published!")

    def control_gripper(self, gripper_name, position, max_effort=30.1):
        """
        控制夹爪移动到指定位置

        Args:
            gripper_name: 'left' 或 'right'
            position: 目标位置 (0.0-0.044)
            max_effort: 最大力度 (默认0.10 N)

        Returns:
            bool: 是否成功执行
        """
        # 限制位置范围
        position = max(0.0, min(0.044, position))

        # 选择Action客户端
        if gripper_name == 'left':
            action_client = self.left_gripper_action
        else:
            action_client = self.right_gripper_action

        # 等待action服务器
        self.get_logger().info(f'等待{gripper_name}夹爪action服务器...')
        if not action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f'{gripper_name}夹爪action服务器不可用')
            return False

        # 创建goal消息
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(
            f'发送{gripper_name}夹爪命令: 目标位置={position:.4f}'
        )

        # 发送goal
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{gripper_name}夹爪goal被拒绝')
            return False

        # 等待执行结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result()
        if result:
            status = result.result
            self.get_logger().info(
                f'{gripper_name}夹爪执行完成: 位置={status.position:.4f}'
            )
            return status.reached_goal
        else:
            self.get_logger().error(f'{gripper_name}夹爪执行失败')
            return False


def main(args=None):
    rclpy.init(args=args)

    node = JointGoalPublisher()

    # 等待一下让节点启动
    time.sleep(1)

    print("\n=== OpenArm 关节控制器测试 ===\n")
    print("请确保已经启动了以下节点：")
    print("1. ros2 launch openarm_bimanual_moveit_config demo.launch.py")
    print(
        "2. ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=left_arm"
    )
    print(
        "3. ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=right_arm\n"
    )

    while True:
        print("\n选择测试场景：")
        print("=== 机械臂控制 ===")
        print("1. 左臂 - 回到零位")
        print("2. 左臂 - 测试位置 1")
        print("3. 右臂 - 回到零位")
        print("4. 右臂 - 测试位置 1")
        print("\n=== 夹爪控制 ===")
        print("5. 左夹爪 - 打开 (0.044)")
        print("6. 左夹爪 - 关闭 (0.0)")
        print("7. 右夹爪 - 打开 (0.044)")
        print("8. 右夹爪 - 关闭 (0.0)")
        print("\n0. 退出")

        choice = input("\n请选择 (0-8): ").strip()

        if choice == "0":
            break
        elif choice == "1":
            # 左臂回零
            node.send_joint_goal(
                joint_names=[
                    "openarm_left_joint1",
                    "openarm_left_joint2",
                    "openarm_left_joint3",
                    "openarm_left_joint4",
                    "openarm_left_joint5",
                    "openarm_left_joint6",
                    "openarm_left_joint7",
                ],
                joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        elif choice == "2":
            # 左臂测试位置1
            node.send_joint_goal(
                joint_names=[
                    "openarm_left_joint1",
                    "openarm_left_joint2",
                    "openarm_left_joint3",
                    "openarm_left_joint4",
                    "openarm_left_joint5",
                    "openarm_left_joint6",
                    "openarm_left_joint7",
                ],
                joint_positions=[
                    -0.35877775234607334,
                    -0.11959258411535778,
                    -0.15201800564583756,
                    1.285763332570383,
                    0.12531471732661892,
                    -0.07267109178301645,
                    -0.3175783932249949,
                ],
            )
        elif choice == "3":
            # 右臂回零
            node.send_joint_goal(
                joint_names=[
                    "openarm_right_joint1",
                    "openarm_right_joint2",
                    "openarm_right_joint3",
                    "openarm_right_joint4",
                    "openarm_right_joint5",
                    "openarm_right_joint6",
                    "openarm_right_joint7",
                ],
                joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
        elif choice == "4":
            # 右臂测试位置1
            node.send_joint_goal(
                joint_names=[
                    "openarm_right_joint1",
                    "openarm_right_joint2",
                    "openarm_right_joint3",
                    "openarm_right_joint4",
                    "openarm_right_joint5",
                    "openarm_right_joint6",
                    "openarm_right_joint7",
                ],
                joint_positions=[
                    0.43812466620889623,
                    0.0017166399633783413,
                    0.014686808575570254,
                    1.3963912413214317,
                    0.029564354924849212,
                    0.04100862134737149,
                    0.11692225528343592,
                ],
            )
        elif choice == "9":
            # 右臂测试位置2
            node.send_joint_goal(
                joint_names=[
                    "openarm_right_joint1",
                    "openarm_right_joint2",
                    "openarm_right_joint3",
                    "openarm_right_joint4",
                    "openarm_right_joint5",
                    "openarm_right_joint6",
                    "openarm_right_joint7",
                ],
                joint_positions=[
                    0.34618905928129884,
                    -0.03316929884794373,
                    -1.1823834592202633,
                    0.5159456778820495,
                    0.028801403830014394,
                    0.31757839322499315,
                    1.3097962920576798,
                ],
            )
        elif choice == "5":
            # 左夹爪打开
            print("打开左夹爪...")
            node.control_gripper('left', 0.044)
        elif choice == "6":
            # 左夹爪关闭
            print("关闭左夹爪...")
            node.control_gripper('left', 0.0)
        elif choice == "7":
            # 右夹爪打开
            print("打开右夹爪...")
            node.control_gripper('right', 0.044)
        elif choice == "8":
            # 右夹爪关闭
            print("关闭右夹爪...")
            node.control_gripper('right', 0.0)
        else:
            print("无效的选择，请重试。")

        # 等待一下
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
