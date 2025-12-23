#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from control_msgs.action import GripperCommand
import time
import yaml
import sys
import os
import threading


class ActionSequenceExecutor(Node):
    def __init__(self):
        super().__init__("action_sequence_executor")

        # Create publishers for both arms
        self.left_publisher = self.create_publisher(
            JointState, "left_arm/joint_goal", 10
        )
        self.right_publisher = self.create_publisher(
            JointState, "right_arm/joint_goal", 10
        )

        # Create action clients for grippers
        self.left_gripper_action = ActionClient(
            self, GripperCommand, "/left_gripper_controller/gripper_cmd"
        )
        self.right_gripper_action = ActionClient(
            self, GripperCommand, "/right_gripper_controller/gripper_cmd"
        )

        # Create subscribers for execution status
        self.left_status_sub = self.create_subscription(
            Bool, "left_arm/execution_status", self.left_status_callback, 10
        )
        self.right_status_sub = self.create_subscription(
            Bool, "right_arm/execution_status", self.right_status_callback, 10
        )
        self.left_result_sub = self.create_subscription(
            String, "left_arm/execution_result", self.left_result_callback, 10
        )
        self.right_result_sub = self.create_subscription(
            String, "right_arm/execution_result", self.right_result_callback, 10
        )

        # Execution status tracking
        self.left_execution_status = None
        self.right_execution_status = None
        self.left_execution_result = ""
        self.right_execution_result = ""
        self.status_lock = threading.Lock()

        self.get_logger().info("动作序列执行器已初始化")

    def left_status_callback(self, msg):
        with self.status_lock:
            self.left_execution_status = msg.data

    def right_status_callback(self, msg):
        with self.status_lock:
            self.right_execution_status = msg.data

    def left_result_callback(self, msg):
        with self.status_lock:
            self.left_execution_result = msg.data

    def right_result_callback(self, msg):
        with self.status_lock:
            self.right_execution_result = msg.data

    def wait_for_arm_execution(self, arm, timeout=30.0):
        """
        等待机械臂执行完成并返回是否成功

        Args:
            arm: 'left' 或 'right'
            timeout: 超时时间（秒）

        Returns:
            bool: 执行是否成功
        """
        start_time = time.time()

        # Reset status
        with self.status_lock:
            if arm == "left":
                self.left_execution_status = None
                self.left_execution_result = ""
            else:
                self.right_execution_status = None
                self.right_execution_result = ""

        self.get_logger().info(f"等待 {arm} 臂执行完成...")

        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            with self.status_lock:
                if arm == "left":
                    status = self.left_execution_status
                    result = self.left_execution_result
                else:
                    status = self.right_execution_status
                    result = self.right_execution_result

            if status is not None:
                if status:
                    self.get_logger().info(f"{arm} 臂执行成功: {result}")
                    return True
                else:
                    # self.get_logger().error(f"{arm} 臂执行失败: {result}")
                    return False

        self.get_logger().error(f"{arm} 臂执行超时")
        return False

    def send_joint_goal(self, arm, joint_names, joint_positions):
        """
        发送关节目标并等待执行完成

        Args:
            arm: 'left' 或 'right'
            joint_names: 关节名称列表
            joint_positions: 关节位置列表

        Returns:
            bool: 是否成功执行
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = joint_positions

        # 选择发布者
        if arm == "left":
            topic = "left_arm/joint_goal"
            publisher = self.left_publisher
        elif arm == "right":
            topic = "right_arm/joint_goal"
            publisher = self.right_publisher
        else:
            self.get_logger().error(f"未知的机械臂: {arm}")
            return False

        self.get_logger().info(f"发送 {arm} 臂关节目标到 {topic}")
        for name, pos in zip(joint_names, joint_positions):
            self.get_logger().info(f"  {name}: {pos:.4f}")

        publisher.publish(msg)
        self.get_logger().info("关节目标已发布，等待执行完成...")

        # 等待执行完成
        return self.wait_for_arm_execution(arm)

    def control_gripper(self, arm, position, max_effort=20.0):
        """
        控制夹爪移动到指定位置

        Args:
            arm: 'left' 或 'right'
            position: 目标位置 (0.0-0.044)
            max_effort: 最大力度

        Returns:
            bool: 是否成功执行
        """
        # 限制位置范围
        position = max(0.0, min(0.044, position))

        # 选择 Action 客户端
        if arm == "left":
            action_client = self.left_gripper_action
        elif arm == "right":
            action_client = self.right_gripper_action
        else:
            self.get_logger().error(f"未知的机械臂: {arm}")
            return False

        # 等待 action 服务器
        self.get_logger().info(f"等待 {arm} 夹爪 action 服务器...")
        if not action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(f"{arm} 夹爪 action 服务器不可用")
            return False

        # 创建 goal 消息
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(
            f"发送 {arm} 夹爪命令: 目标位置={position:.4f}, 最大力度={max_effort:.1f}"
        )

        # 发送 goal
        send_goal_future = action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{arm} 夹爪 goal 被拒绝")
            return False

        self.get_logger().info(f"{arm} 夹爪 goal 已接受，等待执行...")

        # 等待执行结果
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result()
        if result:
            status = result.result
            self.get_logger().info(
                f"{arm} 夹爪执行完成: 位置={status.position:.4f}, "
                f"到达目标={status.reached_goal}"
            )
            return status.reached_goal
        else:
            self.get_logger().error(f"{arm} 夹爪执行失败")
            return False

    def execute_action(self, action):
        """
        执行单个动作

        Args:
            action: 动作字典，包含动作的所有参数

        Returns:
            bool: 是否成功执行
        """
        action_type = action.get("type")
        action_name = action.get("name", "未命名动作")

        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"开始执行动作: {action_name}")
        self.get_logger().info(f"{'='*50}")

        success = False

        if action_type == "arm":
            # 机械臂动作
            arm = action.get("arm")
            joints = action.get("joints")
            positions = action.get("positions")

            if not arm or not joints or not positions:
                self.get_logger().error("机械臂动作缺少必要参数")
                return False

            if len(joints) != len(positions):
                self.get_logger().error("关节名称和位置数量不匹配")
                return False

            self.send_joint_goal(arm, joints, positions)
            success = True
            # success = self.send_joint_goal(arm, joints, positions)

        elif action_type == "gripper":
            # 夹爪动作
            arm = action.get("arm")
            position = action.get("position")
            max_effort = action.get("max_effort", 20.0)

            if not arm or position is None:
                self.get_logger().error("夹爪动作缺少必要参数")
                return False

            success = self.control_gripper(arm, position, max_effort)

        else:
            self.get_logger().error(f"未知的动作类型: {action_type}")
            return False

        # 等待指定的时间
        wait_time = action.get("wait_time", 1.0)
        if success:
            self.get_logger().info(
                f"✓ 动作 '{action_name}' 执行成功，等待 {wait_time} 秒..."
            )
            time.sleep(wait_time)
        # else:
            # self.get_logger().error(f"✗ 动作 '{action_name}' 执行失败")

        return success

    def execute_sequence(self, config):
        """
        执行整个动作序列

        Args:
            config: 从 YAML 文件加载的配置字典

        Returns:
            bool: 是否所有动作都成功执行
        """
        actions = config.get("actions", [])

        if not actions:
            self.get_logger().error("配置文件中没有定义动作")
            return False

        self.get_logger().info(f"\n总共有 {len(actions)} 个动作待执行")
        self.get_logger().info("按 Ctrl+C 可随时终止执行\n")

        success_count = 0
        failed_count = 0

        for i, action in enumerate(actions, 1):
            action_name = action.get("name", f"动作 {i}")
            self.get_logger().info(f"\n[{i}/{len(actions)}] 准备执行: {action_name}")

            try:
                if self.execute_action(action):
                    success_count += 1
                else:
                    failed_count += 1
                    # 动作失败，询问是否继续
                    self.get_logger().warn(f"\n动作 '{action_name}' 执行失败！")
                    response = input("是否继续执行下一个动作？(y/n，默认 y): ").strip().lower()
                    if response == 'n' or response == 'no':
                        self.get_logger().info("用户选择停止执行")
                        break
                    else:
                        self.get_logger().info("继续执行下一个动作...")
                        time.sleep(1.0)

            except KeyboardInterrupt:
                self.get_logger().info("\n用户中断执行")
                raise
            except Exception as e:
                self.get_logger().error(f"执行动作时发生错误: {e}")
                failed_count += 1
                # 异常情况也询问是否继续
                response = input("是否继续执行下一个动作？(y/n，默认 y): ").strip().lower()
                if response == 'n' or response == 'no':
                    self.get_logger().info("用户选择停止执行")
                    break

        # 总结
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info("动作序列执行完成")
        self.get_logger().info(f"{'='*50}")
        self.get_logger().info(f"总动作数: {len(actions)}")
        self.get_logger().info(f"成功: {success_count}")
        self.get_logger().info(f"失败: {failed_count}")
        self.get_logger().info(f"{'='*50}\n")

        return failed_count == 0


def load_yaml_config(yaml_path):
    """
    从 YAML 文件加载配置

    Args:
        yaml_path: YAML 文件路径

    Returns:
        dict: 配置字典
    """
    if not os.path.exists(yaml_path):
        print(f"错误: 配置文件不存在: {yaml_path}")
        return None

    try:
        with open(yaml_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f)
        return config
    except Exception as e:
        print(f"错误: 无法加载 YAML 文件: {e}")
        return None


def print_usage():
    """打印使用说明"""
    print("\n=== OpenArm 动作序列自动执行 ===")
    print("\n使用方法:")
    print("  python3 run_actions.py <yaml_file>")
    print("\n参数说明:")
    print("  <yaml_file>  : 动作序列配置文件路径")
    print("\n示例:")
    print("  python3 run_actions.py config/demo_actions.yaml")
    print("  python3 run_actions.py /path/to/my_actions.yaml")
    print("\n说明:")
    print("  - 程序会按照 YAML 文件中定义的顺序依次执行所有动作")
    print("  - 支持机械臂关节运动和夹爪控制")
    print("  - 每个动作可以设置执行后的等待时间")
    print("  - 按 Ctrl+C 可随时终止执行")
    print()


def main(args=None):
    # 检查命令行参数
    if len(sys.argv) < 2:
        print_usage()
        return

    yaml_path = sys.argv[1]

    # 加载配置文件
    print(f"\n正在加载配置文件: {yaml_path}")
    config = load_yaml_config(yaml_path)
    if config is None:
        return

    actions = config.get("actions", [])
    print(f"成功加载配置，共有 {len(actions)} 个动作\n")

    # 初始化 ROS2
    rclpy.init(args=args)

    try:
        # 创建执行器
        executor = ActionSequenceExecutor()

        # 等待节点启动
        print("等待节点启动...")
        time.sleep(2)

        print("\n请确保已经启动了以下节点：")
        print("1. ros2 launch openarm_bimanual_moveit_config demo.launch.py")
        print(
            "2. ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=left_arm"
        )
        print(
            "3. ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=right_arm"
        )
        print("\n按 Enter 开始执行，或 Ctrl+C 取消...")
        input()

        # 执行动作序列
        success = executor.execute_sequence(config)

        if success:
            print("\n✓ 所有动作执行成功！")
        else:
            print("\n✗ 部分动作执行失败，请查看日志")

    except KeyboardInterrupt:
        print("\n\n用户中断执行")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback

        traceback.print_exc()
    finally:
        # 清理
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
