#!/bin/bash

# Launch both left and right arm joint controllers
# This script should be run after the MoveIt demo launch

echo "启动左臂和右臂关节控制器..."
echo ""
echo "请确保已经运行："
echo "  ros2 launch openarm_bimanual_moveit_config demo.launch.py"
echo ""

# Launch left arm controller in background
echo "启动左臂控制器..."
ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=left_arm &
LEFT_PID=$!

sleep 2

# Launch right arm controller in background
echo "启动右臂控制器..."
ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=right_arm &
RIGHT_PID=$!

echo ""
echo "两个控制器已启动！"
echo "  左臂控制器 PID: $LEFT_PID"
echo "  右臂控制器 PID: $RIGHT_PID"
echo ""
echo "现在可以运行测试脚本："
echo "  ./test_joint_controller.py"
echo ""
echo "按 Ctrl+C 停止所有控制器..."

# Wait for Ctrl+C
trap "echo ''; echo '停止控制器...'; kill $LEFT_PID $RIGHT_PID 2>/dev/null; exit 0" SIGINT SIGTERM

# Wait for both processes
wait $LEFT_PID $RIGHT_PID
