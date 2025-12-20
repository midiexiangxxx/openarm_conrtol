# OpenArm 关节控制器快速启动指南

## 项目结构

```
/home/run/openarm/openarm_control/interface/openarm_joint_controller/
├── src/
│   └── joint_state_controller.cpp    # C++ 控制器节点
├── launch/
│   └── joint_controller.launch.py    # Launch 文件
├── scripts/
│   └── test_joint_controller.py      # 测试脚本
├── CMakeLists.txt
├── package.xml
├── README.md                          # 详细文档
└── QUICKSTART.md                      # 本文件
```

## 快速开始

### 1. 编译项目

```bash
cd /home/run/openarm/openarm_control/interface
colcon build --packages-select openarm_joint_controller --symlink-install
source install/setup.bash
```

### 2. 启动 MoveIt（终端1）

```bash
cd /home/run/openarm/ros2_ws.official
source install/setup.bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

**注意**：等待 MoveIt 完全启动，看到 "You can start planning now!" 消息。

### 3. 启动关节控制器（终端2）

**控制左臂：**
```bash
cd /home/run/openarm/openarm_control/interface
source install/setup.bash
ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=left_arm
```

**控制右臂：**
```bash
cd /home/run/openarm/openarm_control/interface
source install/setup.bash
ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=right_arm
```

### 4. 发送控制命令

#### 方式A：使用测试脚本（推荐，终端3）

```bash
cd /home/run/openarm/openarm_control/interface
source install/setup.bash
ros2 run openarm_joint_controller test_joint_controller.py
```

然后根据提示选择预设的测试场景。

#### 方式B：使用命令行直接发布

**左臂示例：**
```bash
ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{
  name: ['openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3', 'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6', 'openarm_left_joint7'],
  position: [0.5, 0.3, 0.0, 1.0, 0.0, 0.5, 0.0]
}"
```

**右臂示例：**
```bash
ros2 topic pub --once /joint_goal sensor_msgs/msg/JointState "{
  name: ['openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3', 'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6', 'openarm_right_joint7'],
  position: [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
}"
```

## 调整运动速度

如果觉得机械臂移动太慢，可以调整速度和加速度缩放因子：

```bash
ros2 launch openarm_joint_controller joint_controller.launch.py \
  planning_group:=left_arm \
  max_velocity_scaling_factor:=0.5 \
  max_acceleration_scaling_factor:=0.5
```

值的范围是 0.0-1.0：
- 0.1 = 10% 速度（默认，安全）
- 0.5 = 50% 速度（中等）
- 1.0 = 100% 速度（最快，小心）

## 检查系统状态

**查看控制器：**
```bash
ros2 control list_controllers
```

**查看话题：**
```bash
ros2 topic list
ros2 topic echo /joint_states
```

**查看当前关节位置：**
```bash
ros2 topic echo /joint_states --once
```

## 常见问题

### 1. 规划失败 - "Start state out of bounds"

**原因**：当前关节位置超出限制（通常是很小的误差）

**解决方案**：
- 关节限制已经在 MoveIt 配置中添加了余量
- 确保已经重新编译了 `openarm_bimanual_moveit_config`
- 如果还有问题，可以稍微调整目标位置

### 2. 规划失败 - "No acceleration limit"

**解决方案**：已经修复了 joint_limits.yaml，重新编译 MoveIt 配置：
```bash
cd /home/run/openarm/ros2_ws.official
colcon build --packages-select openarm_bimanual_moveit_config --symlink-install
```

### 3. 控制器未响应

**检查步骤**：
1. 确认 MoveIt 已完全启动
2. 确认控制器节点正在运行：`ros2 node list | grep joint_state_controller`
3. 确认话题连接正常：`ros2 topic info /joint_goal`

## 编程接口

### Python 示例

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

node = Node('my_controller')
pub = node.create_publisher(JointState, 'joint_goal', 10)

msg = JointState()
msg.name = ['openarm_left_joint1', 'openarm_left_joint2']
msg.position = [0.5, 0.3]
pub.publish(msg)
```

### C++ 示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

auto node = std::make_shared<rclcpp::Node>("my_controller");
auto pub = node->create_publisher<sensor_msgs::msg::JointState>("joint_goal", 10);

auto msg = sensor_msgs::msg::JointState();
msg.name = {"openarm_left_joint1", "openarm_left_joint2"};
msg.position = {0.5, 0.3};
pub->publish(msg);
```

## 下一步

查看完整文档了解更多功能：
```bash
cat /home/run/openarm/openarm_control/interface/openarm_joint_controller/README.md
```

## 技术支持

- 查看日志了解详细错误信息
- MoveIt 文档：https://moveit.ros.org/
- ROS2 文档：https://docs.ros.org/
