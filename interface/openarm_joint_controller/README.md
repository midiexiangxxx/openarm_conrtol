# OpenArm Joint Controller

ROS2 包，用于通过 MoveIt 规划器接收关节状态目标并控制 OpenArm 机械臂运动。

## 功能特性

- 接收 `sensor_msgs/JointState` 消息作为目标关节状态
- 使用 MoveIt 规划器自动规划轨迹
- 支持左臂和右臂独立控制（通过独立话题）
- 可配置的速度和加速度缩放因子
- 可配置的规划时间和执行超时
- 提供便捷的测试脚本和启动脚本

## 依赖

- ROS2 Jazzy
- MoveIt2
- OpenArm 描述包
- OpenArm MoveIt 配置包

## 编译

```bash
cd /home/run/openarm/openarm_control/interface
colcon build --packages-select openarm_joint_controller
source install/setup.bash
```

## 使用方法

### 1. 首先启动 MoveIt demo（包含硬件和规划器）

```bash
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```

### 2. 启动关节控制器

#### 选项 A: 使用便捷脚本（推荐）

同时启动左臂和右臂控制器：

```bash
cd /home/run/openarm/openarm_control/interface/openarm_joint_controller/scripts
./launch_both_controllers.sh
```

#### 选项 B: 手动启动

在不同的终端中分别启动：

**控制左臂：**
```bash
source /home/run/openarm/openarm_control/interface/install/setup.bash
ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=left_arm
```

**控制右臂：**
```bash
source /home/run/openarm/openarm_control/interface/install/setup.bash
ros2 launch openarm_joint_controller joint_controller.launch.py planning_group:=right_arm
```

### 3. 使用测试脚本（推荐）

```bash
cd /home/run/openarm/openarm_control/interface/openarm_joint_controller/scripts
./test_joint_controller.py
```

测试脚本提供交互式菜单，可以测试不同的关节配置。

### 4. 手动发送关节目标

**示例 1：移动左臂到指定关节位置**
```bash
ros2 topic pub --once /left_arm/joint_goal sensor_msgs/msg/JointState "{
  name: ['openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3', 'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6', 'openarm_left_joint7'],
  position: [0.5, 0.3, 0.0, 1.0, 0.0, 0.5, 0.0]
}"
```

**示例 2：移动右臂到指定关节位置**
```bash
ros2 topic pub --once /right_arm/joint_goal sensor_msgs/msg/JointState "{
  name: ['openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3', 'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6', 'openarm_right_joint7'],
  position: [0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0]
}"
```

**示例 3：只移动部分关节**
```bash
ros2 topic pub --once /left_arm/joint_goal sensor_msgs/msg/JointState "{
  name: ['openarm_left_joint1', 'openarm_left_joint2'],
  position: [0.8, -0.5]
}"
```

## Launch 参数

| 参数名 | 默认值 | 描述 |
|--------|--------|------|
| `planning_group` | `left_arm` | MoveIt 规划组名称（`left_arm` 或 `right_arm`）|
| `execution_timeout` | `10.0` | 轨迹执行超时时间（秒）|
| `planning_time` | `5.0` | 最大规划时间（秒）|
| `max_velocity_scaling_factor` | `0.1` | 最大速度缩放因子（0.0-1.0）|
| `max_acceleration_scaling_factor` | `0.1` | 最大加速度缩放因子（0.0-1.0）|

### 调整速度示例

```bash
ros2 launch openarm_joint_controller joint_controller.launch.py \
  planning_group:=left_arm \
  max_velocity_scaling_factor:=0.5 \
  max_acceleration_scaling_factor:=0.5
```

## 话题接口

### 订阅的话题

每个控制器实例根据其 `planning_group` 参数订阅不同的话题：

- `/left_arm/joint_goal` (`sensor_msgs/msg/JointState`): 左臂控制器接收目标关节状态
- `/right_arm/joint_goal` (`sensor_msgs/msg/JointState`): 右臂控制器接收目标关节状态

这种设计允许左右臂控制器同时运行而不会相互干扰。

### 发布的话题

该节点通过 MoveIt 接口控制机械臂，相关的轨迹执行信息可以通过 MoveIt 的标准话题查看。

## 节点架构

```
┌────────────────────────────────┐      ┌────────────────────────────────┐
│ joint_state_controller (左臂)  │      │ joint_state_controller (右臂)  │
│                                │      │                                │
│  订阅: /left_arm/joint_goal    │      │  订阅: /right_arm/joint_goal   │
│                                │      │                                │
│  ┌──────────────────────────┐  │      │  ┌──────────────────────────┐  │
│  │ MoveGroupInterface       │  │      │  │ MoveGroupInterface       │  │
│  │ (planning_group=left_arm)│  │      │  │ (planning_group=right_arm)│ │
│  └──────────┬───────────────┘  │      │  └──────────┬───────────────┘  │
└─────────────┼──────────────────┘      └─────────────┼──────────────────┘
              │                                       │
              └───────────────┬───────────────────────┘
                              ▼
                      ┌───────────────┐
                      │   MoveGroup   │
                      │  (move_group) │
                      └───────┬───────┘
                              │
                              ▼
                      ┌───────────────┐
                      │  Controllers  │
                      └───────────────┘
```

## 关节限制

请确保发送的关节位置在以下范围内（已添加安全余量）：

**左臂/右臂关节限制：**
- `joint1`: [-1.446, 3.541] rad
- `joint2`: [-1.795, 1.795] rad
- `joint3`: [-1.621, 1.621] rad
- `joint4`: [-0.05, 2.493] rad
- `joint5`: [-1.621, 1.621] rad
- `joint6`: [-0.835, 0.835] rad
- `joint7`: [-1.621, 1.621] rad

## 故障排除

### 规划失败

1. **检查关节限制**: 确保目标关节位置在允许范围内
2. **增加规划时间**: 设置更大的 `planning_time` 参数
3. **检查起始状态**: 确保当前机械臂状态有效且无碰撞

### 执行失败

1. **检查控制器状态**: 确保 ros2_control 控制器正常运行
   ```bash
   ros2 control list_controllers
   ```
2. **检查硬件连接**: 确保 CAN 接口连接正常
3. **增加执行超时**: 设置更大的 `execution_timeout` 参数

### 无加速度限制错误

如果看到 "No acceleration limit was defined" 错误，请确保已经更新了 `joint_limits.yaml` 文件并重新编译了 MoveIt 配置包。

### 控制器未接收到消息

确保：
1. MoveIt demo 正在运行
2. 控制器已启动并针对正确的规划组
3. 发布到正确的话题：
   - 左臂：`/left_arm/joint_goal`
   - 右臂：`/right_arm/joint_goal`
4. 关节名称与规划组匹配

### 多个控制器冲突

如果同时运行多个控制器，确保：
- 每个控制器使用不同的 `planning_group` 参数
- 测试脚本会自动发布到正确的话题
- 不要手动发布到错误的话题（例如，不要将左臂关节发送到右臂话题）

## 开发者信息

- **包名**: openarm_joint_controller
- **节点名**: joint_state_controller
- **编程语言**: C++
- **ROS2 版本**: Jazzy

## 许可证

Apache-2.0
