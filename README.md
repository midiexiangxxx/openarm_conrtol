# Openarm Control

## Setup

1. First, You should follow the official hardware&software setup guide [openarm_can](https://github.com/enactic/openarm_can?tab=readme-ov-file#quick-start)

2. build
```
CMAKE_PREFIX_PATH=<OPENARM_CAN_INSTALL_PATH>:$CMAKE_PREFIX_PATH colcon build --symlink-install --cmake-args "-DCMAKE_EXPORT_COMPILE_COMMANDS=1"
```

## Usage

### moveit
```
# launch openarm_ros2 moveit 
ros2 launch openarm_bimanual_moveit_config demo.launch.py
# launch manipulator joint state controller using script
./interface/openarm_joint_controller/scripts/launch_both_controllers.sh
# run main function
./interface/openarm_joint_controller/scripts/test_joint_controller.py
```
### record joint
1. 
```
ros2 launch openarm_bringup openarm.bimanual.read_only.launch.py
```
2. 
```
ros2 topic echo /joint_states_ordered --once
```
you should follow the name and the corresponding value to record the joint_states

### run sequence plan
```
ros2 launch openarm_bimanual_moveit_config demo.launch.py
./interface/openarm_joint_controller/scripts/launch_both_controllers.sh
cd interface/openarm_joint_controller/scripts && ./run_actions.py ../config/simple_test.yaml
```


1. openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000e
2. openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000e
3. source ~/huadian/openarm_conrtol/install/setup.zsh
4. ros2 launch openarm_bimanual_moveit_config demo.launch.py 
5. cd ./home/ubuntu/huadian/openarm_conrtol/interface/openarm_joint_controller/scripts/ && ./launch_both_controllers.sh
6. cd ./home/ubuntu/huadian/openarm_conrtol/interface/openarm_joint_controller/scripts/ && python3 run_actions.py ../config/dual_heatgun.yaml
Then **Push Enter** to Run



# 26.2.5版本
## 注意事项
1. openarm的can线不要拔，影响电机控制顺序。
2. 上电包括电源线和can-usb线。每次上电需运行openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000e以及openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000e配置can-fd。由于是全局环境变量直接运行即可。
3. 运行过程中可能发生碰撞导致下电，此时红灯闪烁。这时拔掉电源线和can线重插即可。再次运行上面两条命令激活canfd.

## 运行moveit
运行前，为了安全，将机械臂摆到零位。moveit启动后会自动回零，防止力大发生碰撞。
source ~/huadian/openarm_conrtol/install/setup.zsh
ros2 launch openarm_bimanual_moveit_config demo.launch.py 

## 拖动示教、录制数据
1. 关闭moveit程序
2. ros2 launch openarm_bringup openarm.bimanual.read_only.launch.py
3. 拖动到理想位置
4. ros2 topic echo /joint_states_ordered --once
5. 看position字段，值的顺序和joint name的顺序一致的，右臂就是前7个，左臂就是第9-15个。

## 组合动作，运行
1. 关闭示教程序
2. 起moveit
3. cd ./home/ubuntu/huadian/openarm_conrtol/interface/openarm_joint_controller/scripts/ && ./launch_both_controllers.sh
4. cd ./home/ubuntu/huadian/openarm_conrtol/interface/openarm_joint_controller/scripts/ && python3 run_actions.py ../config/dual_heatgun.yaml
run_actions.py会根据你的yaml配置里的程序来从上到下按顺序运行，只改yaml文件即可。见我写的yaml示例即可。
调试一个新的动作时，若不确定，应先将openarm移到空旷的位置进行运行。

