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