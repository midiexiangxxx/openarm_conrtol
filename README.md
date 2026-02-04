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
```
ros2 launch openarm_bringup openarm.bimanual.read_only.launch.py
```

### run sequence plan
```
ros2 launch openarm_bimanual_moveit_config demo.launch.py
./interface/openarm_joint_controller/scripts/launch_both_controllers.sh
cd interface/openarm_joint_controller/scripts && ./run_actions.py ../config/simple_test.yaml
```


1. source ~/huadian/openarm_conrtol/install/setup.zsh
2. ros2 launch openarm_bimanual_moveit_config demo.launch.py 
3. cd ./home/ubuntu/huadian/openarm_conrtol/interface/openarm_joint_controller/scripts/ && ./launch_both_controllers.sh
4. cd ./home/ubuntu/huadian/openarm_conrtol/interface/openarm_joint_controller/scripts/ && python3 run_actions.py ../config/dual_heatgun.yaml
Then **Push Enter** to Run