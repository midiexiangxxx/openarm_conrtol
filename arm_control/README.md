# UR Control with rtde

1. 启动ros_ur_driver:
ros2 launch ur_robot_driver ur_control.launch.py \ ur_type:=ur10e \ robot_ip:=192.168.56.2 \ reverse_port:=50002 \ launch_rviz:=false

2. 启动moveit rviz可视化界面
ros2 launch ur_moveit_config ur_moveit.launch.py \ ur_type:=ur10e \ launch_rviz:=true \

3. conda 激活环境 openarm，并运行ur_rtde_control.py
