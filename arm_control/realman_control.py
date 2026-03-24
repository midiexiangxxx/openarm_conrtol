from Robotic_Arm.rm_robot_interface import *
import time

position_init = [-49.264, -3.728, 58.05, -1.936, 123.968, 9.82]
position_up = [-49.262, 10.211, 36.194, 1.874, 118.313, 9.989]
position_close= [-49.29, -35.635, 104.625, -0.16, 77.272, -0.46]
position_reach= [-49.286, -38.68, 118.786, -0.175, 66.155, -0.424]
position_up_box=[5.998, 5.818, 65.022, 12.442, 86.92, -31.47]
position_in_box=[6.03, 22.792, 67.722, 12.351, 78.704, -31.468]
position_check_box=[6.024, 14.216, 55.404, 4.313, 101.031, 35.682]
position_backway = [-24.415, -16.133, 67.381, 11.432, 122.921, 9.792]


# 实例化RoboticArm类
arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
# 创建机械臂连接
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)

# arm.rm_movej(position_backway, 50, 0, 0, 1)

# 关节阻塞运动
time.sleep(10)
arm.rm_movej(position_init, 50, 0, 0, 1)
arm.rm_movej(position_up, 50, 0, 0, 1)
arm.rm_movej(position_close, 30, 0, 0, 1)
time.sleep(1)
arm.rm_movej(position_reach, 5, 0, 0, 1)
time.sleep(20)
arm.rm_movej(position_close, 20, 0, 0, 1)
arm.rm_movej(position_up, 30, 0, 0, 1)
arm.rm_movej(position_up_box, 50, 0, 0, 1)
time.sleep(1)
arm.rm_movej(position_in_box, 10, 0, 0, 1)
arm.rm_movej(position_up_box, 30, 0, 0, 1)
arm.rm_movej(position_check_box, 50, 0, 0, 1)
time.sleep(10)
arm.rm_movej(position_backway, 50, 0, 0, 1)
arm.rm_movej(position_init, 50, 0, 0, 1)

arm.rm_delete_robot_arm()