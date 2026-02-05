#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 示教角度记录
# position_init=[187.41,-82.72,-105.28,-80.08,87.38,8.61]
# posion_up=[184.82,-135.03,-49.29,-81.51,87.34,8.61]
# posion_close=[185.38,-139.31,-50.80,-77.62,84.49,8.61]
# posion_reach=[185.52,-138.66,-53.36,-74.83,84.21,8.59]
# posion_up_box=[170.54,-126.17,-67.92,-74.80,84.24,8.59]
# posion_in_box=[170.53,-128.03,-76.89,-63.18,84.24,8.59]
# posion_out_box=[170.54,-126.17,-67.92,-74.80,84.24,8.59]

import rtde_control
import rtde_receive
import math
import time

# address of ur10e
rtde_c = rtde_control.RTDEControlInterface("192.168.56.2")
rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.2")

# 用于判断是否启动交互
rad = math.pi / 180.0

acceleration = 0.1
dt = 1.0 / 500  # 2ms
# 定义所有位置（角度转弧度）
position_init = [187.41 * rad, -82.72 * rad, -105.28 * rad, -80.08 * rad, 87.38 * rad, 8.61 * rad]
position_up = [184.82 * rad, -135.03 * rad, -49.29 * rad, -81.51 * rad, 87.34 * rad, 8.61 * rad]
position_close = [183.01 * rad, -139.71 * rad, -49.74 * rad, -77.79 * rad, 91.63 * rad, 8.52 * rad]
position_reach = [183.01 * rad, -140.18 * rad, -51.06 * rad, -76.00 * rad, 91.63 * rad, 8.52 * rad]
position_up_box = [170.54 * rad, -126.17 * rad, -67.92 * rad, -74.80 * rad, 84.24 * rad, 8.59 * rad]
position_in_box = [170.53 * rad, -128.03 * rad, -76.89 * rad, -63.18 * rad, 84.24 * rad, 8.59 * rad]
position_out_box = [170.54 * rad, -126.17 * rad, -67.92 * rad, -74.80 * rad, 84.24 * rad, 8.59 * rad]

joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# speed of ur10e
joint_q_speed = 0.2
joint_q_speed_slow = 0.02  # 慢速，用于close到reach阶段
blend_radius = 0.05  # 混合半径，实现柔顺过渡

def test():

    # rtde_c.moveJ(position_close, joint_q_speed, acceleration)
    time.sleep(0.5)
    rtde_c.moveJ(position_init, joint_q_speed, acceleration)

def main():
    # 从position_init到position_close使用路径混合，实现柔顺连续运动
    # 路径格式：[q1, q2, q3, q4, q5, q6, speed, acceleration, blend_radius]
    path_smooth = [
        position_init + [joint_q_speed, acceleration, blend_radius],
        position_up + [joint_q_speed, acceleration, blend_radius],
        position_close + [joint_q_speed, acceleration, 0.0],  # 最后一个点blend=0，完全停止
    ]

    print("Moving smoothly from position_init to position_close...")
    rtde_c.moveJ(path_smooth)

    # ========== 力控模块：从position_close向position_reach方向缓慢接近，检测1N接触力 ==========
    force_threshold = 2 # 力阈值 1N
    approach_joint_speed = 0.01  # 接近关节速度 rad/s
    force_timeout = 30.0  # 超时保护 30s

    # 清零力传感器，消除自重等初始偏置
    rtde_c.zeroFtSensor()
    time.sleep(0.5)

    # 计算从position_close到position_reach的关节空间运动方向
    direction = [position_reach[i] - position_close[i] for i in range(6)]
    dir_magnitude = math.sqrt(sum(d ** 2 for d in direction))
    speed_vector = [d / dir_magnitude * approach_joint_speed for d in direction]

    print("Force control: approaching surface...")
    t_start = time.time()
    surface_contacted = False

    while time.time() - t_start < force_timeout:
        # 读取当前TCP力/力矩
        tcp_force = rtde_r.getActualTCPForce()
        force_magnitude = math.sqrt(tcp_force[0] ** 2 + tcp_force[1] ** 2 + tcp_force[2] ** 2)

        if force_magnitude >= force_threshold:
            rtde_c.speedStop()
            surface_contacted = True
            print(f"Surface contacted! Force: {force_magnitude:.2f}N")
            break

        # 沿close→reach方向缓慢移动
        rtde_c.speedJ(speed_vector, acceleration, dt)
        time.sleep(dt)

    if not surface_contacted:
        rtde_c.speedStop()
        print("Timeout: surface not detected within 30s")

    time.sleep(10.0)

    print("Moving to position_up...")
    rtde_c.moveJ(position_up, joint_q_speed, acceleration)
    time.sleep(0.1)

    print("Moving to position_up_box...")
    rtde_c.moveJ(position_up_box, joint_q_speed, acceleration)
    time.sleep(0.1)

    print("Moving to position_in_box...")
    rtde_c.moveJ(position_in_box, joint_q_speed, acceleration)
    time.sleep(3.0)

    print("Moving to position_out_box...")
    rtde_c.moveJ(position_out_box, joint_q_speed, acceleration)
    time.sleep(0.5)

    print("Moving to position_init...")
    rtde_c.moveJ(position_init, joint_q_speed, acceleration)

    print("All positions reached!")


if __name__ == "__main__":
    # test()
    main()
