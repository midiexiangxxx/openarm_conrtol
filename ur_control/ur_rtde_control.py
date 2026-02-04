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
# joint_q_initial = [192 * rad, -120 * rad, -78 * rad,
#                     -53 * rad, 90 * rad, 8 * rad]
position_init=[187.41 * rad,-82.72 * rad,-105.28 * rad,-80.08 * rad,87.38 * rad,8.61 * rad]

joint_speed = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# speed of ur10e
joint_q_speed = 0.2

rtde_c.moveJ(position_init, joint_q_speed, acceleration)

