import openarm_can as oa
import time

openarm = oa.OpenArm("can1", True)
openarm.init_arm_motors(
    [oa.MotorType.DM8009, oa.MotorType.DM8009, oa.MotorType.DM4340,
     oa.MotorType.DM4340, oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310],
    [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
    [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
)
openarm.set_callback_mode_all(oa.CallbackMode.STATE)

# 先 refresh（ROS2 在 enable 前会做这一步）
openarm.refresh_all()
time.sleep(0.1)
openarm.recv_all()

print("Enabling...")
openarm.enable_all()
time.sleep(0.1)
openarm.recv_all()

arm = openarm.get_arm()
motors = arm.get_motors()
for i, m in enumerate(motors):
    print(f"Motor {i}: pos={m.get_position():.4f}, vel={m.get_velocity():.4f}, torque={m.get_torque():.4f}")

openarm.disable_all()

