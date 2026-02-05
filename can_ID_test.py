import openarm_can as oa
import time

# 扫描 can1 上的电机
openarm = oa.OpenArm("can1", True)

# 尝试不同的 ID 组合
for motor_id in range(1, 16):  # 扫描 ID 0x01 - 0x0F
    print(f"\nTrying motor ID 0x{motor_id:02X}...")
    try:
        openarm2 = oa.OpenArm("can1", True)
        openarm2.init_arm_motors(
            [oa.MotorType.DM4310],  # 尝试 J4-J7 的电机类型
            [motor_id],             # send ID
            [motor_id + 0x10]       # recv ID (通常是 send_id + 0x10)
        )
        openarm2.set_callback_mode_all(oa.CallbackMode.STATE)
        openarm2.refresh_all()
        time.sleep(0.05)
        openarm2.recv_all()
        
        m = openarm2.get_arm().get_motors()[0]
        if m.get_position() != 0 or m.get_velocity() != 0 or m.get_torque() != 0:
            print(f"  ✓ Found motor at ID 0x{motor_id:02X}! pos={m.get_position():.4f}")
        else:
            print(f"  ✗ No response")
    except Exception as e:
        print(f"  Error: {e}")

