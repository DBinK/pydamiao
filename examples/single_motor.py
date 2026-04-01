import math
import time

from pydamiao import ControlMode, Motor, MotorReg, MotorType, SerialBus

# 创建串口总线和电机对象
bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

breakpoint()

# 读取和修改电机参数示例
if motor.set_mode(ControlMode.POS_VEL):
    print(f"switch POS_VEL success, current mode: {motor.control_mode}")

print("PMAX:", motor.read_param(MotorReg.PMAX).value)
print("MST_ID:", motor.read_param(MotorReg.MST_ID).value)
print("VMAX:", motor.read_param(MotorReg.VMAX).value)
print("TMAX:", motor.read_param(MotorReg.TMAX).value)

# 保存并使能电机
motor.save_params()
motor.enable()

# 控制电机运动示例
for _ in range(100):
    vel = math.sin(time.time())
    motor.set_velocity(vel)

    print(f"\r{motor.slave_id=}, {motor.control_mode=}, {motor.pos:.2f}, {motor.vel:.2f}, {motor.torque:.2f}{''*5}", end="")

    time.sleep(0.01)

