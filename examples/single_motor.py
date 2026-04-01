import math
import time

from pydamiao import ControlMode, Motor, MotorReg, MotorType, SerialBus

# 创建串口总线和电机对象
bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
motor = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

# 读取和修改电机参数示例
if motor.set_mode(ControlMode.POS_VEL):
    print(f"switch POS_VEL success, current mode: {motor.control_mode}")

print("PMAX:", motor.read_param(MotorReg.PMAX).value)
print("MST_ID:", motor.read_param(MotorReg.MST_ID).value)
print("VMAX:", motor.read_param(MotorReg.VMAX).value)
print("TMAX:", motor.read_param(MotorReg.TMAX).value)
print("TIMEOUT:", motor.read_param(MotorReg.TIMEOUT).value)

# 推荐设置一下超时, 这样在我们停止发送控制命令时, 电机不会不受控地继续运动
motor.write_param(MotorReg.TIMEOUT, 1000)
print("TIMEOUT:", motor.read_param(MotorReg.TIMEOUT).value)

# 使能电机
motor.clear_error()  # 如果启用了超时, 电机可能会因为超时而停止, 所以我们先清除错误
motor.enable()

# 控制电机运动示例
for _ in range(1000):
    vel = math.sin(time.time())
    motor.set_velocity(vel)

    print(f"{motor.rotor_temp=}, {motor.mos_temp=}, {motor.fault=}")
    # print(f"\r{motor.slave_id=}, {motor.control_mode=}, {motor.pos:.2f}, {motor.vel:.2f}, {motor.torque:.2f}{''*5}", end="")

    time.sleep(0.01)

# motor.disable()