import math
import time

from pydamiao import ControlMode, Motor, MotorReg, MotorType, SerialBus

# 创建串口总线和电机对象
bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
motor1 = Motor(bus=bus, motor_type=MotorType.DM4310, slave_id=0x06, master_id=0x16)

# 读取和修改电机参数示例
if motor1.set_mode(ControlMode.POS_VEL):
    print("switch POS_VEL success")

print("sub_ver:", motor1.read_param(MotorReg.sub_ver).value)
print("Gr:", motor1.read_param(MotorReg.Gr).value)
print("PMAX:", motor1.read_param(MotorReg.PMAX).value)
print("MST_ID:", motor1.read_param(MotorReg.MST_ID).value)
print("VMAX:", motor1.read_param(MotorReg.VMAX).value)
print("TMAX:", motor1.read_param(MotorReg.TMAX).value)

# 保存并使能电机
motor1.save_params()
motor1.enable()

# 控制电机运动示例
for _ in range(1000):
    pos = math.sin(time.time())
    motor1.set_pos_vel(pos, 0.5)
    print("Motor1:", "POS:", motor1.pos, "VEL:", motor1.vel, "TORQUE:", motor1.torque)
    time.sleep(0.01)

# 语句结束关闭串口
bus.close()
