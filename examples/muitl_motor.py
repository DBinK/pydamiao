import math
import time

from pydamiao import ControlMode, MotorManager, MotorReg, MotorType, SerialBus

# 创建串口总线和多电机管理器
bus = SerialBus("COM3", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)
motor1 = manager.add_motor(MotorType.DM4310, 0x06, 0x12)
motor2 = manager.add_motor(MotorType.DM4310, 0x05, 0x12)

# 读取和修改电机参数示例
if motor1.set_mode(ControlMode.POS_VEL).is_ok:
    print("switch POS_VEL success")
if motor2.set_mode(ControlMode.VEL).is_ok:
    print("switch VEL success")

print("sub_ver:", motor1.read_param(MotorReg.sub_ver).unwrap())
print("Gr:", motor1.read_param(MotorReg.Gr).unwrap())
print("PMAX:", motor1.read_param(MotorReg.PMAX).unwrap())
print("MST_ID:", motor1.read_param(MotorReg.MST_ID).unwrap())
print("VMAX:", motor1.read_param(MotorReg.VMAX).unwrap())
print("TMAX:", motor1.read_param(MotorReg.TMAX).unwrap())

print("Motor2:")
print("PMAX:", motor2.read_param(MotorReg.PMAX).unwrap())
print("MST_ID:", motor2.read_param(MotorReg.MST_ID).unwrap())
print("VMAX:", motor2.read_param(MotorReg.VMAX).unwrap())
print("TMAX:", motor2.read_param(MotorReg.TMAX).unwrap())

# 保存并使能电机
motor1.save_params()
motor2.save_params()
manager.enable_all()

# 控制电机运动示例
i = 0
while i < 10000:
    q = math.sin(time.time())
    i += 1
    motor1.set_pos_vel(q * 8, 3)
    motor2.set_velocity(8 * q)

    time.sleep(0.001)

motor2.set_velocity(0.0)

# 语句结束关闭串口
bus.close()
