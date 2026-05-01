import math
import time

from pydamiao import Motor, MotorManager, ControlMode, MotorType, MotorReg, SerialBus

# 初始化所有对象
bus = SerialBus("COM3", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)

motor1 = Motor(bus, MotorType.DM4310, 0x06, 0x12)
motor2 = Motor(bus, MotorType.DM4310, 0x05, 0x12)

# 注册电机到管理器
manager.register(motor1)
manager.register(motor2)

# 统一控制电机
manager.clean_error_all()
manager.disable_all()
manager.set_mode_all(ControlMode.MIT)

# 查看电机参数
for id, motor in manager.motors.items():
    print(f"电机 ID {id}:")
    print("CTRL_MODE:", motor.read_param(MotorReg.CTRL_MODE).value)
    print("MST_ID:", motor.read_param(MotorReg.MST_ID).value)

# 单独控制电机
if not motor1.set_mode(ControlMode.POS_VEL).is_ok:
    print("motor1 切换到 POS_VEL 失败")
    
if not motor2.set_mode(ControlMode.VEL).is_ok:
    print("motor2 切换到 VEL 失败")

# 运动测试
for _ in range(1000):
    q = math.sin(time.time())
    motor1.set_pos_vel(q * 8, 3)
    motor2.set_velocity(8 * q)
    time.sleep(0.001)

# 程序结束前, 记得失能所有电机 (虽然有自动失能兜底, 但是推荐养成好习惯)
manager.disable_all()
bus.close()