import math
import time

from pydamiao import MotorManager, MotorReg, MotorType, SerialBus
from pydamiao.structs import ControlMode

# 创建串口总线和多电机管理器
bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)

# 添加电机
motor1 = manager.add_motor(MotorType.DM4310, 0x06, 0x16, name="wrist_3")
motor2 = manager.add_motor(MotorType.DM4310, 0x05, 0x15, name="wrist_2")
motor3 = manager.add_motor(MotorType.DM4310, 0x04, 0x14, name="wrist_1")

# 清除电机错误
manager.clean_error_all()
manager.disable_all()

# 读取和修改电机参数示例
ret = [
    motor1.set_mode(ControlMode.MIT),
    motor2.set_mode(ControlMode.POS_VEL),
    motor3.set_mode(ControlMode.MIT),
]
print(f"设置控制模式结果: {[r.is_ok for r in ret]}, 当前模式: {[motor1.control_mode, motor2.control_mode, motor3.control_mode]}")

print("\nMotor1:")
print("sub_ver:", motor1.read_param(MotorReg.sub_ver).value)
print("Gr:", motor1.read_param(MotorReg.Gr).value)
print("CTRL_MODE:", motor1.read_param(MotorReg.CTRL_MODE).value)
print("MST_ID:", motor1.read_param(MotorReg.MST_ID).value)

print("\nMotor2:")
print("CTRL_MODE:", motor2.read_param(MotorReg.CTRL_MODE).value)
print("MST_ID:", motor2.read_param(MotorReg.MST_ID).value)

print("\nMotor3:")
print("CTRL_MODE:", motor3.read_param(MotorReg.CTRL_MODE).value)
print("MST_ID:", motor3.read_param(MotorReg.MST_ID).value)

# 使能电机
manager.clean_error_all()
manager.enable_all()
manager.set_zero_all()

# 控制电机运动示例
timeout = 5000
now = time.time()

while (time.time() - now) < timeout:

    sin = math.sin(time.time())

    motor1.set_mit(sin * 0.2, 0, 15.5, 1.0, 0.0)
    motor2.set_pos_vel(sin * 0.1, 3)
    motor3.set_mit(sin * 0.2, 0, 15.5, 1.0, 0.0)

    # motor1.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
    # motor2.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)
    # motor3.set_mit(0.0, 0.0, 0.0, 0.0, 0.0)

    print(
        f"\r{motor1.name=}, {motor1.pos=:.6f} {motor1.pos*4=:.6f}, {motor3.pos:.6f} {' ' * 8}",
        end="",
    )

    time.sleep(0.001)

# 测试结束关闭电机
manager.disable_all()  # 程序结束前, 推荐手动失能所有电机
