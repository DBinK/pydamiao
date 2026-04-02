import math
import time

from pydamiao import MotorManager, MotorReg, MotorType, SerialBus

# 创建串口总线和多电机管理器
bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)
motor1 = manager.add_motor(MotorType.DM4310, 0x06, 0x16, name="wrist_3")
motor2 = manager.add_motor(MotorType.DM4310, 0x05, 0x15, name="wrist_2")
motor3 = manager.add_motor(MotorType.DM4310, 0x04, 0x14, name="wrist_1")

# 读取和修改电机参数示例
print("Motor1:")
print("sub_ver:", motor1.read_param(MotorReg.sub_ver).value)
print("Gr:", motor1.read_param(MotorReg.Gr).value)
print("PMAX:", motor1.read_param(MotorReg.PMAX).value)
print("MST_ID:", motor1.read_param(MotorReg.MST_ID).value)
print("VMAX:", motor1.read_param(MotorReg.VMAX).value)
print("TMAX:", motor1.read_param(MotorReg.TMAX).value)

print("Motor2:")
print("PMAX:", motor2.read_param(MotorReg.PMAX).value)
print("MST_ID:", motor2.read_param(MotorReg.MST_ID).value)
print("VMAX:", motor2.read_param(MotorReg.VMAX).value)
print("TMAX:", motor2.read_param(MotorReg.TMAX).value)

# 使能电机
manager.clean_error_all()
manager.enable_all()
motor1.set_zero()

# 控制电机运动示例
timeout = 150
now = time.time()

while (time.time() - now) < timeout:

    sin = math.sin(time.time())

    motor1.set_mit(sin * 0.2, 0, 5.5, 1.0, 0.0)
    motor2.set_pos_vel(sin * 0.1, 3)
    motor3.set_mit(sin * 0.2, 0, 5.5, 1.0, 0.0)

    print(
        f"\r{motor1.name=}, {motor1.control_mode=}, {motor1.fault=}, {motor1.pos=:.2f}, {motor1.vel=:.2f}, {motor1.torque=:.2f}{' ' * 8}",
        end="",
    )

    # ret1 = motor1.refresh_state()
    # ret2 = motor2.refresh_state()
    # ret3 = motor3.refresh_state()

    # from rich import print
    # print(f"{ret1=}, {ret2=}, {ret3=}")

    time.sleep(0.001)

# 测试结束关闭电机
# manager.disable_all()  # 程序结束前, 推荐手动失能所有电机
