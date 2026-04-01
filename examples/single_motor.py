import math
import time

import serial
from pydamiao.types import MotorType, ControlMode, MotorReg
from pydamiao.motor import Motor, MotorController

# 创建电机对象和串口对象
motor1 = Motor(MotorType.DM4310, 0x06, 0x16)
serial_device = serial.Serial("COM9", 921600, timeout=0.5)
controller = MotorController(serial_device)
controller.add_motor(motor1)

# 读取和修改电机参数示例
if controller.switch_control_mode(motor1, ControlMode.POS_VEL):
    print("switch POS_VEL success")

print("sub_ver:", controller.read_motor_param(motor1, MotorReg.sub_ver))
print("Gr:", controller.read_motor_param(motor1, MotorReg.Gr))
# if motor_control.change_motor_param(motor1, DM_variable.KP_APR, 54):
#     print("write success")
print("PMAX:", controller.read_motor_param(motor1, MotorReg.PMAX))
print("MST_ID:", controller.read_motor_param(motor1, MotorReg.MST_ID))
print("VMAX:", controller.read_motor_param(motor1, MotorReg.VMAX))
print("TMAX:", controller.read_motor_param(motor1, MotorReg.TMAX))

# 保存并使能电机
controller.save_motor_param(motor1)
controller.enable(motor1)
controller.disable(motor1)

# 控制电机运动示例
i = 0
while i < 10:
    q = math.sin(time.time())
    i += 1
    # motor_control.control_pos_force(motor1, 10, 1000, 100)
    # motor_control.control_vel(motor1, q * 5)
    # motor_control.control_pos_vel(motor1, q * 8, 3)
    controller.refresh_motor_status(motor1)
    print("Motor1:", "POS:", motor1.pos, "VEL:", motor1.vel, "TORQUE:", motor1.torque)
    # motor_control.control_mit(motor2, 35, 0.1, 8 * q, 0, 0)

    time.sleep(0.01)
    # motor_control.control(motor3, 50, 0.3, q, 0, 0)

# 语句结束关闭串口
from rich import print
print(motor1)
serial_device.close()
