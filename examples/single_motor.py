import math
import time

import serial
from pydamiao.types import MotorType, ControlMode, MotorReg
from pydamiao.motor import Motor, MotorControl

# 创建电机对象和串口对象
motor1 = Motor(MotorType.DM4310, 0x05, 0x12)
serial_device = serial.Serial("COM3", 921600, timeout=0.5)
motor_control = MotorControl(serial_device)
motor_control.add_motor(motor1)

# 读取和修改电机参数示例
if motor_control.switch_control_mode(motor1, ControlMode.POS_VEL):
    print("switch POS_VEL success")

print("sub_ver:", motor_control.read_motor_param(motor1, MotorReg.sub_ver))
print("Gr:", motor_control.read_motor_param(motor1, MotorReg.Gr))
# if motor_control.change_motor_param(motor1, DM_variable.KP_APR, 54):
#     print("write success")
print("PMAX:", motor_control.read_motor_param(motor1, MotorReg.PMAX))
print("MST_ID:", motor_control.read_motor_param(motor1, MotorReg.MST_ID))
print("VMAX:", motor_control.read_motor_param(motor1, MotorReg.VMAX))
print("TMAX:", motor_control.read_motor_param(motor1, MotorReg.TMAX))

# 保存并使能电机
motor_control.save_motor_param(motor1)
motor_control.enable(motor1)

# 控制电机运动示例
i = 0
while i < 10000:
    q = math.sin(time.time())
    i += 1
    # motor_control.control_pos_force(motor1, 10, 1000, 100)
    # motor_control.control_vel(motor1, q * 5)
    motor_control.control_pos_vel(motor1, q * 8, 30)
    # print("Motor1:", "POS:", motor1.get_position(), "VEL:", motor1.get_velocity(), "TORQUE:", motor1.get_torque())
    # motor_control.control_mit(motor2, 35, 0.1, 8 * q, 0, 0)

    time.sleep(0.001)
    # motor_control.control(motor3, 50, 0.3, q, 0, 0)

# 语句结束关闭串口
serial_device.close()
