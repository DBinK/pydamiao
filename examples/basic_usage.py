import math
import time

import serial
from pydamiao.enums import DamiaoMotorType, ControlType, MotorVariable
from pydamiao.motor import Motor, MotorControl

# 创建电机对象和串口对象
motor1 = Motor(DamiaoMotorType.DM4310, 0x01, 0x11)
motor2 = Motor(DamiaoMotorType.DM4310, 0x05, 0x15)
serial_device = serial.Serial("COM8", 921600, timeout=0.5)
motor_control = MotorControl(serial_device)
motor_control.add_motor(motor1)
motor_control.add_motor(motor2)

# 读取和修改电机参数示例
if motor_control.switch_control_mode(motor1, ControlType.POS_VEL):
    print("switch POS_VEL success")
if motor_control.switch_control_mode(motor2, ControlType.VEL):
    print("switch VEL success")

print("sub_ver:", motor_control.read_motor_param(motor1, MotorVariable.sub_ver))
print("Gr:", motor_control.read_motor_param(motor1, MotorVariable.Gr))
# if motor_control.change_motor_param(motor1, DM_variable.KP_APR, 54):
#     print("write success")
print("PMAX:", motor_control.read_motor_param(motor1, MotorVariable.PMAX))
print("MST_ID:", motor_control.read_motor_param(motor1, MotorVariable.MST_ID))
print("VMAX:", motor_control.read_motor_param(motor1, MotorVariable.VMAX))
print("TMAX:", motor_control.read_motor_param(motor1, MotorVariable.TMAX))
print("Motor2:")
print("PMAX:", motor_control.read_motor_param(motor2, MotorVariable.PMAX))
print("MST_ID:", motor_control.read_motor_param(motor2, MotorVariable.MST_ID))
print("VMAX:", motor_control.read_motor_param(motor2, MotorVariable.VMAX))
print("TMAX:", motor_control.read_motor_param(motor2, MotorVariable.TMAX))

# 保存并使能电机
# motor_control.enable(motor3)  # 如果 motor3 未定义，需注释或定义
motor_control.save_motor_param(motor1)
motor_control.save_motor_param(motor2)
motor_control.enable(motor1)
motor_control.enable(motor2)

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

    motor_control.control_vel(motor2, 8 * q)
    # print("Motor2:", "POS:", motor2.get_position(), "VEL:", motor2.get_velocity(), "TORQUE:", motor2.get_torque())
    # print(motor1.get_torque())
    # print(motor2.get_torque())
    time.sleep(0.001)
    # motor_control.control(motor3, 50, 0.3, q, 0, 0)

# 语句结束关闭串口
serial_device.close()
