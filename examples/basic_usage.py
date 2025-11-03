import math
import time

import serial
from pydamiao.enums import DM_Motor_Type, Control_Type, DM_variable
from pydamiao.motor import Motor, MotorControl

# 创建电机对象和串口对象
motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
motor2 = Motor(DM_Motor_Type.DM4310, 0x05, 0x15)
serial_device = serial.Serial("COM8", 921600, timeout=0.5)
motor_control = MotorControl(serial_device)
motor_control.add_motor(motor1)
motor_control.add_motor(motor2)

# 读取和修改电机参数示例
if motor_control.switch_control_mode(motor1, Control_Type.POS_VEL):
    print("switch POS_VEL success")
if motor_control.switch_control_mode(motor2, Control_Type.VEL):
    print("switch VEL success")

print("sub_ver:", motor_control.read_motor_param(motor1, DM_variable.sub_ver))
print("Gr:", motor_control.read_motor_param(motor1, DM_variable.Gr))
# if motor_control.change_motor_param(motor1, DM_variable.KP_APR, 54):
#     print("write success")
print("PMAX:", motor_control.read_motor_param(motor1, DM_variable.PMAX))
print("MST_ID:", motor_control.read_motor_param(motor1, DM_variable.MST_ID))
print("VMAX:", motor_control.read_motor_param(motor1, DM_variable.VMAX))
print("TMAX:", motor_control.read_motor_param(motor1, DM_variable.TMAX))
print("Motor2:")
print("PMAX:", motor_control.read_motor_param(motor2, DM_variable.PMAX))
print("MST_ID:", motor_control.read_motor_param(motor2, DM_variable.MST_ID))
print("VMAX:", motor_control.read_motor_param(motor2, DM_variable.VMAX))
print("TMAX:", motor_control.read_motor_param(motor2, DM_variable.TMAX))

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
    # motor_control.control_Vel(motor1, q * 5)
    motor_control.control_pos_vel(motor1, q * 8, 30)
    # print("Motor1:", "POS:", motor1.getPosition(), "VEL:", motor1.getVelocity(), "TORQUE:", motor1.getTorque())
    # motor_control.controlMIT(motor2, 35, 0.1, 8 * q, 0, 0)

    motor_control.control_vel(motor2, 8 * q)
    # print("Motor2:", "POS:", motor2.getPosition(), "VEL:", motor2.getVelocity(), "TORQUE:", motor2.getTorque())
    # print(motor1.getTorque())
    # print(motor2.getTorque())
    time.sleep(0.001)
    # motor_control.control(motor3, 50, 0.3, q, 0, 0)

# 语句结束关闭串口
serial_device.close()
