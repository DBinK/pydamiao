import time
from pydamiao import MotorManager, SerialBus, MotorType, MotorReg, ControlMode
from pydamiao.motor import Motor

# 初始化总线和管理器
bus = SerialBus("COM9", baudrate=921600, timeout=0.01)
manager = MotorManager(bus)

# 添加电机
motors = [
    manager.add_motor(MotorType.DM4310, 0x06, 0x16, name="wrist_3"),
    manager.add_motor(MotorType.DM4310, 0x05, 0x15, name="wrist_2"),
    manager.add_motor(MotorType.DM4310, 0x04, 0x14, name="wrist_1"),
    manager.add_motor(MotorType.DM4340, 0x03, 0x13, name="elbow"),
    manager.add_motor(MotorType.DM4340, 0x02, 0x12, name="shoulder"),
    manager.add_motor(MotorType.DM4340, 0x01, 0x11, name="base"),
]

pin_motors = [
    # manager.get_by_name("wrist_3"),
    manager.get_by_name("wrist_2"),
    manager.get_by_name("wrist_1"),
    manager.get_by_name("elbow"),
    manager.get_by_name("shoulder"),
    manager.get_by_name("base"),
]

    

need_set_zero = [
    manager.get_by_name("wrist_3"),
    # manager.get_by_name("wrist_2"),
    # manager.get_by_name("wrist_1"),
    # manager.get_by_name("elbow"),
    # manager.get_by_name("shoulder"),
    # manager.get_by_name("base")
]

for motor in pin_motors:
    if motor is None:
        print("未找到电机")
        break
    motor.enable()
    motor.set_mode(ControlMode.POS_VEL)
    motor.set_pos_vel(0, 10)
    p_m = motor.read_param(MotorReg.p_m).value
    print(f"电机当前位置 {p_m=}")
        
    time.sleep(0.1) 

for motor in need_set_zero:
    if motor is None:
        print("未找到电机")
        break
    motor.enable()
    motor.set_mode(ControlMode.MIT)
    motor.set_mit(0, 0,0,0,0)
    
    p_m = motor.read_param(MotorReg.p_m).value
    print(f"电机当前位置 {p_m=}")

    
    time.sleep(0.1) 


breakpoint()

# 执行读取
for motor in need_set_zero:
    if motor is None:
        print("未找到电机")
        break
    try:
        p_m = motor.read_param(MotorReg.p_m).value
        print(f"电机当前位置 {p_m=}")

        motor.set_zero()

        p_m = motor.read_param(MotorReg.p_m).value
        print(f"电机当前位置(应为极小值) {p_m=}")

        time.sleep(0.1) 
        
        breakpoint()

    except Exception as e:
        print(f"与电机 {motor.name} 通讯失败: {e}")



bus.close()