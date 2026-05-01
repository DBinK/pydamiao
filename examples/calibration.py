import time
from pydamiao import MotorManager, SerialBus, MotorType, MotorReg, ControlMode

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

# 需要固定的电机列表
need_to_pin = [
    # manager.get_by_name("wrist_3"),
    manager.get_by_name("wrist_2"),
    manager.get_by_name("wrist_1"),
    manager.get_by_name("elbow"),
    manager.get_by_name("shoulder"),
    manager.get_by_name("base"),
]

# 需要设置零点的电机列表
need_set_zero = [
    manager.get_by_name("wrist_3"),
    # manager.get_by_name("wrist_2"),
    # manager.get_by_name("wrist_1"),
    # manager.get_by_name("elbow"),
    # manager.get_by_name("shoulder"),
    # manager.get_by_name("base")
]

# 固定电机位置并读取当前位置
for motor in need_to_pin:
    if motor is None:
        print("未找到电机")
        break
    motor.enable()
    motor.set_mode(ControlMode.POS_VEL)
    motor.set_pos_vel(0, 10)
    p_m = motor.read_param(MotorReg.p_m).value
    print(f"{motor.name} 电机当前位置 {p_m=}")
        
    time.sleep(0.1) 

# 设置待校准电机为 MIT 零力矩模式, 以便于我们手动调整电机位置到零点位置
for motor in need_set_zero:
    if motor is None:
        print("未找到电机")
        break
    motor.enable()
    motor.set_mode(ControlMode.MIT)
    motor.set_mit(0, 0,0,0,0)
    
    p_m = motor.read_param(MotorReg.p_m).value
    print(f"{motor.name} 电机当前位置 {p_m=}")

    time.sleep(0.1) 

# 暂停程序以便观察结果
input("按 Enter 键开始设置电机零点...")

# 执行读取
for motor in need_set_zero:
    if motor is None:
        print("未找到电机")
        break
    try:
        p_m = motor.read_param(MotorReg.p_m).value
        print(f"{motor.name} 电机当前位置 {p_m=}")

        motor.set_zero()

        p_m = motor.read_param(MotorReg.p_m).value
        print(f"{motor.name} 电机当前位置 (应为极小值) {p_m=}")

        time.sleep(0.1) 
        
        input("按 Enter 键继续设置下一个电机...")

    except Exception as e:
        print(f"与电机 {motor.name} 通讯失败: {e}")

# 测试结束失能电机
manager.disable_all()  
bus.close()