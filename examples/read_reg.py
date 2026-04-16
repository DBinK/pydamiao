import time
from pydamiao import MotorManager, SerialBus, MotorType, MotorReg
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

def read_motor_configs(motor: Motor):
    print(f"\n--- 正在读取电机: {motor.name} (ID: {hex(motor.slave_id)}) (motor_type: {motor.motor_type.name}) ---")
    
    # 1. 读取版本号 (针对您读出的巨大数字进行检查)
    # 正常版本号如 V13, V15, V17 应该是较小的整数
    sw = motor.read_param(MotorReg.sw_ver).value
    sub = motor.read_param(MotorReg.sub_ver).value
    print(f"原始软件版本值: {sw} | 子版本: {sub}")

    # 2. 关键映射参数排查
    try:
        pmax = motor.read_param(MotorReg.PMAX).value
        vmax = motor.read_param(MotorReg.VMAX).value
        tmax = motor.read_param(MotorReg.TMAX).value
        gr = motor.read_param(MotorReg.Gr).value
        sw_ver = motor.read_param(MotorReg.sw_ver).value
        SN = motor.read_param(MotorReg.SN).value
        
        print(f"位置映射范围 (PMAX): {pmax}")
        print(f"速度映射范围 (VMAX): {vmax}")
        print(f"扭矩映射范围 (TMAX): {tmax}")
        print(f"齿轮减速比 (Gr): {gr}")
        print(f"软件版本: {sw_ver}")
        print(f"电机编号: {SN}")
        
        # 3. 读取控制模式
        mode = motor.read_param(MotorReg.CTRL_MODE).value
        print(f"当前控制模式编码: {mode}")
        
    except Exception as e:
        print(f"读取寄存器数值时出错: {e}")

# 执行读取
for m in motors:
    try:
        read_motor_configs(m)
        time.sleep(0.1) 
    except Exception as e:
        print(f"与电机 {m.name} 通讯失败: {e}")



bus.close()