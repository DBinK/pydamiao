"""
重构后的使用示例

展示新架构的使用方式：
- SerialBus: 串口总线层
- MotorProtocol: 协议层
- Motor: 单电机抽象
- MotorManager: 多电机管理
"""

import math
import time
import serial

from pydamiao import SerialBus, Motor, MotorManager, MotorType, ControlMode, MotorReg


def main():
    # ==================== 1. 初始化串口总线 ====================
    # 注意：请根据实际情况修改串口号
    port = "COM3"  # Linux: "/dev/ttyUSB0"
    baudrate = 921600
    
    bus = SerialBus(port, baudrate)
    bus.open()
    
    # ==================== 2. 创建电机管理器 ====================
    manager = MotorManager(bus)
    
    # ==================== 3. 添加电机 ====================
    motor1 = Motor(bus, MotorType.DM4310, slave_id=0x06, master_id=0x12)
    motor2 = Motor(bus, MotorType.DM4310, slave_id=0x05, master_id=0x12)
    
    manager.add_motor(motor1)
    manager.add_motor(motor2)
    
    print(f"已添加 {len(manager)} 个电机")
    
    # ==================== 4. 读取和修改参数 ====================
    print("\n--- 读取电机参数 ---")
    
    # 切换控制模式
    if motor1.switch_control_mode(ControlMode.POS_VEL):
        print("Motor1: 切换到 POS_VEL 模式成功")
    else:
        print("Motor1: 切换模式失败")
        
    if motor2.switch_control_mode(ControlMode.VEL):
        print("Motor2: 切换到 VEL 模式成功")
    else:
        print("Motor2: 切换模式失败")
    
    # 读取参数
    sub_ver = motor1.read_motor_param(MotorReg.sub_ver)
    print(f"Motor1 子版本号：{sub_ver}")
    
    gr = motor1.read_motor_param(MotorReg.Gr)
    print(f"Motor1 齿轮比：{gr}")
    
    pmax = motor1.read_motor_param(MotorReg.PMAX)
    print(f"Motor1 PMAX: {pmax}")
    
    # ==================== 5. 保存参数并使能 ====================
    print("\n--- 保存参数并使能电机 ---")
    motor1.save_motor_param()
    motor2.save_motor_param()
    
    motor1.enable()
    motor2.enable()
    print("电机已使能")
    
    # ==================== 6. 控制循环 ====================
    print("\n--- 开始控制循环 ---")
    
    try:
        start_time = time.time()
        for i in range(1000):
            t = time.time() - start_time
            q = math.sin(t)
            
            # 控制电机 1 (位置速度模式)
            motor1.control_pos_vel(q * 8, 3)
            
            # 控制电机 2 (速度模式)
            motor2.control_velocity(q * 8)
            
            # 打印状态（从回调自动更新）
            if i % 100 == 0:
                print(f"[{i}] Motor1: pos={motor1.pos:.3f}, vel={motor1.vel:.3f}, torque={motor1.torque:.3f}")
                print(f"[{i}] Motor2: pos={motor2.pos:.3f}, vel={motor2.vel:.3f}, torque={motor2.torque:.3f}")
            
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n用户中断")
    
    # ==================== 7. 清理 ====================
    print("\n--- 停止电机并关闭 ---")
    motor2.control_velocity(0.0)
    motor1.disable()
    motor2.disable()
    
    bus.close()
    print("串口已关闭")


if __name__ == "__main__":
    main()
