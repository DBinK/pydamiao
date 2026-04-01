# src/pydamiao/manager.py
"""
电机管理层 - 多电机管理器
管理多个电机对象，提供批量控制和同步读取功能
"""

from typing import Dict, List, Optional, Callable
import struct

from pydamiao.types import Hex, ControlMode, MotorType, MotorReg, CanResp
from pydamiao.bus import SerialBus
from pydamiao.protocol import MotorProtocol
from pydamiao.motor import Motor


class MotorManager:
    """
    多电机管理器
    
    功能：
    - 管理多个电机对象
    - 批量控制（使能所有、失能所有）
    - 接收回调处理（自动更新电机状态）
    - 提供统一的 API 接口
    """
    
    def __init__(self, bus: SerialBus):
        """
        初始化电机管理器
        
        Args:
            bus: 串口总线对象
        """
        self.bus = bus
        self.motors: Dict[Hex, Motor] = {}
        self._rx_buffer = bytearray()
        
        # 设置总线数据回调
        self.bus.set_data_callback(self._on_data_received)
    
    def add_motor(self, motor: Motor) -> bool:
        """
        添加电机到管理器
        
        Args:
            motor: 电机对象
            
        Returns:
            True 表示成功
        """
        self.motors[motor.slave_id] = motor
        if motor.master_id != 0:
            self.motors[motor.master_id] = motor
        return True
    
    def remove_motor(self, motor_id: Hex) -> bool:
        """
        从管理器移除电机
        
        Args:
            motor_id: 电机 ID
            
        Returns:
            True 表示成功，False 表示电机不存在
        """
        if motor_id in self.motors:
            del self.motors[motor_id]
            return True
        return False
    
    def get_motor(self, motor_id: Hex) -> Optional[Motor]:
        """
        获取电机对象
        
        Args:
            motor_id: 电机 ID
            
        Returns:
            电机对象，或 None 如果不存在
        """
        return self.motors.get(motor_id)
    
    def __getitem__(self, motor_id: Hex) -> Motor:
        """通过索引访问电机"""
        return self.motors[motor_id]
    
    def __len__(self) -> int:
        """获取管理的电机数量"""
        return len(self.motors)
    
    def __iter__(self):
        """迭代所有电机"""
        return iter(self.motors.values())
    
    # ==================== 批量控制 ====================
    
    def enable_all(self):
        """使能所有电机"""
        for motor in self.motors.values():
            motor.enable()
    
    def disable_all(self):
        """失能所有电机"""
        for motor in self.motors.values():
            motor.disable()
    
    def set_zero_all(self):
        """所有电机清零位置"""
        for motor in self.motors.values():
            motor.set_zero_position()
    
    def refresh_all_status(self):
        """刷新所有电机状态"""
        for motor in self.motors.values():
            motor.refresh_motor_status()
    
    # ==================== 数据接收处理 ====================
    
    def _on_data_received(self, data: bytes):
        """
        数据接收回调（由 SerialBus 调用）
        
        Args:
            data: 接收到的原始数据
        """
        # 添加到缓冲区
        self._rx_buffer.extend(data)
        
        # 提取完整帧
        frames, remainder = MotorProtocol.extract_frames(bytes(self._rx_buffer))
        self._rx_buffer = bytearray(remainder)
        
        # 处理每一帧
        for frame in frames:
            self._process_frame(frame)
    
    def _process_frame(self, frame: bytes):
        """
        处理单帧数据
        
        Args:
            frame: 完整的帧数据 (16 字节)
        """
        result = MotorProtocol.decode_response(frame)
        if result is None:
            return
        
        motor_id = result["motor_id"]
        cmd = result["cmd"]
        data = result["data"]
        
        # 查找目标电机
        target_motor = None
        if motor_id in self.motors:
            target_motor = self.motors[motor_id]
        else:
            # 尝试从数据中提取 slave_id
            if len(data) >= 2:
                slave_id = (data[1] << 8) | data[0]
                if slave_id in self.motors:
                    target_motor = self.motors[slave_id]
        
        if target_motor is None:
            return
        
        # 根据命令类型处理
        if cmd == 0x03:  # 数据响应
            self._process_state_response(target_motor, data)
        elif cmd in [0x33, 0x55]:  # 参数响应
            self._process_param_response(target_motor, data)
    
    def _process_state_response(self, motor: Motor, data: bytes):
        """
        处理状态响应数据
        
        Args:
            motor: 目标电机
            data: 数据载荷
        """
        if len(data) < 6:
            return
            
        # 解析 MIT 反馈数据
        pos_uint = (data[1] << 8) | data[2]
        vel_uint = (data[3] << 4) | (data[4] >> 4)
        torque_uint = ((data[4] & 0xF) << 8) | data[5]
        
        from pydamiao.utils import uint_to_float
        from pydamiao.types import MOTOR_LIMITS
        
        limits = MOTOR_LIMITS[motor.motor_type]
        
        recv_pos = uint_to_float(pos_uint, -limits.POS_MAX, limits.POS_MAX, 16)
        recv_vel = uint_to_float(vel_uint, -limits.VEL_MAX, limits.VEL_MAX, 12)
        recv_torque = uint_to_float(torque_uint, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12)
        
        motor.update_from_callback(float(recv_pos), float(recv_vel), float(recv_torque))
    
    def _process_param_response(self, motor: Motor, data: bytes):
        """
        处理参数响应数据
        
        Args:
            motor: 目标电机
            data: 数据载荷
        """
        if len(data) < 8 or data[2] not in [0x33, 0x55]:
            return
        
        reg_id = data[3]
        
        # 判断数据类型
        from pydamiao.types import MotorReg
        from pydamiao.utils import uint8s_to_uint32, uint8s_to_float
        
        if MotorReg.is_int_type(reg_id):
            value = uint8s_to_uint32(data[4], data[5], data[6], data[7])
        else:
            value = uint8s_to_float(data[4], data[5], data[6], data[7])
        
        motor.update_param_cache(MotorReg(reg_id), value)
    
    # ==================== 工具方法 ====================
    
    def get_all_positions(self) -> Dict[Hex, float]:
        """获取所有电机位置"""
        return {m.slave_id: m.pos for m in self.motors.values()}
    
    def get_all_velocities(self) -> Dict[Hex, float]:
        """获取所有电机速度"""
        return {m.slave_id: m.vel for m in self.motors.values()}
    
    def get_all_torques(self) -> Dict[Hex, float]:
        """获取所有电机扭矩"""
        return {m.slave_id: m.torque for m in self.motors.values()}
    
    def clear_errors(self):
        """清除所有电机错误状态"""
        for motor in self.motors.values():
            # 可以通过重新使能来清除错误
            if not motor.enabled:
                motor.enable()
