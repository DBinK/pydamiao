# src/pydamiao/motor.py
"""
电机抽象层 - 单电机驱动类
提供高级 API（enable、set_pos 等）
"""

from time import sleep
from typing import Optional, Dict, Any
import struct

import numpy as np

from pydamiao.types import Hex, ControlMode, MotorType, MotorReg, MOTOR_LIMITS, CanResp
from pydamiao.utils import (
    int_to_uint8s,
    float_to_uint,
    float_to_uint8s,
    uint8s_to_float,
    uint8s_to_uint32,
    uint_to_float,
)
from pydamiao.bus import SerialBus
from pydamiao.protocol import MotorProtocol


class Motor:
    """
    单电机抽象类
    
    每个电机对象对应一个 CAN ID，提供高级控制接口：
    - enable/disable: 使能/失能
    - set_zero: 清零位置
    - control_mit: MIT 控制模式
    - control_pos_vel: 位置速度控制
    - control_velocity: 速度控制
    - read_param/write_param: 参数读写
    """

    def __init__(self, bus: SerialBus, motor_type: MotorType, slave_id: Hex, master_id: Hex):
        """
        初始化电机对象
        
        Args:
            bus: 串口总线对象 (共享)
            motor_type: 电机类型
            slave_id: 从机 ID (CAN ID)
            master_id: 主机 ID，建议不要设为 0
        """
        self.bus = bus
        self.motor_type = motor_type
        self.slave_id = slave_id
        self.master_id = master_id

        # 电机状态反馈
        self.pos: float = 0.0
        self.vel: float = 0.0
        self.torque: float = 0.0

        self.enabled: bool = False  
        self.control_mode: Optional[ControlMode] = None  
        self.param_cache: Dict[MotorReg, Any] = {}

    # ==================== 基础控制命令 ====================

    def enable(self):
        """使能电机"""
        frame = MotorProtocol.encode_basic_cmd(self.slave_id, MotorProtocol.BASIC_ENABLE)
        self.bus.send(frame)
        sleep(0.1)
        self.enabled = True

    def disable(self):
        """失能电机"""
        frame = MotorProtocol.encode_basic_cmd(self.slave_id, MotorProtocol.BASIC_DISABLE)
        self.bus.send(frame)
        sleep(0.1)
        self.enabled = False
    
    def set_zero_position(self):
        """设置电机零位"""
        frame = MotorProtocol.encode_basic_cmd(self.slave_id, MotorProtocol.BASIC_SET_ZERO)
        self.bus.send(frame)
        sleep(0.1)

    # ==================== 控制模式 ====================

    def control_mit(self, kp: float, kd: float, pos_cmd: float, vel_cmd: float, torque_cmd: float):
        """
        MIT 控制模式
        
        Args:
            kp: 比例系数 (0-500)
            kd: 微分系数 (0-5)
            pos_cmd: 期望位置
            vel_cmd: 期望速度
            torque_cmd: 期望力矩
        """
        limits = MOTOR_LIMITS[self.motor_type]
        
        # 量化参数
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        pos_uint = float_to_uint(pos_cmd, -limits.POS_MAX, limits.POS_MAX, 16)
        vel_uint = float_to_uint(vel_cmd, -limits.VEL_MAX, limits.VEL_MAX, 12)
        torque_uint = float_to_uint(torque_cmd, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12)
        
        frame = MotorProtocol.encode_mit_control(
            self.slave_id, pos_uint, vel_uint, kp_uint, kd_uint, torque_uint
        )
        self.bus.send(frame)
    
    def control_mit_delay(
        self, kp: float, kd: float, pos_cmd: float, vel_cmd: float, torque_cmd: float, delay: float
    ):
        """MIT 控制模式（带延迟）"""
        self.control_mit(kp, kd, pos_cmd, vel_cmd, torque_cmd)
        sleep(delay)
    
    def control_pos_vel(self, pos_cmd: float, vel_cmd: float):
        """
        位置速度控制模式
        
        Args:
            pos_cmd: 期望位置
            vel_cmd: 期望速度
        """
        motor_id = 0x100 + self.slave_id
        frame = MotorProtocol.encode_pos_vel(motor_id, pos_cmd, vel_cmd)
        self.bus.send(frame)
    
    def control_velocity(self, vel_cmd: float):
        """
        速度控制模式
        
        Args:
            vel_cmd: 期望速度
        """
        motor_id = 0x200 + self.slave_id
        frame = MotorProtocol.encode_velocity(motor_id, vel_cmd)
        self.bus.send(frame)
    
    def control_pos_force(self, pos_cmd: float, vel_cmd: float, cur_cmd: float):
        """
        EMIT 控制模式（力位混合模式）
        
        Args:
            pos_cmd: 期望位置 (rad)
            vel_cmd: 期望速度 (放大 100 倍)
            cur_cmd: 期望电流标幺值 (放大 10000 倍)
        """
        motor_id = 0x300 + self.slave_id
        tx_buf = np.array([0x00] * 8, dtype=np.uint8)
        
        pos_bytes = struct.pack('f', pos_cmd)
        tx_buf[0:4] = pos_bytes
        
        vel_cmd_uint = np.uint16(vel_cmd)
        cur_cmd_uint = np.uint16(cur_cmd)
        tx_buf[4] = vel_cmd_uint & 0xFF
        tx_buf[5] = (vel_cmd_uint >> 8) & 0xFF
        tx_buf[6] = cur_cmd_uint & 0xFF
        tx_buf[7] = (cur_cmd_uint >> 8) & 0xFF
        
        frame = MotorProtocol.encode_frame(motor_id, MotorProtocol.CMD_MIT_CONTROL, tx_buf.tobytes())
        self.bus.send(frame)

    # ==================== 参数管理 ====================

    def refresh_motor_status(self):
        """获取电机状态"""
        data = bytearray(8)
        data[0] = self.slave_id & 0xFF
        data[1] = (self.slave_id >> 8) & 0xFF
        data[2] = 0xCC
        data[3] = 0x00
        frame = MotorProtocol.encode_frame(0x7FF, MotorProtocol.CMD_READ_PARAM, bytes(data))
        self.bus.send(frame)

    def read_motor_param(self, reg_id: MotorReg, max_retries: int = 20, retry_interval: float = 0.05) -> Optional[Any]:
        """
        读取电机内部参数
        
        Args:
            reg_id: 寄存器 ID
            max_retries: 最大重试次数
            retry_interval: 重试间隔
            
        Returns:
            参数值，或 None 如果读取失败
        """
        data = bytearray(8)
        data[0] = self.slave_id & 0xFF
        data[1] = (self.slave_id >> 8) & 0xFF
        data[2] = 0x33
        data[3] = reg_id
        frame = MotorProtocol.encode_frame(0x7FF, MotorProtocol.CMD_WRITE_PARAM, bytes(data))
        self.bus.send(frame)
        
        for _ in range(max_retries):
            sleep(retry_interval)
            if reg_id in self.param_cache:
                return self.param_cache[reg_id]
        return None

    def write_motor_param(
        self, reg_id: MotorReg, value: Any, max_retries: int = 20, retry_interval: float = 0.05
    ) -> bool:
        """
        写入电机参数（仅当前生效，需调用 save_motor_param 保存）
        
        Args:
            reg_id: 寄存器 ID
            value: 参数值
            max_retries: 最大重试次数
            retry_interval: 重试间隔
            
        Returns:
            True 表示成功，False 表示失败
        """
        data = bytearray(8)
        data[0] = self.slave_id & 0xFF
        data[1] = (self.slave_id >> 8) & 0xFF
        data[2] = 0x33 if MotorReg.is_int_type(reg_id) else 0x55
        data[3] = reg_id
        
        if MotorReg.is_int_type(reg_id):
            data[4:8] = int(value).to_bytes(4, 'little')
        else:
            data[4:8] = struct.pack('f', float(value))
            
        frame = MotorProtocol.encode_frame(0x7FF, MotorProtocol.CMD_WRITE_PARAM, bytes(data))
        self.bus.send(frame)
        
        for _ in range(max_retries):
            sleep(retry_interval)
            if reg_id in self.param_cache:
                cached_value = self.param_cache[reg_id]
                if isinstance(cached_value, float) and isinstance(value, (int, float)):
                    return abs(cached_value - float(value)) < 0.1
                elif cached_value == int(value):
                    return True
                return False
        return False

    def save_motor_param(self):
        """保存所有参数到 flash"""
        data = bytearray(8)
        data[0] = self.slave_id & 0xFF
        data[1] = (self.slave_id >> 8) & 0xFF
        data[2] = 0xAA
        data[3] = 0x00
        self.disable()
        frame = MotorProtocol.encode_frame(0x7FF, MotorProtocol.CMD_SAVE_PARAM, bytes(data))
        self.bus.send(frame)
        sleep(0.001)
    
    def switch_control_mode(self, mode: ControlMode, max_retries: int = 20, retry_interval: float = 0.1) -> bool:
        """
        切换电机控制模式
        
        Args:
            mode: 目标控制模式
            max_retries: 最大重试次数
            retry_interval: 重试间隔
            
        Returns:
            True 表示成功，False 表示失败
        """
        success = self.write_motor_param(MotorReg.CTRL_MODE, mode, max_retries, retry_interval)
        if success:
            self.control_mode = mode
        return success

    # ==================== 状态更新 ====================

    def update_from_callback(self, pos: float, vel: float, torque: float):
        """
        从回调被动更新数据
        
        Args:
            pos: 位置
            vel: 速度
            torque: 扭矩
        """
        self.pos = pos
        self.vel = vel
        self.torque = torque
        
    def update_param_cache(self, reg_id: MotorReg, value: Any):
        """更新参数缓存"""
        self.param_cache[reg_id] = value
