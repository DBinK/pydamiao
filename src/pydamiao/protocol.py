# src/pydamiao/protocol.py
"""
协议层 - 负责帧编码/解码
处理帧头、ID、命令、CRC 校验
"""

import struct
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass


@dataclass
class Frame:
    """数据帧结构"""
    header: int       # 帧头
    cmd: int          # 命令
    length: int       # 数据长度
    motor_id: int     # 电机 ID (CAN ID)
    data: bytes       # 数据载荷
    crc: int          # CRC 校验
    
    @classmethod
    def from_bytes(cls, data: bytes) -> Optional['Frame']:
        """从字节数据解析帧"""
        if len(data) < 16:
            return None
            
        try:
            # 检查帧头 (0xAA 0x55)
            if data[0] != 0xAA or data[15] != 0x55:
                return None
                
            header = (data[0] << 8) | data[1]
            cmd = data[3]
            length = data[2]
            
            # CAN ID is at [13:17] in little endian
            can_id = (data[16] << 24) | (data[15] << 16) | (data[14] << 8) | data[13]
            
            # Data is at [20:28]
            frame_data = data[20:28] if len(data) >= 28 else b'\x00' * 8
            
            crc = data[29] if len(data) > 29 else 0
            
            return cls(
                header=header,
                cmd=cmd,
                length=length,
                motor_id=can_id,
                data=frame_data,
                crc=crc
            )
        except Exception:
            return None


class MotorProtocol:
    """
    电机协议编解码类
    
    帧格式 (30 字节):
    [0x55, 0xAA]  - 帧头
    [Length]      - 帧长 (固定 0x1e = 30)
    [Cmd]         - 命令字
    [4 bytes]     - 发送次数
    [4 bytes]     - 时间间隔
    [ID Type]     - ID 类型 (0x00 标准帧)
    [4 bytes]     - CAN ID (小端序)
    [Frame Type]  - 帧类型 (0x00 数据帧)
    [Data Len]    - 数据长度 (固定 0x08)
    [ID Acc]      - ID 累加器
    [Data Acc]    - 数据累加器
    [8 bytes]     - 数据载荷
    [CRC]         - CRC 校验
    [0x55]        - 帧尾
    """
    
    HEADER = bytes([0x55, 0xAA])
    FRAME_LENGTH = 30  # 完整帧长度
    
    # 命令定义
    CMD_MIT_CONTROL = 0x03
    CMD_BASIC = 0xFC  # 基础命令 (使能/失能等通过 data 区分)
    CMD_READ_PARAM = 0x33
    CMD_WRITE_PARAM = 0x55
    CMD_SAVE_PARAM = 0xAA
    
    # 基础命令子命令
    BASIC_ENABLE = 0xFC
    BASIC_DISABLE = 0xFD
    BASIC_SET_ZERO = 0xFE
    
    @classmethod
    def encode_frame(
        cls,
        motor_id: int,
        cmd: int,
        data: bytes = b"",
        send_count: int = 1,
        time_interval: int = 10,
    ) -> bytes:
        """
        编码发送帧
        
        Args:
            motor_id: 电机 CAN ID
            cmd: 命令字
            data: 数据载荷 (最多 8 字节)
            send_count: 发送次数
            time_interval: 时间间隔
            
        Returns:
            编码后的帧 (30 字节)
        """
        if len(data) > 8:
            raise ValueError("Data length must be <= 8 bytes")
            
        frame = bytearray(30)
        
        # 帧头
        frame[0] = 0x55
        frame[1] = 0xAA
        
        # 帧长
        frame[2] = 0x1E
        
        # 命令
        frame[3] = cmd
        
        # 发送次数 (4 bytes, little endian)
        frame[4:8] = send_count.to_bytes(4, 'little')
        
        # 时间间隔 (4 bytes, little endian)
        frame[8:12] = time_interval.to_bytes(4, 'little')
        
        # ID 类型 (0x00 标准帧)
        frame[12] = 0x00
        
        # CAN ID (4 bytes, little endian)
        frame[13:17] = motor_id.to_bytes(4, 'little')
        
        # 帧类型 (0x00 数据帧)
        frame[17] = 0x00
        
        # 数据长度
        frame[18] = 0x08
        
        # ID 累加器
        frame[19] = 0x00
        
        # 数据累加器
        frame[20] = 0x00
        
        # 数据载荷 (填充到 8 字节)
        data_padded = data.ljust(8, b'\x00')[:8]
        frame[21:29] = data_padded
        
        # CRC 校验 (简单求和)
        frame[29] = sum(frame[:29]) & 0xFF
        
        return bytes(frame)
        
    @classmethod
    def encode_basic_cmd(cls, motor_id: int, sub_cmd: int) -> bytes:
        """
        编码基础命令 (使能/失能/清零)
        
        Args:
            motor_id: 电机 ID
            sub_cmd: 子命令 (0xFC 使能，0xFD 失能，0xFE 清零)
            
        Returns:
            编码后的帧
        """
        data = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, sub_cmd])
        return cls.encode_frame(motor_id, cls.CMD_BASIC, data)
        
    @classmethod
    def encode_mit_control(
        cls,
        motor_id: int,
        pos: int,
        vel: int,
        kp: int,
        kd: int,
        torque: int,
    ) -> bytes:
        """
        编码 MIT 控制命令
        
        MIT 数据打包格式 (8 字节):
        [Pos(16bit)][Vel(12bit)][Kp(12bit)][Kd(12bit)][Torque(12bit)]
        
        Args:
            motor_id: 电机 ID
            pos: 位置 (16bit 无符号)
            vel: 速度 (12bit 无符号)
            kp: Kp 增益 (12bit 无符号)
            kd: Kd 增益 (12bit 无符号)
            torque: 扭矩 (12bit 无符号)
            
        Returns:
            编码后的帧
        """
        data = bytearray(8)
        
        # 位置 (16bit)
        data[0] = (pos >> 8) & 0xFF
        data[1] = pos & 0xFF
        
        # 速度 (12bit) + Kp 高 4 位
        data[2] = (vel >> 4) & 0xFF
        data[3] = ((vel & 0xF) << 4) | ((kp >> 8) & 0xF)
        data[4] = kp & 0xFF
        
        # Kd (12bit) + Torque 高 4 位
        data[5] = (kd >> 4) & 0xFF
        data[6] = ((kd & 0xF) << 4) | ((torque >> 8) & 0xF)
        data[7] = torque & 0xFF
        
        return cls.encode_frame(motor_id, cls.CMD_MIT_CONTROL, bytes(data))
        
    @classmethod
    def encode_pos_vel(cls, motor_id: int, pos: float, vel: float) -> bytes:
        """
        编码位置速度控制命令
        
        Args:
            motor_id: 电机 ID (实际为 0x100 + slave_id)
            pos: 期望位置 (float, 4 字节)
            vel: 期望速度 (float, 4 字节)
            
        Returns:
            编码后的帧
        """
        pos_bytes = struct.pack('f', pos)
        vel_bytes = struct.pack('f', vel)
        data = pos_bytes + vel_bytes
        return cls.encode_frame(motor_id, cls.CMD_MIT_CONTROL, data)
        
    @classmethod
    def encode_velocity(cls, motor_id: int, vel: float) -> bytes:
        """
        编码速度控制命令
        
        Args:
            motor_id: 电机 ID (实际为 0x200 + slave_id)
            vel: 期望速度 (float, 4 字节)
            
        Returns:
            编码后的帧
        """
        vel_bytes = struct.pack('f', vel)
        data = vel_bytes + b'\x00' * 4
        return cls.encode_frame(motor_id, cls.CMD_MIT_CONTROL, data)
        
    @classmethod
    def decode_response(cls, data: bytes) -> Optional[Dict[str, Any]]:
        """
        解码响应帧
        
        Args:
            data: 接收到的原始数据 (至少 16 字节)
            
        Returns:
            解析后的字典，包含:
            - motor_id: 电机 ID
            - cmd: 命令
            - data: 数据载荷
            - 或者 None 如果解析失败
        """
        if len(data) < 16:
            return None
            
        # 检查帧头
        if data[0] != 0xAA or data[15] != 0x55:
            return None
        
        try:
            cmd = data[3]
            # CAN ID is at [13:17] in little endian
            can_id = (data[16] << 24) | (data[15] << 16) | (data[14] << 8) | data[13]
            # Data is at [20:28]
            frame_data = data[20:28] if len(data) >= 28 else b'\x00' * 8
            
            return {
                "motor_id": can_id,
                "cmd": cmd,
                "data": frame_data,
            }
        except Exception:
            return None
        
    @classmethod
    def extract_frames(cls, buffer: bytes) -> Tuple[List[bytes], bytes]:
        """
        从缓冲区提取完整帧 (处理粘包)
        
        Args:
            buffer: 接收缓冲区数据
            
        Returns:
            (frames, remainder): 完整帧列表和剩余数据
        """
        frames = []
        header_start = 0xAA
        header_end = 0x55
        tail = 0x55
        frame_length = 16
        
        i = 0
        remainder_pos = 0
        
        while i <= len(buffer) - frame_length:
            # 检查帧头 (0xAA 0x55) 和帧尾 (0x55)
            if buffer[i] == header_start and buffer[i+1] == header_end and buffer[i + frame_length - 1] == tail:
                frame = buffer[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
                
        remainder = buffer[remainder_pos:]
        return frames, remainder
