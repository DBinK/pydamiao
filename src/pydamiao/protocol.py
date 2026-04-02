# src/pydamiao/protocol.py

import numpy as np
from typing import Optional, Dict, Any, Tuple

from pydamiao.types import CanResp, MotorReg
from pydamiao.utils import (
    float_to_uint8s,
    int_to_uint8s,
    uint8s_to_float,
    uint8s_to_uint32,
    float_to_uint,
    uint_to_float,
    limit_min_max,
)
from pydamiao.result import Result


class DamiaoProtocol:
    """
    Protocol layer for Damiao motor communication.
    
    Handles:
    - Frame encoding/decoding
    - Command types (0xFC/0xFD/0xFE basic commands)
    - Register read/write (0x33/0x55)
    - Status query (0xCC)
    - Control mode offsets (0x100/0x200/0x300)
    - MIT/Position-Velocity/Velocity/Torque packing
    """
    
    # Frame structure constants
    FRAME_HEADER = 0xAA
    FRAME_TAIL = 0x55
    FRAME_LENGTH = 16
    
    # Command types
    CMD_ENABLE = 0xFC
    CMD_DISABLE = 0xFD
    CMD_SET_ZERO = 0xFE
    CMD_READ_REG = 0x33
    CMD_WRITE_REG = 0x55
    CMD_QUERY_STATUS = 0xCC
    CMD_SAVE_PARAMS = 0xAA
    
    # Control mode ID offsets
    OFFSET_POS_VEL = 0x100
    OFFSET_VEL = 0x200
    OFFSET_TORQUE_POS = 0x300
    
    # Broadcast ID
    BROADCAST_ID = 0x7FF
    
    # TX frame template (30 bytes for serial bridge)
    # The serial bridge expects a 30-byte frame with specific structure
    TX_FRAME_TEMPLATE = np.array([
        0x55, 0xAA,  # Frame header (reversed in actual transmission order)
        0x1E,        # Frame length (30)
        0x03,        # Command
        0x01, 0x00, 0x00, 0x00,  # Send count
        0x0A, 0x00, 0x00, 0x00,  # Time interval
        0x00,        # ID type: 0x00 standard, 0x01 extended
        0x00, 0x00, 0x00, 0x00,  # CAN ID (little endian)
        0x00,        # Frame type: 0x00 data, 0x01 remote
        0x08,        # Data length
        0x00,        # ID accumulator
        0x00,        # Data accumulator
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  # Data[8]
        0x00,        # CRC (not parsed by firmware, can be any value)
    ], dtype=np.uint8)
    
    # Received frame format from motor (16 bytes):
    # [0xAA][CMD][CAN_ID_L][CAN_ID_H][...][DATA(8bytes)][0x55]
    RX_FRAME_HEADER = 0xAA
    RX_FRAME_TAIL = 0x55
    RX_FRAME_LENGTH = 16
    
    @classmethod
    def build_frame(cls, motor_id: int, data: np.ndarray) -> bytes:
        """
        Build a complete 16-byte frame for transmission.
        
        Args:
            motor_id: Target motor CAN ID
            data: 8-byte data payload
            
        Returns:
            Complete 16-byte frame
        """
        frame = cls.TX_FRAME_TEMPLATE.copy()
        frame[13] = motor_id & 0xFF
        frame[14] = (motor_id >> 8) & 0xFF
        frame[21:29] = data
        return bytes(frame)
    
    @classmethod
    def encode_basic_cmd(cls, motor_id: int, cmd: int) -> bytes:
        """
        Encode basic command (enable/disable/set zero).
        
        Args:
            motor_id: Motor slave ID
            cmd: Command byte (0xFC/0xFD/0xFE)
            
        Returns:
            Complete frame bytes
        """
        data = np.array([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd], dtype=np.uint8)
        return cls.build_frame(motor_id, data)
    
    @classmethod
    def encode_mit(
        cls, 
        motor_id: int, 
        kp: float, 
        kd: float, 
        pos: float, 
        vel: float, 
        torque: float,
        limits: Any
    ) -> bytes:
        """
        Encode MIT control command.
        
        Args:
            motor_id: Motor slave ID
            kp: Position gain (0-500)
            kd: Velocity gain (0-5)
            pos: Position command
            vel: Velocity command
            torque: Torque command
            limits: MotorLimits for the motor type
            
        Returns:
            Complete frame bytes
        """
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        
        pos_uint = float_to_uint(pos, -limits.POS_MAX, limits.POS_MAX, 16)
        vel_uint = float_to_uint(vel, -limits.VEL_MAX, limits.VEL_MAX, 12)
        torque_uint = float_to_uint(torque, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12)
        
        tx_buf = np.zeros(8, dtype=np.uint8)
        tx_buf[0] = (pos_uint >> 8) & 0xFF
        tx_buf[1] = pos_uint & 0xFF
        tx_buf[2] = vel_uint >> 4
        tx_buf[3] = ((vel_uint & 0xF) << 4) | ((kp_uint >> 8) & 0xF)
        tx_buf[4] = kp_uint & 0xFF
        tx_buf[5] = kd_uint >> 4
        tx_buf[6] = ((kd_uint & 0xF) << 4) | ((torque_uint >> 8) & 0xF)
        tx_buf[7] = torque_uint & 0xFF
        
        return cls.build_frame(motor_id, tx_buf)
    
    @classmethod
    def encode_pos_vel(cls, motor_id: int, pos: float, vel: float) -> bytes:
        """
        Encode position-velocity control command.
        
        Args:
            motor_id: Motor ID (with offset 0x100)
            pos: Position command (rad)
            vel: Velocity command (rad/s)
            
        Returns:
            Complete frame bytes
        """
        target_id = cls.OFFSET_POS_VEL + motor_id
        pos_bytes = float_to_uint8s(pos)
        vel_bytes = float_to_uint8s(vel)
        
        data = np.zeros(8, dtype=np.uint8)
        data[0:4] = pos_bytes
        data[4:8] = vel_bytes
        
        return cls.build_frame(target_id, data)
    
    @classmethod
    def encode_velocity(cls, motor_id: int, vel: float) -> bytes:
        """
        Encode velocity control command.
        
        Args:
            motor_id: Motor slave ID
            vel: Velocity command (rad/s)
            
        Returns:
            Complete frame bytes
        """
        target_id = cls.OFFSET_VEL + motor_id
        vel_bytes = float_to_uint8s(vel)
        
        data = np.zeros(8, dtype=np.uint8)
        data[0:4] = vel_bytes
        
        return cls.build_frame(target_id, data)
    
    @classmethod
    def encode_torque_pos(
        cls, 
        motor_id: int, 
        pos: float, 
        vel_cmd: float, 
        cur_cmd: float
    ) -> bytes:
        """
        Encode EMIT (torque-position) control command.
        
        Args:
            motor_id: Motor slave ID
            pos: Position command (rad)
            vel_cmd: Velocity command (amplified 100x)
            cur_cmd: Current command (amplified 10000x)
            
        Returns:
            Complete frame bytes
        """
        target_id = cls.OFFSET_TORQUE_POS + motor_id
        pos_bytes = float_to_uint8s(pos)
        
        data = np.zeros(8, dtype=np.uint8)
        data[0:4] = pos_bytes
        data[4] = vel_cmd & 0xFF
        data[5] = (vel_cmd >> 8) & 0xFF
        data[6] = cur_cmd & 0xFF
        data[7] = (cur_cmd >> 8) & 0xFF
        
        return cls.build_frame(target_id, data)
    
    @classmethod
    def encode_read_reg(cls, motor_id: int, reg_id: MotorReg) -> bytes:
        """
        Encode register read command.
        
        Args:
            motor_id: Motor slave ID
            reg_id: Register ID to read
            
        Returns:
            Complete frame bytes
        """
        can_id_l = motor_id & 0xFF
        can_id_h = (motor_id >> 8) & 0xFF
        
        data = np.array([
            can_id_l, can_id_h,
            cls.CMD_READ_REG,
            reg_id,
            0x00, 0x00, 0x00, 0x00
        ], dtype=np.uint8)
        
        return cls.build_frame(cls.BROADCAST_ID, data)
    
    @classmethod
    def encode_write_reg(
        cls, 
        motor_id: int, 
        reg_id: MotorReg, 
        value: float | int
    ) -> bytes:
        """
        Encode register write command.
        
        Args:
            motor_id: Motor slave ID
            reg_id: Register ID to write
            value: Value to write
            
        Returns:
            Complete frame bytes
        """
        can_id_l = motor_id & 0xFF
        can_id_h = (motor_id >> 8) & 0xFF
        
        data = np.array([
            can_id_l, can_id_h,
            cls.CMD_WRITE_REG,
            reg_id,
            0x00, 0x00, 0x00, 0x00
        ], dtype=np.uint8)
        
        if MotorReg.is_int_type(reg_id):
            data[4:8] = int_to_uint8s(int(value))
        else:
            data[4:8] = float_to_uint8s(value)
        
        return cls.build_frame(cls.BROADCAST_ID, data)
    
    @classmethod
    def encode_query_status(cls, motor_id: int) -> bytes:
        """
        Encode status query command.
        
        Args:
            motor_id: Motor slave ID
            
        Returns:
            Complete frame bytes
        """
        can_id_l = motor_id & 0xFF
        can_id_h = (motor_id >> 8) & 0xFF
        
        data = np.array([
            can_id_l, can_id_h,
            cls.CMD_QUERY_STATUS,
            0x00, 0x00, 0x00, 0x00, 0x00
        ], dtype=np.uint8)
        
        return cls.build_frame(cls.BROADCAST_ID, data)
    
    @classmethod
    def encode_save_params(cls, motor_id: int) -> bytes:
        """
        Encode save parameters command.
        
        Args:
            motor_id: Motor slave ID
            
        Returns:
            Complete frame bytes
        """
        can_id_l = motor_id & 0xFF
        can_id_h = (motor_id >> 8) & 0xFF
        
        data = np.array([
            can_id_l, can_id_h,
            cls.CMD_SAVE_PARAMS,
            0x00, 0x00, 0x00, 0x00, 0x00
        ], dtype=np.uint8)
        
        return cls.build_frame(cls.BROADCAST_ID, data)
    
    @classmethod
    def decode_frame(cls, frame: bytes) -> Optional[Dict[str, Any]]:
        """
        Decode a received frame.
        
        Args:
            frame: 16-byte frame from motor
            
        Returns:
            Parsed message dict or None if invalid
        """
        if len(frame) != cls.RX_FRAME_LENGTH:
            return None
        
        # Received frames have header 0xAA at start, 0x55 at end
        if frame[0] != cls.RX_FRAME_HEADER or frame[-1] != cls.RX_FRAME_TAIL:
            return None
        
        # Extract CAN ID (little endian) - bytes 2-5 in RX frame
        can_id = (frame[5] << 24) | (frame[4] << 16) | (frame[3] << 8) | frame[2]
        
        # Extract data (bytes 7-14)
        data = frame[7:15]
        
        # Extract command/response type (byte 1)
        can_cmd = frame[1] if len(frame) > 1 else 0
        
        result = {
            'raw': frame,
            'can_id': can_id,
            'data': data,
            'can_cmd': can_cmd,
        }
        
        # Parse based on command type
        if can_cmd == CanResp.RECEIVE_SUCCESS:
            result['type'] = 'status'
            result['motor_id'] = can_id if can_id != 0 else (data[0] & 0x0F)
            
            if len(data) >= 6:
                pos_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                vel_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                torque_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
                
                result['pos_raw'] = pos_uint
                result['vel_raw'] = vel_uint
                result['torque_raw'] = torque_uint
        
        elif can_cmd == CanResp.RECEIVE_SUCCESS and len(data) >= 3 and data[2] in (cls.CMD_READ_REG, cls.CMD_WRITE_REG):
            result['type'] = 'reg_response'
            result['motor_id'] = can_id if can_id != 0 else ((data[1] << 8) | data[0])
            result['reg_id'] = data[3]
            
            if len(data) >= 8:
                if MotorReg.is_int_type(data[3]):
                    result['value'] = uint8s_to_uint32(data[4], data[5], data[6], data[7])
                else:
                    result['value'] = uint8s_to_float(data[4], data[5], data[6], data[7])
        
        elif can_cmd == CanResp.HEARTBEAT:
            result['type'] = 'heartbeat'
        elif can_cmd == CanResp.RECEIVE_FAILED:
            result['type'] = 'receive_failed'
        elif can_cmd == CanResp.SEND_FAILED:
            result['type'] = 'send_failed'
        elif can_cmd == CanResp.SEND_SUCCESS:
            result['type'] = 'send_success'
        elif can_cmd == CanResp.COMM_ERROR:
            result['type'] = 'comm_error'
        else:
            result['type'] = 'unknown'
        
        return result
    
    @classmethod
    def unpack_status(
        cls, 
        data: bytes, 
        limits: Any
    ) -> Tuple[float, float, float]:
        """
        Unpack status data into position, velocity, torque.
        
        Args:
            data: 8-byte status data
            limits: MotorLimits for conversion
            
        Returns:
            (position, velocity, torque) tuple
        """
        if len(data) < 6:
            return (0.0, 0.0, 0.0)
        
        pos_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
        vel_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
        torque_uint = np.uint16(((data[4] & 0xF) << 8) | data[5])
        
        pos = uint_to_float(pos_uint, -limits.POS_MAX, limits.POS_MAX, 16)
        vel = uint_to_float(vel_uint, -limits.VEL_MAX, limits.VEL_MAX, 12)
        torque = uint_to_float(torque_uint, -limits.TORQUE_MAX, limits.TORQUE_MAX, 12)
        
        return (float(pos), float(vel), float(torque))
