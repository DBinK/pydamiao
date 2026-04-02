# src/pydamiao/driver.py

import time
from typing import Optional, Dict, Any

import numpy as np

from pydamiao.bus import SerialBus
from pydamiao.protocol import DamiaoProtocol
from pydamiao.types import MotorType, ControlMode, MotorReg, MOTOR_LIMITS, CanResp
from pydamiao.result import Result, check_result


class Motor:
    """
    Single motor abstraction with high-level API.
    
    Features:
    - Automatic mode switching for control commands
    - State caching (pos, vel, torque, enabled)
    - Async feedback via background callback
    - Sync parameter read/write
    
    Usage:
        motor = Motor(bus, MotorType.DM4310, slave_id=0x06)
        
        # Control commands auto-switch mode
        motor.enable()
        motor.set_pos_vel(1.0, 0.5)  # Auto switches to POS_VEL mode
        
        # Read cached state (updated asynchronously)
        print(f"Position: {motor.position}")
        
        # Sync parameter access
        pmax = motor.read_param(MotorReg.PMAX)
        
        motor.disable()
    """
    
    def __init__(
        self, 
        bus: SerialBus, 
        motor_type: MotorType, 
        slave_id: int, 
        master_id: int = 0
    ):
        """
        Initialize a motor object.
        
        Args:
            bus: Shared SerialBus instance
            motor_type: Motor type enum
            slave_id: Motor CAN ID (slave)
            master_id: Master ID (default 0)
        """
        self.bus = bus
        self.motor_type = motor_type
        self.slave_id = slave_id
        self.master_id = master_id
        
        # State cache
        self.pos = 0.0
        self.vel = 0.0
        self.torque = 0.0
        self.enabled = False
        self.control_mode: Optional[ControlMode] = None
        self.param_cache: Dict[MotorReg, Any] = {}
        self.last_update_time: float = 0.0
        
        # Get limits for this motor type
        self.limits = MOTOR_LIMITS.get(motor_type)
        if self.limits is None:
            raise ValueError(f"Unknown motor type: {motor_type}")
        
        # Register callback for state updates
        self.bus.register_callback(self._handle_message)
    
    def _handle_message(self, msg: dict):
        """Handle incoming messages from bus."""
        msg_type = msg.get('type')
        motor_id = msg.get('motor_id')
        
        # Check if message is for this motor
        if not self._is_my_message(motor_id):
            return
        
        if msg_type == 'status':
            self._update_state_from_status(msg)
        elif msg_type == 'reg_response':
            self._update_param_cache(msg)
    
    def _is_my_message(self, motor_id: Optional[int]) -> bool:
        """Check if a message is for this motor."""
        if motor_id in (self.slave_id, self.master_id):
            return True
        # Fallback: when master_id is 0, match slave_id
        if self.master_id == 0 and motor_id == self.slave_id:
            return True
        return False
    
    def _update_state_from_status(self, msg: dict):
        """Update state cache from status message."""
        pos_raw = msg.get('pos_raw')
        vel_raw = msg.get('vel_raw')
        torque_raw = msg.get('torque_raw')
        
        if all(v is not None for v in [pos_raw, vel_raw, torque_raw]):
            from pydamiao.utils import uint_to_float
            self.pos = float(uint_to_float(
                np.uint16(pos_raw), 
                -self.limits.POS_MAX, 
                self.limits.POS_MAX, 
                16
            ))
            self.vel = float(uint_to_float(
                np.uint16(vel_raw), 
                -self.limits.VEL_MAX, 
                self.limits.VEL_MAX, 
                12
            ))
            self.torque = float(uint_to_float(
                np.uint16(torque_raw), 
                -self.limits.TORQUE_MAX, 
                self.limits.TORQUE_MAX, 
                12
            ))
            self.last_update_time = time.time()
    
    def _update_param_cache(self, msg: dict):
        """Update parameter cache from register response."""
        reg_id = msg.get('reg_id')
        value = msg.get('value')
        if reg_id is not None and value is not None:
            self.param_cache[MotorReg(reg_id)] = value
    
    def _set_mode_if_needed(self, target_mode: ControlMode) -> bool:
        """Switch to target mode if not already in it."""
        if self.control_mode != target_mode:
            result = self.write_param(MotorReg.CTRL_MODE, target_mode)
            if result:
                self.control_mode = target_mode
                return True
            return False
        return True
    
    # ===== Basic Commands =====
    
    def enable(self) -> bool:
        """Enable the motor."""
        frame = DamiaoProtocol.encode_basic_cmd(self.slave_id, DamiaoProtocol.CMD_ENABLE)
        success = self.bus.send(frame)
        if success:
            self.enabled = True
        return success
    
    def disable(self) -> bool:
        """Disable the motor."""
        frame = DamiaoProtocol.encode_basic_cmd(self.slave_id, DamiaoProtocol.CMD_DISABLE)
        success = self.bus.send(frame)
        if success:
            self.enabled = False
        return success
    
    def set_zero(self) -> bool:
        """Set current position as zero."""
        frame = DamiaoProtocol.encode_basic_cmd(self.slave_id, DamiaoProtocol.CMD_SET_ZERO)
        return self.bus.send(frame)
    
    def clear_error(self) -> bool:
        """
        Clear motor errors.
        
        Note: Currently returns False as the protocol command
        is not yet defined. Will be implemented when command is confirmed.
        """
        print("[Motor] clear_error not yet supported")
        return False
    
    # ===== Control Modes =====
    
    def set_mode(self, mode: ControlMode) -> bool:
        """Set control mode explicitly."""
        result = self.write_param(MotorReg.CTRL_MODE, mode)
        if result:
            self.control_mode = mode
            return True
        return False
    
    def set_mit(
        self, 
        kp: float, 
        kd: float, 
        pos: float, 
        vel: float, 
        torque: float
    ) -> bool:
        """
        MIT control mode. Automatically switches to MIT mode.
        
        Args:
            kp: Position gain (0-500)
            kd: Velocity gain (0-5)
            pos: Position command (rad)
            vel: Velocity command (rad/s)
            torque: Torque command (N·m)
            
        Returns:
            True if send successful
        """
        if not self._set_mode_if_needed(ControlMode.MIT):
            return False
        
        frame = DamiaoProtocol.encode_mit(
            self.slave_id, kp, kd, pos, vel, torque, self.limits
        )
        return self.bus.send(frame)
    
    def set_pos_vel(self, pos: float, vel: float) -> bool:
        """
        Position-velocity control mode. Automatically switches to POS_VEL mode.
        
        Args:
            pos: Position command (rad)
            vel: Velocity command (rad/s)
            
        Returns:
            True if send successful
        """
        if not self._set_mode_if_needed(ControlMode.POS_VEL):
            return False
        
        frame = DamiaoProtocol.encode_pos_vel(self.slave_id, pos, vel)
        return self.bus.send(frame)
    
    def set_velocity(self, vel: float) -> bool:
        """
        Velocity control mode. Automatically switches to VEL mode.
        
        Args:
            vel: Velocity command (rad/s)
            
        Returns:
            True if send successful
        """
        if not self._set_mode_if_needed(ControlMode.VEL):
            return False
        
        frame = DamiaoProtocol.encode_velocity(self.slave_id, vel)
        return self.bus.send(frame)
    
    def set_pos_force(
        self, 
        pos: float, 
        vel_cmd: float, 
        cur_cmd: float
    ) -> bool:
        """
        EMIT (torque-position) control mode. Automatically switches to TORQUE_POS mode.
        
        Args:
            pos: Position command (rad)
            vel_cmd: Velocity command (amplified 100x)
            cur_cmd: Current command (amplified 10000x)
            
        Returns:
            True if send successful
        """
        if not self._set_mode_if_needed(ControlMode.TORQUE_POS):
            return False
        
        frame = DamiaoProtocol.encode_torque_pos(self.slave_id, pos, vel_cmd, cur_cmd)
        return self.bus.send(frame)
    
    # ===== State & Parameters =====
    
    def refresh_state(self) -> bool:
        """
        Request latest motor state.
        
        Note: State will be updated asynchronously via callback.
        Use time.sleep() or polling to wait for update.
        
        Returns:
            True if send successful
        """
        frame = DamiaoProtocol.encode_query_status(self.slave_id)
        return self.bus.send(frame)
    
    def read_param(self, reg_id: MotorReg, timeout: float = 0.5) -> Optional[Any]:
        """
        Read a motor register parameter synchronously.
        
        Args:
            reg_id: Register to read
            timeout: Timeout in seconds
            
        Returns:
            Parameter value or None if failed
        """
        frame = DamiaoProtocol.encode_read_reg(self.slave_id, reg_id)
        response = self.bus.request_sync(
            frame,
            msg_type='reg_response',
            motor_id=self.slave_id,
            reg_id=reg_id,
            timeout=timeout
        )
        
        if response:
            value = response.get('value')
            self.param_cache[reg_id] = value
            return value
        return None
    
    def write_param(self, reg_id: MotorReg, value: float | int, timeout: float = 0.5) -> bool:
        """
        Write a motor register parameter synchronously.
        
        Args:
            reg_id: Register to write
            value: Value to write
            timeout: Timeout in seconds
            
        Returns:
            True if successful
        """
        frame = DamiaoProtocol.encode_write_reg(self.slave_id, reg_id, value)
        response = self.bus.request_sync(
            frame,
            msg_type='reg_response',
            motor_id=self.slave_id,
            reg_id=reg_id,
            timeout=timeout
        )
        
        if response:
            written_value = response.get('value')
            if written_value is not None:
                if abs(written_value - value) < 0.1:
                    self.param_cache[reg_id] = written_value
                    return True
        return False
    
    def save_params(self) -> bool:
        """
        Save current parameters to flash.
        
        Note: Motor should be disabled before saving.
        
        Returns:
            True if send successful
        """
        frame = DamiaoProtocol.encode_save_params(self.slave_id)
        return self.bus.send(frame)
    
    # ===== Properties =====
    
    @property
    def position(self) -> float:
        """Get current position (cached)."""
        return self.pos
    
    @property
    def velocity(self) -> float:
        """Get current velocity (cached)."""
        return self.vel
    
    @property
    def torque(self) -> float:
        """Get current torque (cached)."""
        return self.torque
    
    def __repr__(self) -> str:
        return (
            f"Motor(id={self.slave_id}, type={self.motor_type.name}, "
            f"pos={self.pos:.3f}, vel={self.vel:.3f}, torque={self.torque:.3f})"
        )
