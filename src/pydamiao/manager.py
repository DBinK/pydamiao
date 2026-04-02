# src/pydamiao/manager.py

from typing import Dict, List, Optional

from pydamiao.bus import SerialBus
from pydamiao.driver import Motor
from pydamiao.types import MotorType


class MotorManager:
    """
    Multi-motor management layer.
    
    Provides:
    - Motor registration and lookup
    - Batch operations (enable_all, disable_all, etc.)
    - Shared SerialBus management
    
    Usage:
        with MotorManager(SerialBus("/dev/ttyUSB0")) as manager:
            motor1 = manager.add_motor(MotorType.DM4310, slave_id=0x06)
            motor2 = manager.add_motor(MotorType.DM4310, slave_id=0x07)
            
            manager.enable_all()
            
            # Access motors by ID
            motor1.set_pos_vel(1.0, 0.5)
            manager[0x07].set_velocity(2.0)
    """
    
    def __init__(self, bus: SerialBus):
        """
        Initialize motor manager.
        
        Args:
            bus: Shared SerialBus instance
        """
        self.bus = bus
        self._motors: Dict[int, Motor] = {}
    
    def add_motor(
        self, 
        motor_type: MotorType, 
        slave_id: int, 
        master_id: int = 0
    ) -> Motor:
        """
        Create and register a new motor.
        
        Args:
            motor_type: Motor type enum
            slave_id: Motor CAN ID
            master_id: Master ID (default 0)
            
        Returns:
            Created Motor object
        """
        motor = Motor(self.bus, motor_type, slave_id, master_id)
        self._motors[slave_id] = motor
        if master_id != 0:
            self._motors[master_id] = motor
        return motor
    
    def register(self, motor: Motor) -> None:
        """
        Register an existing motor object.
        
        Args:
            motor: Motor object to register
        """
        self._motors[motor.slave_id] = motor
        if motor.master_id != 0:
            self._motors[motor.master_id] = motor
    
    def get(self, motor_id: int) -> Optional[Motor]:
        """
        Get a motor by ID.
        
        Args:
            motor_id: Motor ID (slave or master)
            
        Returns:
            Motor object or None if not found
        """
        return self._motors.get(motor_id)
    
    def __getitem__(self, motor_id: int) -> Motor:
        """Get a motor by ID (dictionary-style access)."""
        return self._motors[motor_id]
    
    def __contains__(self, motor_id: int) -> bool:
        """Check if motor ID is registered."""
        return motor_id in self._motors
    
    def __len__(self) -> int:
        """Get number of registered motors."""
        return len(self._motors)
    
    def list_motors(self) -> List[Motor]:
        """Get list of all registered motors."""
        return list(self._motors.values())
    
    # ===== Batch Operations =====
    
    def enable_all(self) -> Dict[int, bool]:
        """Enable all registered motors."""
        results = {}
        for motor_id, motor in self._motors.items():
            if motor_id == motor.slave_id:  # Only call once per physical motor
                results[motor_id] = motor.enable()
        return results
    
    def disable_all(self) -> Dict[int, bool]:
        """Disable all registered motors."""
        results = {}
        for motor_id, motor in self._motors.items():
            if motor_id == motor.slave_id:
                results[motor_id] = motor.disable()
        return results
    
    def set_zero_all(self) -> Dict[int, bool]:
        """Set zero position for all motors."""
        results = {}
        for motor_id, motor in self._motors.items():
            if motor_id == motor.slave_id:
                results[motor_id] = motor.set_zero()
        return results
    
    def refresh_all_states(self) -> Dict[int, bool]:
        """Request state refresh for all motors."""
        results = {}
        for motor_id, motor in self._motors.items():
            if motor_id == motor.slave_id:
                results[motor_id] = motor.refresh_state()
        return results
    
    def save_all_params(self) -> Dict[int, bool]:
        """
        Save parameters to flash for all motors.
        
        Note: Motors should be disabled before saving.
        """
        results = {}
        for motor_id, motor in self._motors.items():
            if motor_id == motor.slave_id:
                results[motor_id] = motor.save_params()
        return results
    
    # ===== Cleanup =====
    
    def shutdown(self):
        """Shutdown the manager and close the bus."""
        self.disable_all()
        self.bus.close()
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensures cleanup."""
        self.shutdown()
        return False
