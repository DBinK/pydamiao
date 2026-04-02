# src/pydamiao/__init__.py
"""
PyDamiao - Damiao Motor Control Library

Architecture:
- SerialBus: Simple thread-safe serial communication
- DamiaoProtocol: Protocol encoding/decoding  
- Motor: Single motor abstraction with auto mode switching
- MotorManager: Multi-motor management and batch operations
- Result/DamiaoError: Pythonic error handling

Quick Start:
    from pydamiao import SerialBus, MotorManager, MotorType
    
    with MotorManager(SerialBus("/dev/ttyUSB0")) as manager:
        motor = manager.add_motor(MotorType.DM4310, slave_id=0x06)
        
        # Control commands auto-switch mode
        motor.enable()
        motor.set_pos_vel(1.0, 0.5)
        
        # Read cached state
        print(f"Position: {motor.position}")
        
        # Sync parameter access
        pmax = motor.read_param(MotorReg.PMAX)
"""

from pydamiao.bus import SerialBus
from pydamiao.protocol import DamiaoProtocol
from pydamiao.driver import Motor
from pydamiao.manager import MotorManager
from pydamiao.result import Result, DamiaoError, check_result
from pydamiao.types import MotorType, ControlMode, MotorReg, MOTOR_LIMITS
from pydamiao.utils import (
    float_to_uint8s,
    int_to_uint8s,
    uint8s_to_float,
    uint8s_to_uint32,
    float_to_uint,
    uint_to_float,
)

__version__ = "2.1.0"

__all__ = [
    # Core classes
    "SerialBus",
    "DamiaoProtocol",
    "Motor",
    "MotorManager",
    # Error handling
    "Result",
    "DamiaoError",
    "check_result",
    # Types
    "MotorType",
    "ControlMode",
    "MotorReg",
    "MOTOR_LIMITS",
    # Utilities
    "float_to_uint8s",
    "int_to_uint8s",
    "uint8s_to_float",
    "uint8s_to_uint32",
    "float_to_uint",
    "uint_to_float",
]