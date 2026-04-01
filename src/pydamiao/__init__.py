# src/pydamiao/__init__.py
from pydamiao.motor import Motor, MotorControl
from pydamiao.types import MotorType, ControlMode, MotorReg

__all__ = [
    "Motor",
    "MotorControl",
    "MotorType",
    "ControlMode",
    "MotorReg"
]