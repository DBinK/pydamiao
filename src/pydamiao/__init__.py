# src/pydamiao/__init__.py
"""
PyDamiao - 达妙电机控制库

架构：总线设备 + 协议驱动 + 电机抽象层

- SerialBus: 串口总线层（线程安全、异步接收）
- MotorProtocol: 协议层（帧编码/解码）
- Motor: 单电机抽象层
- MotorManager: 多电机管理层
"""

from pydamiao.bus import SerialBus
from pydamiao.protocol import MotorProtocol
from pydamiao.motor import Motor
from pydamiao.manager import MotorManager
from pydamiao.types import MotorType, ControlMode, MotorReg, MotorLimits, MOTOR_LIMITS

__version__ = "2.0.0"

__all__ = [
    # 核心类
    "SerialBus",
    "MotorProtocol",
    "Motor",
    "MotorManager",
    
    # 类型定义
    "MotorType",
    "ControlMode",
    "MotorReg",
    "MotorLimits",
    "MOTOR_LIMITS",
]
