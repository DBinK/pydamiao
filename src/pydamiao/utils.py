# src/pydamiao/utils.py

from collections.abc import Sequence
from struct import pack, unpack

import numpy as np


def print_hex(data: bytes | bytearray | Sequence[int]) -> None:
    """打印字节的十六进制值"""
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))


# ==============================================================================
# 一些打包数据包用的函数
# ==============================================================================

def limit_min_max(x: float, min_value: float, max_value: float) -> float:
    """限制数值在最小值和最大值之间"""
    return min(max(x, min_value), max_value)

def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> np.uint16:
    """将浮点数转换为无符号整数"""
    x = limit_min_max(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))

def uint_to_float(x: np.uint16, min: float, max: float, bits: int) -> np.float32:
    """将无符号整数转换为浮点数"""
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)

def float_to_uint8s(value: float) -> tuple[int, int, int, int]:
    """将浮点数转换为uint8元组"""
    # 将浮点数打包成4字节
    packed = pack('f', value)
    # 将字节解包为四个uint8值
    return unpack('4B', packed)

def int_to_uint8s(value: int) -> tuple[int, int, int, int]:
    """将整数转换为uint8元组"""
    # 检查值是否在uint32范围内
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # 将uint32打包成4字节
        packed = pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")
    # 将字节解包为四个uint8值
    return unpack('4B', packed)

def uint8s_to_uint32(byte1: int, byte2: int, byte3: int, byte4: int) -> int:
    """将uint8组合转换为uint32"""
    # 将四个uint8值按小端序打包为单个uint32值
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # 解包为uint32值
    return unpack('<I', packed)[0]

def uint8s_to_float(byte1: int, byte2: int, byte3: int, byte4: int) -> float:
    """将uint8组合转换为浮点数"""
    # 将四个uint8值按小端序打包为单个浮点值
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # 解包为浮点值
    return unpack('<f', packed)[0]


def is_in_ranges(value: int) -> bool:
    """兼容旧测试的范围判断函数."""
    return (7 <= value <= 10) or (13 <= value <= 16) or (35 <= value <= 36)
