# src/pydamiao/utils.py

from collections.abc import Sequence
from struct import pack, unpack

import numpy as np

def limit_min_max(x: float, min_value: float, max_value: float) -> float:
    """limit the value between min_value and max_value"""
    return min(max(x, min_value), max_value)


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> np.uint16:
    """convert float to uint"""
    x = limit_min_max(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))

def uint_to_float(x: np.uint16, min: float, max: float, bits: int) -> np.float32:
    """convert uint to float"""
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)

def float_to_uint8s(value: float) -> tuple[int, int, int, int]:
    """convert float to a tuple of uint8s"""
    # Pack the float into 4 bytes
    packed = pack('f', value)
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)

def int_to_uint8s(value: int) -> tuple[int, int, int, int]:
    """convert int to a tuple of uint8s"""
    # Check if the value is within the range of uint32
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # Pack the uint32 into 4 bytes
        packed = pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)

def uint8s_to_uint32(byte1: int, byte2: int, byte3: int, byte4: int) -> int:
    """convert a group of uint8s to uint32"""
    # Pack the four uint8 values into a single uint32 value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a uint32 value
    return unpack('<I', packed)[0]

def uint8s_to_float(byte1: int, byte2: int, byte3: int, byte4: int) -> float:
    """convert a group of uint8s to float"""
    # Pack the four uint8 values into a single float value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a float value
    return unpack('<f', packed)[0]


def is_in_ranges(number: int) -> bool:
    """
    check if the number is in the range of uint32
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False

def print_hex(data: bytes | bytearray | Sequence[int]) -> None:
    """print the hex values of the bytes"""
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))
