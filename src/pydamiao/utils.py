# src/pydamiao/utils.py

from collections.abc import Sequence
from struct import pack, unpack

import numpy as np


def limit_min_max(x: float, min_value: float, max_value: float) -> float:
    """Limits the value between min_value and max_value.
    
    Args:
        x: Input value to be limited.
        min_value: Minimum allowed value.
        max_value: Maximum allowed value.
        
    Returns:
        Value clamped between min_value and max_value.
    """
    return min(max(x, min_value), max_value)


def float_to_uint(x: float, x_min: float, x_max: float, bits: int) -> np.uint16:
    """Converts float to uint.
    
    Args:
        x: Input float value.
        x_min: Minimum range value.
        x_max: Maximum range value.
        bits: Number of bits for the output.
        
    Returns:
        Converted uint16 value.
    """
    x = limit_min_max(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, min: float, max: float, bits: int) -> np.float32:
    """Converts uint to float.
    
    Args:
        x: Input uint16 value.
        min: Minimum range value.
        max: Maximum range value.
        bits: Number of bits used in original conversion.
        
    Returns:
        Converted float32 value.
    """
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


def float_to_uint8s(value: float) -> tuple[int, int, int, int]:
    """Converts float to a tuple of uint8s.
    
    Args:
        value: Input float value.
        
    Returns:
        Tuple of four uint8 values representing the float in little-endian format.
    """
    # Pack the float into 4 bytes
    packed = pack('f', value)
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def int_to_uint8s(value: int) -> tuple[int, int, int, int]:
    """Converts int to a tuple of uint8s.
    
    Args:
        value: Input integer value (must be within uint32 range).
        
    Returns:
        Tuple of four uint8 values representing the uint32 in little-endian format.
        
    Raises:
        ValueError: If value is not within uint32 range.
    """
    # Check if the value is within the range of uint32
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # Pack the uint32 into 4 bytes
        packed = pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def uint8s_to_uint32(byte1: int, byte2: int, byte3: int, byte4: int) -> int:
    """Converts a group of uint8s to uint32.
    
    Args:
        byte1: First byte (least significant).
        byte2: Second byte.
        byte3: Third byte.
        byte4: Fourth byte (most significant).
        
    Returns:
        Combined uint32 value in little-endian order.
    """
    # Pack the four uint8 values into a single uint32 value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a uint32 value
    return unpack('<I', packed)[0]


def uint8s_to_float(byte1: int, byte2: int, byte3: int, byte4: int) -> float:
    """Converts a group of uint8s to float.
    
    Args:
        byte1: First byte (least significant).
        byte2: Second byte.
        byte3: Third byte.
        byte4: Fourth byte (most significant).
        
    Returns:
        Float value reconstructed from the bytes in little-endian order.
    """
    # Pack the four uint8 values into a single float value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a float value
    return unpack('<f', packed)[0]


def is_in_ranges(number: int) -> bool:
    """Checks if the number is in the specified ranges.
    
    Args:
        number: Input integer to check.
        
    Returns:
        True if number is in ranges [7-10], [13-16], or [35-36], False otherwise.
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


def print_hex(data: bytes | bytearray | Sequence[int]) -> None:
    """Prints the hex values of the bytes.
    
    Args:
        data: Input bytes, bytearray, or sequence of integers.
    """
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))