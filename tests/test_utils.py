import pytest
import numpy as np
from struct import pack, unpack

# Import the functions to test
from pydamiao.utils import (
    limit_min_max,
    float_to_uint,
    uint_to_float,
    float_to_uint8s,
    int_to_uint8s,
    is_in_ranges,
    uint8s_to_uint32,
    uint8s_to_float,
    print_hex,
)


class TestLIMITMINMAX:
    def test_within_range(self):
        assert limit_min_max(5, 0, 10) == 5
    
    def test_below_min(self):
        assert limit_min_max(-5, 0, 10) == 0
    
    def test_above_max(self):
        assert limit_min_max(15, 0, 10) == 10
    
    def test_edge_cases(self):
        assert limit_min_max(0, 0, 10) == 0
        assert limit_min_max(10, 0, 10) == 10


class TestFloatToUint:
    def test_normal_conversion(self):
        result = float_to_uint(5.0, 0.0, 10.0, 16)
        expected = np.uint16((5.0 / 10.0) * 65535)
        assert result == expected
    
    def test_clamped_values(self):
        # Below min should be clamped to min
        result_low = float_to_uint(-1.0, 0.0, 10.0, 16)
        expected_low = np.uint16(0)
        assert result_low == expected_low
        
        # Above max should be clamped to max
        result_high = float_to_uint(15.0, 0.0, 10.0, 16)
        expected_high = np.uint16(65535)
        assert result_high == expected_high
    
    def test_different_bit_widths(self):
        result_8bit = float_to_uint(5.0, 0.0, 10.0, 8)
        expected_8bit = np.uint16((5.0 / 10.0) * 255)
        assert result_8bit == expected_8bit


class TestUintToFloat:
    def test_normal_conversion(self):
        result = uint_to_float(np.uint16(32767), 0.0, 10.0, 16)
        # 32767 is approximately half of 65535
        expected = np.float32(5.0)
        assert abs(result - expected) < 0.01  # Allow small floating point error
    
    def test_edge_values(self):
        # Min value
        result_min = uint_to_float(np.uint16(0), 0.0, 10.0, 16)
        expected_min = np.float32(0.0)
        assert result_min == expected_min
        
        # Max value
        result_max = uint_to_float(np.uint16(65535), 0.0, 10.0, 16)
        expected_max = np.float32(10.0)
        assert result_max == expected_max


class TestFloatToUint8s:
    def test_positive_float(self):
        value = 3.14159
        result = float_to_uint8s(value)
        # Convert back to verify
        packed_back = pack('4B', *result)
        value_back = unpack('f', packed_back)[0]
        assert abs(value - value_back) < 1e-5
    
    def test_negative_float(self):
        value = -2.71828
        result = float_to_uint8s(value)
        packed_back = pack('4B', *result)
        value_back = unpack('f', packed_back)[0]
        assert abs(value - value_back) < 1e-5
    
    def test_zero(self):
        value = 0.0
        result = float_to_uint8s(value)
        packed_back = pack('4B', *result)
        value_back = unpack('f', packed_back)[0]
        assert value_back == 0.0


class TestDataToUint8s:
    def test_valid_uint32_values(self):
        test_values = [0, 1, 0xFFFFFFFF, 0x12345678]
        for value in test_values:
            result = int_to_uint8s(value)
            assert len(result) == 4
            # Verify by converting back
            packed_back = pack('4B', *result)
            value_back = unpack('I', packed_back)[0]
            assert value_back == value
    
    def test_invalid_values(self):
        invalid_values = [-1, 0x100000000, 1.5, "string"]
        for value in invalid_values:
            with pytest.raises(ValueError):
                int_to_uint8s(value)


class TestIsInRanges:
    def test_valid_ranges(self):
        # Test values in range 7-10
        for i in range(7, 11):
            assert is_in_ranges(i)
        
        # Test values in range 13-16
        for i in range(13, 17):
            assert is_in_ranges(i)
        
        # Test values in range 35-36
        for i in range(35, 37):
            assert is_in_ranges(i)
    
    def test_invalid_ranges(self):
        # Test values outside all ranges
        invalid_values = [0, 1, 6, 11, 12, 17, 34, 37, 100]
        for value in invalid_values:
            assert not is_in_ranges(value)


class TestUint8sToUint32:
    def test_various_values(self):
        test_cases = [
            (0x00, 0x00, 0x00, 0x00, 0x00000000),
            (0xFF, 0xFF, 0xFF, 0xFF, 0xFFFFFFFF),
            (0x78, 0x56, 0x34, 0x12, 0x12345678),
        ]
        
        for b1, b2, b3, b4, expected in test_cases:
            result = uint8s_to_uint32(b1, b2, b3, b4)
            assert result == expected


class TestUint8sToFloat:
    def test_various_floats(self):
        test_values = [0.0, 1.0, -1.0, 3.14159, -2.71828]
        
        for value in test_values:
            # Convert float to bytes first
            bytes_val = float_to_uint8s(value)
            # Convert back using uint8s_to_float
            result = uint8s_to_float(*bytes_val)
            assert abs(value - result) < 1e-5


class TestPrintHex:
    def test_print_hex(self, capsys):
        # Test that the function prints correctly
        data = [0x12, 0x34, 0xAB, 0xCD]
        print_hex(data)
        captured = capsys.readouterr()
        assert captured.out.strip() == "12 34 AB CD"

