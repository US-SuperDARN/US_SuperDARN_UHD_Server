# This is a pytest testing program

import sys
import numpy as np
import pytest

sys.path.insert(0, '../')

from utils import complex_float_to_int16


def test_complex_float_to_int16_integer_array():
    complex_number = np.array([1, 32767], dtype=complex)
    int_numbers = complex_float_to_int16(complex_number, "test")

    int_numbers[0] = int_numbers[0] >> 16
    int_numbers[1] = int_numbers[1] >> 16

    assert int_numbers == [1, 32767], "Should be [1, 32767]"


def test_complex_float_to_int16_complex_number():
    complex_number = np.array([1.+2.j], dtype=complex)
    int_number = complex_float_to_int16(complex_number, "test")

    int_numbers = [0, 0]
    int_numbers[0] = int_number[0] >> 16
    int_numbers[1] = int_number[0] % 16**2

    assert int_numbers == [1, 2], "Should be [1, 2(j)]"


# def test_complex_float_to_int16_overflow_error():
#     complex_number = np.array([32768], dtype=complex)
#     with pytest.raises(OverflowError):
#         int_number = complex_float_to_int16(complex_number, "test")
