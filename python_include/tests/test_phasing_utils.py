# This is a pytest testing program

import sys
import numpy as np
import pytest

sys.path.insert(0, '../')

from phasing_utils import calc_beam_azm_rad, calc_phase_increment


# def test_rad_to_rect():
#     number = np.complex(1.0, 1.0)
#     angle = 0.78539  # 45 degrees
#
#     assert np.real(rad_to_rect(angle)) == pytest.approx(0.707, 0.2)
#     assert np.imag(rad_to_rect(angle)) == pytest.approx(0.707, 0.2)

def test_calc_beam_azm_rad():
    number_of_beams = 16
    beam_separation_angle = 3.24
    beam = [0, 15, 8]
    expected_azmuth = [-0.424115, 0.424115, 0.02827433]

    for beam_number in range(len(beam)):
        azmuth_angle = calc_beam_azm_rad(number_of_beams, beam[beam_number], beam_separation_angle)
        assert azmuth_angle == pytest.approx(expected_azmuth[beam_number], 0.000001)


def test_calc_phase_increment():
    transmit_frequency = 10000000  # 10 MHz
    beam_azmuth = [-0.424115, 0.424115, 0.02827433]
    antenna_spacing = 15.24

    expected_phase = [-1.313496, 1.313496, 0.090236]

    for beam in range(len(beam_azmuth)):
        phase_shift = calc_phase_increment(beam_azmuth[beam], transmit_frequency, antenna_spacing)
        assert phase_shift == pytest.approx(expected_phase[beam], 0.00001)
