
import sys
sys.path.append("..")
sys.path.insert(0, '../../python_include')

from usrp_server import RadarHardwareManager


def test_default_rhm():
    rhm = RadarHardwareManager(0)
    assert rhm.exit_usrp_server is False
    assert rhm.processing_swing_invalid is False
