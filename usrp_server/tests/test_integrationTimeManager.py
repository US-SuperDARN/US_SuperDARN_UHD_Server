
import pytest

import sys
sys.path.append("..")
sys.path.insert(0, '../../python_include')

from usrp_server import integrationTimeManager, RadarHardwareManager

'''
@pytest.fixture
def RHM():
    return RadarHardwareManager(0)

def test_default_integrationTimeManager(RHM):
    manager = integrationTimeManager(RHM)
    assert manager.last_start == None

'''
