# script to test transmit pulses out the USRP

from drivermsg_library import *
from rosmsg import *
from phasing_utils import calc_beam_azm_rad, calc_phase_increment, rad_to_rect
from radar_config_constants import *
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal


def test_burst_worker():
    import sys

    # setup to talk to usrp_driver, request clear frequency search
    usrp_drivers = ['localhost'] # hostname of usrp drivers, currently hardcoded to one
    usrp_driver_socks = []
    USRP_ANTENNA_IDX = [0]
    USRP_DRIVER_PORT = 54420
    
    for aidx in USRP_ANTENNA_IDX:
        usrp_driver_port = USRP_DRIVER_PORT + aidx

        try:
            cprint('connecting to usrp driver on port {}'.format(usrp_driver_port), 'blue')
            usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            usrpsock.connect(('localhost', usrp_driver_port))
            usrp_driver_socks.append(usrpsock)

        except ConnectionRefusedError:
            cprint('USRP server connection failed', 'blue')
            sys.exit(1)
    
    clrfreq_struct = clrfreqprm_struct(usrp_driver_socks)

    # simulate received clrfreq_struct
    clrfreq_struct.payload['start'] = 10050
    clrfreq_struct.payload['end'] = 13050
    clrfreq_struct.payload['filter_bandwidth'] = 3250
    clrfreq_struct.payload['pwr_threshold'] = .9
    clrfreq_struct.payload['nave'] =  10
    clear_freq, noise = clrfreq_search(clrfreq_struct, usrp_driver_socks, restricted_frequencies, 3, 3.24)
    print('clear frequency: {}, noise: {}'.format(clear_freq, noise))


if __name__ == '__main__':
    test_clrfreq()

    

