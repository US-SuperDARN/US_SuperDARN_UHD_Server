#!/usr/bin/python3
# test the cuda driver..
import unittest
import cuda_driver
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess
from pylab import *

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from shm_library import *

START_DRIVER = False


def create_testsequence():
    # fill ctrlprm with something reasonable, create test sequence

    import configparser

    ctrlprm = {\
    '' : np.zeros(120, dtype=np.uint8), \
    'radar' : 1, \
    'channel' : 1, \
    'local' : 0, \
    'priority' : 1, \
    'current_pulseseq_idx': 0, \
    'tbeam' : 0, \
    'tbeamcode' : 0, \
    'tbeamazm': 0, \
    'tbeamwidth': 0.0, \
    'tfreq': 11000, \
    'trise': 5000, \
    'number_of_samples' : 200, \
    'buffer_index' : 0, \
    'baseband_samplerate' : 3333.3333, \
    'filter_bandwidth' : 3333, \
    'match_filter' : 0, \
    'rfreq' : 11000, \
    'rbeam' : 0, \
    'rbeamcode' : 0, \
    'rbeamazm' : 0, \
    'rbeamwidth' : 0.0, \
    'status' : 0, \
    'pulseseq_idx' : 0}

    npulses = 1

    tr_to_pulse_delay = 60
    #pulse_offsets_vector = [1.35e-3, 6.15e-3, 12.15e-3]
    pulse_offsets_vector = [230, 21230, 33230, 36230, 40730, 46730, 63230, 64730]
    pulse_offsets_vector = [val/1e6 for val in pulse_offsets_vector]
    txbbrate = ctrlprm['baseband_samplerate']
    pulse_lens = [300] # length of pulse in baseband samples?
    phase_masks = [np.zeros(30)] # ... 
    pulse_masks = [np.zeros(30)]

#    usrp_config = configparser.ConfigParser()
#    usrp_config.read('usrp_config.ini')
    driver_config = configparser.ConfigParser()
    driver_config.read('../driver_config.ini')
    samplingRate_rx_rf = float(driver_config['cuda_settings']['FSampRX'])
    channelScalingFactor = 0.95
    nSequences = 5
    nSamples_rx_integrationPeriod = int(samplingRate_rx_rf * nSequences * (pulse_offsets_vector[-1] +  pulse_lens[0]/1e6 + 35e3 * 75 * 2 / 3e8))
    pdb.set_trace()
    seq = sequence(npulses, nSamples_rx_integrationPeriod, tr_to_pulse_delay, pulse_offsets_vector, pulse_lens, phase_masks, pulse_masks, channelScalingFactor, ctrlprm)
    return seq



if sys.hexversion < 0x030300F0:
    print('this code requires python 3.3 or greater')
    sys.exit(0)

def start_cudaserver():
    # open up ports..
    time.sleep(.5)
    if not START_DRIVER:
        return -1
    cuda_driver = subprocess.Popen(['python3', 'cuda_driver.py'])
    time.sleep(2)
    return cuda_driver.pid

def stop_cudaserver(sock, pid):
    # transmit clean exit command
    exitcmd = cuda_exit_command([sock])
    exitcmd.transmit()
    # kill the process just to be sure..
    if not START_DRIVER:
        return
    subprocess.Popen(['pkill', str(pid)])
    time.sleep(1)
    
class CUDA_ServerTestCases(unittest.TestCase):
    def setUp(self):
        time.sleep(1)
        self.pid = start_cudaserver()
        self.serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        max_connect_attempts = 5 
        for i in range(max_connect_attempts): 
            print('attempting connection to usrp_driver') 
            try: 
                self.serversock.connect(('localhost', 55420))
                break
            except:
                print('connecting to cuda driver failed on attempt ' + str(i + 1))
                time.sleep(5)

        rx_shm_list[0].append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'rx')))
        tx_shm_list.append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'tx')))

    def tearDown(self):
        global rx_shm_list, tx_shm_list
        stop_cudaserver(self.serversock, self.pid)
        self.serversock.close()
        rx_shm_list[0][0].close()
        rx_shm_list[0] = []
        tx_shm_list[0].close()
        tx_shm_list = []
   
    def test_cuda_downsample_and_filter(self):
        cprint('testing cuda get data (downsampling)', 'red')
        seq = create_testsequence()
	
        channel_number = 1
        nAntennas = 16
        nMainAntennas = 16
        rf_sampling_rate = 5e6 
        tone_frequency = 0
        usrp_center_frequency = 13e6

        bb_nSamples_per_antenna = seq.ctrlprm['number_of_samples']
        bb_sampling_rate = seq.ctrlprm['baseband_samplerate']
        usrp_channel_frequency = seq.ctrlprm['tfreq'] * 1000

        rf_nSamples_per_antenna = int(bb_nSamples_per_antenna * (rf_sampling_rate / bb_sampling_rate))

        signal_frequency = (usrp_channel_frequency - usrp_center_frequency) + tone_frequency
      

        # create rf and baseband test signals, copy it into shared memory, gpu accepts complex int16
        rf_test_signal = np.zeros((nAntennas, 2 * rf_nSamples_per_antenna), dtype=np.int16)
        expected_bb_signal = np.zeros((nAntennas, bb_nSamples_per_antenna), dtype=np.complex64)

        rf_time_vector = np.arange(rf_nSamples_per_antenna, dtype=np.float64) / rf_sampling_rate 
        bb_time_vector = np.arange(bb_nSamples_per_antenna, dtype=np.float64) / bb_sampling_rate

        #pdb.set_trace()
        rf_test_signal[0][0::2] = 1000 * np.sin(rf_time_vector * 2 * np.pi * signal_frequency)
        rf_test_signal[0][1::2] = 1000 * np.cos(rf_time_vector * 2 * np.pi * signal_frequency)

        expected_bb_signal[0][:] = 1000 * np.sin(bb_time_vector * 2 * np.pi * tone_frequency)

        # rx_shm is [swing][antenna][samples]
        # rf_test_signal is [antenna][samples]
        # we're only looking at swing a, antenna 0..
        # move samples into shared memory
        rx_shm_list[0][0].write(rf_test_signal[0].tobytes())
        
        tstart = time.time()
        # send channel information
        swing = 0
        cmd = cuda_add_channel_command([self.serversock], seq, swing)
        cmd.transmit()
        cmd.client_return()

        cprint('finished add channel command', 'red')
        # TODO: populate samples in shared memory
        
        # run generate pulse command to initialize memory..
        cmd = cuda_generate_pulse_command([self.serversock], swing)
        cmd.transmit()
        cmd.client_return()
        cprint('finished generate pulse command', 'red')


        # process samples from shared memory
        cmd = cuda_process_command([self.serversock], swing)
        cmd.transmit()
        cmd.client_return()
        cprint('finished process command', 'red')

        # copy processed samples
        cmd = cuda_get_data_command(self.serversock, swing)
        cmd.transmit()
        
        main_samples = None
        back_samples = None

        cudasock = self.serversock
        
        cprint('waiting for number of antennas from cuda_driver', 'red')
        nAntennas = recv_dtype(cudasock, np.uint32)
        cprint('collecting data from {} antennas'.format(nAntennas), 'red')
        transmit_dtype(cudasock, channel_number, np.int32)


        for iAntenna in range(nAntennas):
            antIdx = recv_dtype(cudasock, np.uint16)

            cprint('collecting samples from antenna {}'.format(antIdx), 'red')
            num_samples = recv_dtype(cudasock, np.uint32)
            samples = recv_dtype(cudasock, np.float32, num_samples)
            samples = samples[0::2] + 1j * samples[1::2] # unpacked interleaved i/q


            #... initialize main/back sample arrays once num_samples is known
            if main_samples is None:
                main_samples = np.zeros((4, 16, num_samples/2))	
                back_samples = np.zeros((4, 4, num_samples/2))

            if antIdx < nMainAntennas:
                main_samples[channel_number][antIdx] = samples[:]

            else:
                back_samples[channel_number][antIdx - nMainAntennas] = samples[:]


        transmit_dtype(cudasock, -1, np.int32) # send channel -1 to cuda driver to end transfer process

        tstop = time.time()
        cprint('finished collecting samples!', 'red')
        cprint('it took {} seconds'.format(tstop - tstart))

        cmd.client_return()
        '''
        # compare processed samples with expected samples 
        processed_samples = main_samples[channel_number][0] # pull samples from antenna 0
        expected_samples = expected_bb_signal[0] 
        pdb.set_trace()

        subplot(2,1,1)
        plt.plot(bb_time_vector, processed_samples)
        title('cuda processed samples')

        subplot(2,1,2)
        plt.plot(bb_time_vector, expected_samples)
        title('expected signal')
        plt.show()
        '''

    '''
    
    def test_cuda_tx_shm(self):
        cprint('testing cuda transmit sample generation', 'red')
        seq = create_testsequence()
        setupcmd = cuda_setup_command([self.serversock], seq) # cudas
        setupcmd.transmit()
        
    # TODO: test removing channels
    def test_channel_remove(self):
        print('testing removing a channel from a gpu')
  
     # TODO: test registering multiple channels
    def test_cuda_multichannel(self):
        print('testing adding multiple channels')

    '''

def dbPrint(msg):
    print(' {}: {}'.format(__file__, msg))  
       
if __name__ == '__main__':
    unittest.main()
