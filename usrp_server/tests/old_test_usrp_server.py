#!/usr/bin/python3
# test the cuda driver..

import sys
if sys.hexversion < 0x030300F0:
    print('This code requires Python 3.3 or greater, exiting..')
    sys.exit(0)


import unittest
import usrp_server 
import socket
import numpy as np
import subprocess 
import posix_ipc
import pdb
import time
import copy
import usrp_server

from drivermsg_library import *
from socket_utils import *
HOST = '127.0.0.1'
START_SERVER = True 


S_BIT = np.uint8(0x01) # sample impulses 
R_BIT = np.uint8(0x02) # tr gate, use for tx pulse times
X_BIT = np.uint8(0x04) # transmit path, use for bb 
A_BIT = np.uint8(0x08) # enable attenuator 
P_BIT = np.uint8(0x10) # phase code

CTRLPRM_DEFAULT = {\
    'radar' : 0, \
    'channel' : 0, \
    'local' : 0, \
    'priority' : 0, \
    'current_pulseseq_idx': 0, \
    'tbeam' : 0, \
    'tbeamcode' : 0, \
    'tbeamazm': 0, \
    'tbeamwidth': 0, \
    'tfreq': 10010, \
    'trise': 100, \
    'number_of_baseband_samples' : 500, \
    'buffer_index' : 0, \
    'baseband_samplerate' : 200000, \
    'filter_bandwidth' : 0, \
    'match_filter' : 0, \
    'rfreq' : 0, \
    'rbeam' : 10010, \
    'rbeamcode' : 0, \
    'rbeamazm' : 0, \
    'rbeamwidth' : 0, \
    'status' : 0}

def start_uhdserver():
    # open up ports..
    subprocess.Popen(['python3', 'usrp_server.py'])
    time.sleep(.1)

def stop_uhdserver(sock):
    send_servercmd(sock, usrp_server.CLEAN_EXIT, status = 0)

def send_servercmd(sock, cmd, status = 0):
    transmit_dtype(sock, np.int32(ord(cmd)))
    transmit_dtype(sock, np.int32(status))
  
class ServerTestCases(unittest.TestCase):
    def setUp(self):
        time.sleep(1)
        self.arbysockserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.arbysockserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.arbysockserver.bind((HOST, ARBYSERVER_PORT))
        self.arbysockserver.listen(1)

        self.usrpsockserver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.usrpsockserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.usrpsockserver.bind((HOST, USRPDRIVER_PORT))
        self.usrpsockserver.listen(1)

        self.cudasockserver= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.cudasockserver.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
        self.cudasockserver.bind((HOST, CUDADRIVER_PORT))
        self.cudasockserver.listen(1)
        if START_SERVER:
            start_uhdserver()

        (self.arbysock, addr) = self.arbysockserver.accept()
        (self.usrpsock, addr) = self.usrpsockserver.accept()
        (self.cudasock, addr) = self.cudasockserver.accept()
    
    def tearDown(self):
        stop_uhdserver(self.arbysock)
        self.arbysock.close()
        self.usrpsock.close()
        self.cudasock.close()
        self.arbysockserver.close()
        self.usrpsockserver.close()
        self.cudasockserver.close()
        time.sleep(1)
    ''' 
    def test_wait(self):
        send_servercmd(self.arbysock, usrp_server.WAIT)

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)

    def test_getstatus(self):
        send_servercmd(self.arbysock, usrp_server.GET_STATUS)

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)
    '''
    '''
    def test_registerseq(self):
        send_servercmd(self.arbysock, usrp_server.REGISTER_SEQ)

   
        
        # create test sequence
        # ###_______###_______##%%______
        # ~20 ms long
        # step is 5 us
        # 300 us pulse width
        # 5 ms pulse to pulse
        # 60 us TR to pulse

        step = 5e-6
        seqlen = 20e-3
        smsep = 300e-6
        nsamples = 66
        pulses = [1.35e-3, 6.15e-3, 12.15e-3]
        trtotx = 50e-6
        pulselen = 300e-6
        t_seq = smsep * nsamples

        # create ctrlprm struct
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)

        # send ctrlprm
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()
 
        # create decompress vector, then run length encode..

        # create sample_mask, S_BIT every 60 bins
        sample_mask = np.zeros(int(np.round(smsep / step)))
        sample_mask[0] = S_BIT
        sample_mask = np.tile(sample_mask, nsamples)
        
        # create tr_mask
        tr_mask = np.zeros(len(sample_mask))
        for pulse in pulses:
            trstartidx = int(np.round((pulse - trtotx)  / step))
            trstopidx = int(np.round((pulse + trtotx + pulselen) / step))
            tr_mask[trstartidx:trstopidx] = R_BIT

        # create transmit mask  
        x_mask = np.zeros(len(sample_mask))
        for pulse in pulses:
            startidx = int(np.round((pulse)  / step))
            stopidx = int(np.round((pulse + pulselen) / step))
            x_mask[startidx:stopidx] = X_BIT
   
        
        # create phase mask  
        p_mask = np.zeros(len(sample_mask))
        startidx = int(np.round((pulses[-1] + pulselen / 2) / step))
        stopidx = int(np.round((pulses[-1] + pulselen) / step))
        p_mask[startidx:stopidx] = P_BIT

        seq_buf = sample_mask + tr_mask + x_mask + p_mask
        
        # compress seq_buf
        # rle, see http://mail.scipy.org/pipermail/numpy-discussion/2007-October/029378.html
        pos, = np.where(np.diff(seq_buf) != 0)
        pos = np.concatenate(([0],pos+1,[len(seq_buf)]))
        rle = [(a,b,seq_buf[a]) for (a,b) in zip(pos[:-1],pos[1:])]        
         
        tsg_rep = []
        tsg_code = []

        for c in rle:
            tsg_code.append(c[2])
            tsg_rep.append(c[1] - c[0])

        transmit_dtype(self.arbysock, np.int32(0)) # seq_idx
        transmit_dtype(self.arbysock, np.int32(0)) # tsg_idx
        transmit_dtype(self.arbysock, np.int32(len(tsg_code))) # tsg_len TODO: create this
        transmit_dtype(self.arbysock, np.int32(5)) # tsg_step TODO: what are the units on this?

        transmit_dtype(self.arbysock, np.int8(tsg_rep))
        transmit_dtype(self.arbysock, np.int8(tsg_code))

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)
    ''' 
    '''
    def test_ctrlprgready(self):
        send_servercmd(self.arbysock, usrp_server.CTRLPROG_READY)
        
        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()

        # check status and close up ports

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)

    def test_ctrlprogend(self):
        send_servercmd(self.arbysock, usrp_server.CTRLPROG_END)
        
        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()

        # check status and close up ports

        drivertype  = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)
    
    '''

    '''
    def test_get_data(self):
        # first, register sequence
        print('testing get_data')
        self.test_registerseq()

        print('registered test sequence')
        # next, request data for that channel
        send_servercmd(self.arbysock, usrp_server.RECV_GET_DATA)
        
        print('requesting data')
        # next, request data for that channel
        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()

        print('waiting for sample status')
        # check status
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)
        print('received status: ' + str(status))

        # pretend to be a usrp driver, send status okay..
        transmit_dtype(self.usrpsock, np.int32(1))
    

        # pretend to be a usrp driver, antenna data samples
        transmit_dtype(self.usrpsock, np.int32(1)) # transmit number of requested antennas
        transmit_dtype(self.usrpsock, np.int32(0)) # transmit array of requested antenna indexes
        # and some samples
        nbb_samples = ctrlprm['number_of_baseband_samples']
        main_samples = np.zeros(nbb_samples, dtype=np.float64)
        back_samples = np.zeros(nbb_samples, dtype=np.float64)
        transmit_dtype(self.usrpsock, main_samples) # transmit number of requested antennas

        print('waiting for sample metadata')
        # receive shm/socket config
        shm_config = recv_dtype(self.arbysock, np.int32)
        frame_header = recv_dtype(self.arbysock, np.int32)
        buffer_number = recv_dtype(self.arbysock, np.int32)
        nsamples = recv_dtype(self.arbysock, np.int32)
    
        print('waiting for samples')
        # receive sample vector
        main_beamformed = recv_dtype(self.arbysock, np.uint32, nsamples)
        #back_beamformed = recv_dtype(self.arbysock, np.uint32, nsamples)

        # TODO: is this expected to be sent again?
        drivertype = recv_dtype(self.arbysock, np.int32)
        status = recv_dtype(self.arbysock, np.int32)
        print('get data test complete')

    def test_trigger(self):
        send_servercmd(self.arbysock, usrp_server.TRIGGER)
        status = recv_dtype(self.arbysock, np.int32)
        self.assertTrue(status == 0)


    def test_pretrigger_newbeam(self):
        # load test sequence
        self.test_registerseq()

        # send pretrigger command
        send_servercmd(self.arbysock, usrp_server.PRETRIGGER)

        # send ctrlprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()
       
        # accept new usrp setup command as usrp_driver
        cmd = usrp_setup_command([self.usrpsock], CTRLPRM_DEFAULT, 0, 10000)
        cmd.receive()

        # receive bad_tx data 
        badtx_len = recv_dtype(self.arbysock, np.int32)
        badtx_start = recv_dtype(self.arbysock, np.int32)
        badtx_dur = recv_dtype(self.arbysock, np.int32)

    
    # TODO: write this test
    def test_posttrigger(self):
        pass

    '''

    def test_clrfreq_narrow(self):
        send_servercmd(self.arbysock, usrp_server.CLRFREQ)
       
        fstart = 10000
        fstop =  10100
        filter_bandwidth = 1
        power_threshold = .9
        nave = 1

        # send clrfreq struct
        transmit_dtype(self.arbysock, np.int32(fstart)) # kHz
        transmit_dtype(self.arbysock, np.int32(fstop)) # kHz
        transmit_dtype(self.arbysock, np.float32(filter_bandwidth)) # kHz (c/(2 * rsep))
        transmit_dtype(self.arbysock, np.float32(power_threshold)) # (typically .9, threshold before changing freq)
        transmit_dtype(self.arbysock, np.int32(nave)) 

        
        # and send a ctlrprm
        ctrlprm = copy.copy(CTRLPRM_DEFAULT)
        ctrlprm_struct = server_ctrlprm(self.arbysock, ctrlprm)
        ctrlprm_struct.transmit()
    
        print('test_clrfreq_narrow - waiting for UHD_GETTIME command')
        # pretend to be USRP, send back current time
        assert(recv_dtype(self.usrpsock, np.uint8) == usrp_server.UHD_GETTIME)
        transmit_dtype(self.usrpsock, np.int32(10))
        transmit_dtype(self.usrpsock, np.float64(.5))
        transmit_dtype(self.usrpsock, np.uint8(usrp_server.UHD_GETTIME))
        
        print('test_clrfreq_narrow - waiting for CLRFREQ command to usrp_driver')
        assert(recv_dtype(self.usrpsock, np.uint8) == UHD_CLRFREQ)
        uhd_num_clrfreq_samples = recv_dtype(self.usrpsock, np.int32)
        uhd_time_full = recv_dtype(self.usrpsock, np.int32)
        uhd_time_frac = recv_dtype(self.usrpsock, np.float64)
        uhd_clr_rate = recv_dtype(self.usrpsock, np.float64)
   
        
        print('test_clrfreq_narrow - sending fake clrfreq samples')
        clrfreq_samples = np.zeros(uhd_num_clrfreq_samples, dtype = np.complex64)
        transmit_dtype(self.usrpsock, clrfreq_samples) 

        print('test_clrfreq_narrow - waiting for updated clrfreq frequencies')
        fstart = recv_dtype(self.arbysock, np.int32) # kHz
        fstop = recv_dtype(self.arbysock, np.int32) # kHz
        filter_bandwidth = recv_dtype(self.arbysock, np.float32) # kHz (c/(2 * rsep))
        power_threshold = recv_dtype(self.arbysock, np.float32) # (typically .9, threshold before changing freq)
        nave = recv_dtype(self.arbysock, np.int32) # (typically .9, threshold before changing freq)

        usable_bandwidth = recv_dtype(self.arbysock, np.float64)
        pwr = recv_dtype(self.arbysock, np.float64, nitems = usable_bandwidth) # length usable_bandwidth array?
        
        assert recv_dtype(self.arbysock, np.uint8) == usrp_server.UHD_CLRFREQ, 'CLRFREQ should return CLRFREQ command upon success'

if __name__ == '__main__':
    unittest.main()
