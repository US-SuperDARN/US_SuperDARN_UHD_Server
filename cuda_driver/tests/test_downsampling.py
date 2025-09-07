#!/usr/bin/python3
# test the cuda driver..
#import unittest
import cuda_driver
import numpy as np
import sys
import posix_ipc
import pdb
import time
import subprocess
import pycuda.driver as cuda
import myPlotTools as mpt

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from shm_library import *


CUDADRIVER_PORT = 55420

def stop_cudaserver(sock):
    # transmit clean exit command
    exitcmd = cuda_exit_command([sock])
    exitcmd.transmit()
    time.sleep(1)
    
def tearDown(serversock):
    global rx_shm_list, tx_shm_list
    stop_cudaserver(serversock)
    serversock.close()
    rx_shm_list[0][0].close()
    rx_shm_list[0] = []
   # tx_shm_list[0].close()
   # tx_shm_list = []

def dbPrint(msg):
    print(' {}: {}'.format(__file__, msg))


serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

max_connect_attempts = 5 
for i in range(max_connect_attempts): 
    print('attempting connection to cuda_driver at localhost:{}'.format(CUDADRIVER_PORT)) 
    try: 
        serversock.connect(('localhost', CUDADRIVER_PORT))
        break
    except:
        print('connecting to cuda driver failed on attempt ' + str(i + 1))
        time.sleep(5)



   
        
cprint('testing cuda get data (downsampling)', 'red')


# get test sequence and adjust ...
seq = create_testsequence_uafscan()

seq.ctrlprm['rfreq'] = 16e6 / 1000
seq.ctrlprm['tfreq'] = 16e6 / 1000

temp = seq.phase_masks
temp = [np.zeros(1,dtype=np.uint8) for idx in range(len(temp))]
seq.phase_masks = temp




nSamplesBB = seq.ctrlprm['number_of_samples']
bbrate_rx = seq.ctrlprm['baseband_samplerate']

# values from ini file
fsamprx = 10e6
usrp_mix_freq = 13e6
rx_shm_size = 160000000 # from driver confi.ini

nSamples_rx_rf = int(np.round(nSamplesBB / bbrate_rx * fsamprx))

cudasocks = [] # TODO...
#rx_shm_list[0].append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'rx')))
rx_shm_list[0].append(cuda_driver.create_shm(ANTENNA, SWING0, SIDEA, rx_shm_size, direction = 'rx'))
#tx_shm_list.append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'tx')))


cmd = cuda_setup_command([serversock], seq) # cudas
cmd.transmit()
cmd.client_return()

cmd = cuda_add_channel_command([serversock], seq) 
cmd.transmit()
cmd.client_return()


cmd = cuda_generate_pulse_command(serversock)
cmd.transmit()
cmd.client_return()


nAnts = 1
rx_rf_data = np.zeros((nAnts, 1, 2 * nSamples_rx_rf), dtype=np.int16)
 
timeVector = np.array(range(nSamples_rx_rf)) / fsamprx
maxAmp = 2**15 * 0.9

# sine
#sig  = np.sin(2*np.pi*carrierFreq*timeVector)*maxAmp
# bandpass signal
#sig  = np.sinc((timeVector-timeVector[nSamples_rx_rf/2-1])*np.pi*10000) * np.sin(2*np.pi*carrierFreq*timeVector) * maxAmp

B = 1000 # bandwidth
pulseFreq = seq.ctrlprm['rfreq']*1000 - usrp_mix_freq 
sig = np.sinc((timeVector-timeVector[int(nSamples_rx_rf/2-1)]) * B ) * np.exp(1j*2*np.pi*pulseFreq*timeVector) *maxAmp

# add other frequencies (think of faster way...)
if False:
  sig += np.convolve(np.sinc((timeVector-timeVector[int(nSamples_rx_rf/2-1)]) * B*2 ), np.random.rand(nSamples_rx_rf), mode='same') * np.exp(1j*2*np.pi*(pulseFreq - 2e6) *timeVector) *maxAmp

# apply window
if True: 
   nSamples = 25000
   window = np.hanning(nSamples*2)
   sig[0:nSamples] *= window[0:nSamples]
   sig[-nSamples:] *= window[-nSamples:]


sig = np.round(sig) # important: typecast uses floor and this introduces distortions

rx_rf_data[0][0][0::2] = np.int16(sig.real)
rx_rf_data[0][0][1::2] = np.int16(sig.imag)

# no need for noise since quantizing noise (16 bit) is enough
# rx_rf_data[0][0][:] += np.int16((np.random.rand(nSamples_rx_rf*2)-0.5)*maxAmp/100000  )

import myPlotTools as mpt
#pdb.set_trace()
#mpt.plot_time_freq(sig, fsamprx)
#mpt.plot_time_freq(rx_rf_data[0][0], fsamprx, iqInterleaved=True)


 
# inceasing numbers
#rx_rf_data[0][0][:] = np.array(range(nSamples_rx_rf*2)) 

 
rx_shm_list[0][0].seek(0)
rx_shm_list[0][0].write(rx_rf_data[0][0].tobytes())
rx_shm_list[0][0].flush()


cmd = cuda_process_command([serversock])
cmd.transmit()
cmd.client_return()



# Copy from usrm_server.py
cmd = cuda_get_data_command(serversock)

cmd.transmit()

# TODO: setup through all sockets
# recv metadata from cuda drivers about antenna numbers and whatnot
# fill up provided main and back baseband sample buffers

# TODO: fill in sample buffer with baseband sample array
cudasock = serversock
    # TODO: recieve antenna number

transmit_dtype(cudasock, 0 , np.int32)
nants = recv_dtype(cudasock, np.uint32)
dbPrint(' received nants= {}'.format( nants))
main_samples = np.complex64(np.zeros((nants, nSamplesBB)))


for ant in range(nants):
    ant = recv_dtype(cudasock, np.uint16)
    dbPrint('received idx ant = {}'.format(ant))
    num_samples = recv_dtype(cudasock, np.uint32)
    samples = recv_dtype(cudasock, np.float32, num_samples)
     
#    if ant < MAX_MAIN_ARRAY_TODO:
    main_samples[ant] = samples[0::2] + samples[1::2] *1j
 #   else:
 #      back_samples[ant - MAX_MAIN_ARRAY_TODO] = samples
		   
# samples is interleaved I/Q float32 [self.nants, self.nchans, nbbsamps_rx]

# TODO: currently assuming samples is main samples, no back array!


# pdb.set_trace()
cmd.client_return


##mpt.plot_time_freq(main_samples[0], bbrate_rx)
#pdb.set_trace()

# setupcmd = cuda_get_data_command([self.serversock]) #seq 
# setupcmd.transmit()

# transmit_dtype(self.serversock, 0, np.int32)
# nAnts = recv_dtype(self.serversock, np.int32)

stop_cudaserver(serversock)
tearDown(serversock)










