#!/usr/bin/python3
# test the cuda driver..
#import unittest
import cuda_driver
import numpy as np
import sys

import pdb
import time

import pycuda.driver as cuda
import myPlotTools as mpt

from termcolor import cprint

from drivermsg_library import *
from socket_utils import *
from shm_library import *

f = open('cuda.dump.tx', 'w+') 
f.close()

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

seq.ctrlprm['rfreq'] = 10e6 / 1000
seq.ctrlprm['tfreq'] = 10e6 / 1000

temp = seq.phase_masks
temp = [np.zeros(1,dtype=np.uint8) for idx in range(len(temp))]
seq.phase_masks = temp

nSamplesBB = seq.ctrlprm['number_of_samples']
bbrate_rx  = seq.ctrlprm['baseband_samplerate']
seq.ctrlprm['tbeam'] = 0 
seq.ctrlprm['rbeam'] =  0
print("tbeam {} rbeam: {}".format(seq.ctrlprm['tbeam'], seq.ctrlprm['rbeam']))


# second channel 
seq2 = create_testsequence_uafscan()
seq2.ctrlprm['channel'] = 1
seq2.ctrlprm['rfreq'] = 10.02e6 / 1000
seq2.ctrlprm['tfreq'] = 10.02e6 / 1000
seq2.phase_masks = temp
seq2.ctrlprm['tbeam'] = 6 
seq2.ctrlprm['rbeam'] = 6



# values from ini file
fsamprx = 16e6

rx_shm_size = 160000000 # from driver confi.ini

nSamples_rx_rf = int(np.round(nSamplesBB / bbrate_rx * fsamprx))

cudasocks = [] # TODO...
rx_shm_list[0].append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'rx')))
rx_shm_list[0].append(cuda_driver.create_shm(ANTENNA, SWING0, SIDEA, rx_shm_size, direction = 'rx'))
tx_shm_list.append(shm_mmap(shm_namer(ANTENNA, SWING0, SIDEA, direction = 'tx')))


cmd = cuda_setup_command([serversock], seq) # cudas
cmd.transmit()
cmd.client_return()

cmd = cuda_add_channel_command([serversock], seq) 
cmd.transmit()
cmd.client_return()

cmd = cuda_add_channel_command([serversock], seq2) 
cmd.transmit()
cmd.client_return()

cmd = cuda_generate_pulse_command(serversock)
cmd.transmit()
cmd.client_return()



stop_cudaserver(serversock)
tearDown(serversock)










