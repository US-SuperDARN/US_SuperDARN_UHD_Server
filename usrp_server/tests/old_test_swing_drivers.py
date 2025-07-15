#!/usr/bin/python3 
# test the swing buffers with usrp_driver and cuda_driver 
import unittest 
import numpy as np 
import sys 
import os
import posix_ipc 
import pdb 
import time 
import subprocess 
import configparser 
import datetime 
 
from termcolor import cprint 
sys.path.insert(0, '../python_include') 
sys.path.insert(0, '../cuda_driver') 
 
from drivermsg_library import * 
from socket_utils import * 
from cuda_driver import * 
from radar_config_constants import * 
import clear_frequency_search 
import rosmsg 
import test_cuda_driver 

 
 
START_DRIVER = False 
ANTENNA_UNDER_TEST = [0, 1, 2, 3, 4 ,5, 6, 7] 
#ANTENNA_UNDER_TEST = [0] 
INTEGRATION_PERIOD_SYNC_TIME = 0.3  # todo: get from file
 

# init logging
logFile = open("../log/test_swing.log", "wt")

def logmsg(msg):
   logFile.write("{} - {}\n".format(datetime.now().strftime("%H:%M:%S:%f"), msg) )



# parse gpu config file
driverconfig = configparser.ConfigParser()
driverconfig.read('../driver_config.ini')
shm_settings = driverconfig['shm_settings']
cuda_settings = driverconfig['cuda_settings']
network_settings = driverconfig['network_settings']

rxshm_size = shm_settings.getint('rxshm_size')
txshm_size = shm_settings.getint('txshm_size')


cuda_port = network_settings.getint('CUDADriverPort')
usrp_port = network_settings.getint('USRPDriverPort')

samplingRate_rx = float(cuda_settings['FSampTX'])
samplingRate_tx = float(cuda_settings['FSampRX'])



rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
swings = [SWING0, SWING1]
sides = [SIDEA]

# list of all semaphores/shared memory paths for cleaning up
shm_list = []
sem_list = []

def connect_to_cuda_driver():
  serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

  max_connect_attempts = 5
  for i in range(max_connect_attempts):
      print('attempting connection to usrp_driver')
      try:
          serversock.connect(('localhost', cuda_port))
          break
      except:
          print('connecting to cuda driver failed on attempt ' + str(i + 1))
          time.sleep(5)
  return [serversock]


def connect_to_usrp_driver():
    max_connect_attempts = 5

    serversock = []
    for iUSRP, antennaNumber in enumerate(ANTENNA_UNDER_TEST):
       serversock.append(socket.socket(socket.AF_INET, socket.SOCK_STREAM))
       for i in range(max_connect_attempts):
           print('attempting connection to usrp_driver ( at localhost:{} )'.format(usrp_port + antennaNumber))
           try:
               serversock[iUSRP].connect(('localhost', usrp_port + antennaNumber))
               break;
           except:
               print('connecting to usrp driver failed on attempt ' + str(i + 1))
               time.sleep(3)

               if i == (max_connect_attempts - 1):
                   print('connecting to usrp driver failed, exiting')
                   sys.exit(1)
    return serversock

def sync_usrps(sock):
      logmsg("Start sync_usrps")
      cprint('testing usrp trigger with one period','red')

      cmd = usrp_sync_time_command(sock)
      cmd.transmit()
      ret = cmd.client_return()
      logmsg("End sync_usrps")

class swingParameter():
   def __init__(self, seq):
      self.seq = seq
      self.rx_rate = 5e6
      self.tx_rate = 5e6
      self.nPulses_per_period = 9
      self.nSamples_per_pulse = 4200
      self.sample_offsets = None
      self.swing = 0

   @property
   def tx_freq(self):
      self.seq.ctrlprm['tfreq']
   
   @tx_freq.setter
   def tx_freq(self, value):
       self.seq.ctrlprm['tfreq'] = value

   @property
   def rx_freq(self):
      self.seq.ctrlprm['rfreq']
   
   @rx_freq.setter
   def rx_freq(self, value):
       self.seq.ctrlprm['rfreq'] = value



def generate_parameter(integration_period = 2):
      seq = test_cuda_driver.create_testsequence()
      usrp_par = swingParameter(seq)
     

      # determine the length of integration periods for all channels in seconds
      PULSE_SEQUENCE_PADDING_TIME = 35e3 * 75 * 2 / 3e8 # without offset
      nPulses_in_sequence = 8 # seq.npulses


#      nSamples_per_pulse = np.uint64(int(cuda_settings['TXUpsampleRate']) *  ( np.floor(samplingRate_tx * seq.pulse_lens[0]/1e6 ) + 2 * np.floor(samplingRate_tx * seq.tr_to_pulse_delay/1e6 )))
      nSamples_per_pulse = np.uint64( ( np.floor(samplingRate_tx * seq.pulse_lens[0]/1e6 ) + 2 * np.floor(samplingRate_tx * seq.tr_to_pulse_delay/1e6 )))

      # to find out how much time is available in an integration period for pulse sequences, subtract out startup delay
      sampling_duration = integration_period - INTEGRATION_PERIOD_SYNC_TIME

      # calculate the pulse sequence period with padding
      time_sequence     = PULSE_SEQUENCE_PADDING_TIME + seq.pulse_offsets_vector[-1] + seq.pulse_lens[-1] /1e6
      nSamples_sequence = int(np.round(time_sequence * samplingRate_tx))
      usrp_par.seq.ctrlprm['number_of_samples'] = int(time_sequence*usrp_par.seq.ctrlprm['baseband_samplerate'])
      # calculate the number of pulse sequences that fit in the available time within an integration period
      nSequences_in_period = int(np.floor(sampling_duration / time_sequence))

      # then calculate sample indicies at which pulse sequences start within a pulse sequence
      pulse_sequence_offsets_samples = [int(offset* samplingRate_tx) for offset in seq.pulse_offsets_vector]
      pulse_sequence_offsets_vector = seq.pulse_offsets_vector

      # then, calculate sample indicies at which pulses start within an integration period
      nPulses_per_period = int(nPulses_in_sequence * nSequences_in_period )
      integration_period_pulse_sample_offsets = np.zeros(nPulses_per_period, dtype=np.uint64)
      for iSequence in range(nSequences_in_period):
          for iPulse in range(nPulses_in_sequence):
              integration_period_pulse_sample_offsets[iSequence * nPulses_in_sequence + iPulse] = iSequence * nSamples_sequence + pulse_sequence_offsets_samples[iPulse]

      # calculate the number of RF transmit and receive samples  
      usrp_par.seq.nbb_rx_samples_per_integration_period = nSamples_sequence * nSequences_in_period

      print("nSamples_rx:{}, nSamples_per_pulse:{}, integration_period_pulse_sample_offsets:".format(usrp_par.seq.nbb_rx_samples_per_integration_period , nSamples_per_pulse))
      print("nSequences_in_period:{}, nPulses_per_period:{}, ".format(nSequences_in_period, nPulses_per_period))
      print(integration_period_pulse_sample_offsets)


      swing = 0 


      usrp_par.rx_rate = samplingRate_rx
      usrp_par.tx_rate = samplingRate_tx
      usrp_par.nPulses_per_period = nPulses_per_period

      usrp_par.nSamples_per_pulse = nSamples_per_pulse
      usrp_par.sample_offsets = integration_period_pulse_sample_offsets
      usrp_par.swing = swing

      return usrp_par

def usrp_setup(sock, usrp_par):
      logmsg("Start usrp_setup (swing{})".format(usrp_par.swing))
      start_setup = time.time()
      cmd = usrp_setup_command(sock, usrp_par.tx_freq,  usrp_par.rx_freq,  usrp_par.rx_rate , usrp_par.tx_rate,  usrp_par.nPulses_per_period, usrp_par.seq.nbb_rx_samples_per_integration_period, usrp_par.nSamples_per_pulse, usrp_par.sample_offsets, usrp_par.swing)
      cmd.transmit()
      client_returns = cmd.client_return()

      for r in client_returns:
          assert(r == UHD_SETUP)
      time_needed_for_setup = time.time() - start_setup
      print("Time for:  setup: {}".format(time_needed_for_setup))
      logmsg("End usrp_setup (swing{})".format(usrp_par.swing))

def usrp_trigger(sock, usrp_par):
    logmsg("Start usrp_tigger (swing{})".format(usrp_par.swing))
    # grab current usrp time from one usrp_driver
    cmd = usrp_get_time_command(sock[0])
    cmd.transmit()
    usrp_time = cmd.recv_time(sock[0])
    cmd.client_return()

    trigger_time = usrp_time +  INTEGRATION_PERIOD_SYNC_TIME
    cmd = usrp_trigger_pulse_command(sock, trigger_time, usrp_par.seq.tr_to_pulse_delay / 1e9, usrp_par.swing)
    cmd.transmit()
    client_returns = cmd.client_return()
    for r in client_returns:
        assert(r == UHD_TRIGGER_PULSE)
    logmsg("End usrp_tigger (swing{})".format(usrp_par.swing))

def usrp_ready_data(sock, usrp_par):
    logmsg("Start usrp_ready (swing{})".format(usrp_par.swing))

    # request pulse data
    cmd = usrp_ready_data_command(sock, usrp_par.swing)
    cmd.transmit()
    logmsg("Start usrp_ready waiting (swing{})".format(usrp_par.swing))
    for iSock in sock:
       ret = cmd.recv_metadata(iSock)
       print("  recieved READY STATUS: status:{}, ant: {}, nSamples: {}, fault: {}".format(ret['status'], ret['antenna'], ret['nsamples'], ret['fault']))
    logmsg("End usrp_ready waiting (swing{})".format(usrp_par.swing))

    client_returns = cmd.client_return()
    for r in client_returns:
        assert(r == UHD_READY_DATA)
    logmsg("End usrp_ready (swing{})".format(usrp_par.swing))




def plot():
      # plot data
      import matplotlib.pyplot as plt
      print("Reading data from shm:")
      for ant in ANTENNA_UNDER_TEST:
         rx_shm = rx_shm_list[0][ant*2]
         rx_shm.seek(0)
         ar = np.frombuffer(rx_shm, dtype=np.int16, count=nSamples_rx*2)
         arp = np.float32(ar[0::2]) ** 2 + np.float32(ar[1::2]) ** 2
         print("  ant {}: rms: {:5.3f}   max: {:5.3f}".format(ant, np.sqrt(np.mean(arp) ), np.sqrt(np.max(arp)) ))
         if True:
             plt.plot(ar[::2])
             plt.plot(ar[1::2])
             plt.plot(np.sqrt(arp))
             plt.show()

      print('sampled phase')
      # pdb.set_trace() 

def cuda_add_channel(sock, parClass):
   logmsg("Start cuda_add_channel (swing{})".format(parClass.swing))
   cmd = cuda_add_channel_command(sock, parClass.seq, parClass.swing)
   cmd.transmit()
   cmd.client_return()
   logmsg("End cuda_add_channel (swing{})".format(parClass.swing))

def cuda_generate_pulse(sock, parClass):
   logmsg("Start cuda_generate (swing{})".format(parClass.swing))
   cmd = cuda_generate_pulse_command(sock, parClass.swing)
   cmd.transmit()
   cmd.client_return()
   logmsg("End cuda_generate (swing{})".format(parClass.swing))


def cuda_process(sock, parClass):
   logmsg("Start cuda_process (swing{})".format(parClass.swing))
   # process samples from shared memory
   cmd = cuda_process_command(sock, parClass.swing)
   cmd.transmit()
   cmd.client_return()
   logmsg("End cuda_process (swing{})".format(parClass.swing))

def cuda_get_data(sock, parClass, channel_number):
     logmsg("Start cuda_get_data (swing{})".format(parClass.swing))
     # copy processed samples
     cmd = cuda_get_data_command(sock, parClass.swing)
     cmd.transmit()

     main_samples = None
     back_samples = None
   
     for cudasock in sock:

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

            if antIdx < 16:
                main_samples[channel_number-1][antIdx] = samples[:]

            else:
                back_samples[channel_number-1][antIdx - nMainAntennas] = samples[:]


        transmit_dtype(cudasock, -1, np.int32) # send channel -1 to cuda driver to end transfer process
     cprint('finished collecting samples!', 'red')

     cmd.client_return()
     logmsg("End cuda_get_data (swing{})".format(parClass.swing))
     return [main_samples, back_samples]


def cuda_exit(sock):
    cmd = cuda_exit_command(sock)
    cmd.transmit()



def test_one_swing():
   cuda_sock = connect_to_cuda_driver()
   usrp_sock = connect_to_usrp_driver()
   par_swing0 = generate_parameter()
   
   usrp_setup(usrp_sock, par_swing0)
   cuda_add_channel(cuda_sock, par_swing0)
   cuda_generate_pulse(cuda_sock, par_swing0)
   usrp_trigger(usrp_sock, par_swing0)
   usrp_ready_data(usrp_sock, par_swing0)
   cuda_process(cuda_sock, par_swing0)
   channel_number = 1
   data = cuda_get_data(cuda_sock, par_swing0, channel_number)

   cuda_exit(cuda_sock)


def test_both_swings():
   cuda_sock = connect_to_cuda_driver()
   usrp_sock = connect_to_usrp_driver()
   par_swing0 = generate_parameter()
   par_swing1 = generate_parameter()
   par_swing1.swing = 1

   # swing A
   usrp_setup(usrp_sock, par_swing0)
   cuda_add_channel(cuda_sock, par_swing0)
   cuda_generate_pulse(cuda_sock, par_swing0)
   usrp_trigger(usrp_sock, par_swing0)
 
   # B
   cuda_add_channel(cuda_sock, par_swing1)
   cuda_generate_pulse(cuda_sock, par_swing1)

   # wait for A
   usrp_ready_data(usrp_sock, par_swing0)
   cuda_process(cuda_sock, par_swing0)
   
   usrp_trigger(usrp_sock, par_swing1)
   
   channel_number = 1
   data = cuda_get_data(cuda_sock, par_swing0, channel_number) # swin A finished
   cuda_add_channel(cuda_sock, par_swing0)
   cuda_generate_pulse(cuda_sock, par_swing0)

   # wait for B
   usrp_ready_data(usrp_sock, par_swing1)
   cuda_process(cuda_sock, par_swing1)
   

#   usrp_setup(usrp_sock, par_swing0)
   channel_number = 1
   data = cuda_get_data(cuda_sock, par_swing1, channel_number) # swing B finished

   cuda_exit(cuda_sock)

def test_nSequences(nSequences):
   
   if nSequences < 1:
      print("nSequences have to be >= 1. Setting it to 1")
      nSequences = 1

   # connect and setup
   cuda_sock = connect_to_cuda_driver()
   usrp_sock = connect_to_usrp_driver()
   par_swing0 = generate_parameter()
   par_swing1 = generate_parameter()
   par_swing1.swing = 1
   par_swing1.tx_freq = 15000
   par_swing1.rx_freq = 15000
   par_vec = [par_swing0, par_swing1]
   rx_data_list = []

   # prepare  first swing
   usrp_setup(usrp_sock, par_swing0)
   cuda_add_channel(cuda_sock, par_swing0)
   cuda_generate_pulse(cuda_sock, par_swing0)
   usrp_trigger(usrp_sock, par_swing0)
 
   channel_number = 1
   active_swing = 0 # swing finishing tx/rx, signal processing and tranismitting of final data
   prep_swing   = 1 # preparing swing (add cuda channel, generate pulse, usrp setup) and start transmitting

   for iSequence in range(nSequences-1):
      # prepare prep_swing in cuda
      cuda_add_channel(cuda_sock, par_vec[prep_swing])
      cuda_generate_pulse(cuda_sock, par_vec[prep_swing])
   
      # wait active_swing and start cuda processing
      usrp_ready_data(usrp_sock, par_vec[active_swing])
#      cuda_process(cuda_sock, par_vec[active_swing])
      
      # setup usrp for prep_swing and trigger usrp
      usrp_setup(usrp_sock, par_vec[prep_swing])
      usrp_trigger(usrp_sock, par_vec[prep_swing])

      cuda_process(cuda_sock, par_vec[active_swing])
      # get data of active_swing
      rx_data_list.append( cuda_get_data(cuda_sock, par_vec[active_swing], channel_number) )

      # switch prep and active swing
      active_swing = 1 - active_swing
      prep_swing = 1 - prep_swing

   usrp_ready_data(usrp_sock, par_vec[active_swing])
   cuda_process(cuda_sock, par_vec[active_swing])
   rx_data_list.append(cuda_get_data(cuda_sock, par_vec[active_swing], channel_number) )

#   cuda_exit(cuda_sock)
   os.system("../srr.py stop") 
   return rx_data_list



def plot_sequences(rx_data_list):
    import matplotlib.pyplot as plt
    # rx_data[iSequence|iSwing][mainArray|backArray][iChannel][iAntenne][iComplexSample]
    nSequences = len(rx_data_list)
    for iSequence, seqRxData in enumerate(rx_data_list):
    #  plt.figure()
      for i, antennaData in enumerate(seqRxData[0][0]):
         if i > 7:
            break
         plt.subplot(nSequences,8,  i+1+iSequence*8)
         if i == 0:
             plt.title("seq {} ant {}".format(iSequence, i))

         plt.plot(np.real(antennaData))
         plt.plot(np.imag(antennaData))
         power = np.abs(antennaData)**2
         print("sequence {}: ant {: >2}, rms {: >9.3f}   max  {: >9.3f}".format(iSequence, i, np.sqrt(np.sum(power)), np.sqrt(max(power))  ))
    plt.figure()
    data =rx_data_list[0][0][0][0] 
    plt.plot([i/3333.33 for i in range(len(data))], data)
    plt.title("sequence 0, ant 0")
    plt.show()


def tmp():
   cuda_sock = connect_to_cuda_driver()
   par_swing0 = generate_parameter()
   cuda_add_channel(cuda_sock, par_swing0)
   cuda_generate_pulse(cuda_sock, par_swing0)
   cuda_exit(cuda_sock)


rx_data_list = test_nSequences(3)
plot_sequences(rx_data_list)
