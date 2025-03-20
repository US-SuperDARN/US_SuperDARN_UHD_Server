#!/usr/bin/python3
# Interface between control program(s) and drivers (cuda and usrp)
#
# RadarChannelHandler class
# - one object for each control program
# - communicates with the control program and changes the state of the channel
#
# RadarHardwareManager class (RHM)
# - has a radar_main_control_loop() that executes clear freq search, adding new channels and triggering
#

import sys
import os
import numpy as np
import threading
import logging
import pdb
import socket
import time
import datetime
import configparser
import copy
import posix_ipc
import mmap
import pickle
import struct


sys.path.insert(0, '../python_include')

from phasing_utils import *
from socket_utils import *
from rosmsg import *
from drivermsg_library import *
from radar_config_constants import * 
from clear_frequency_search import read_restrict_file, record_clrfreq_raw_samples, calc_clear_freq_on_raw_samples
from profiling_tools import *
import logging_usrp
import utils


RMSG_PORT = 45000
MAX_CHANNELS = 4
USRP_BANDWIDTH_RESTRICTION = 5000 # in Hz. No channels allowed on both edges of the URSP bandwidth to avoid aliasing 
USRP_SOCK_TIMEOUT = 7 # sec

RMSG_SUCCESS = 0
RMSG_FAILURE = -1

RADAR_STATE_TIME = .0001
CHANNEL_STATE_TIMEOUT = 12000
# TODO: move this out to a config file
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.mcm/restrict.dat.inst'
nSwings = 2 

debug = False 

DEFAULT_USRP_MIXING_FREQ = 13000

# channel states (CS) for each channel
CS_INACTIVE      = 'CS_INACTIVE'
CS_READY         = 'CS_READY'
CS_TRIGGER       = 'CS_TRIGGER'
CS_PROCESSING    = 'CS_PROCESSING'
CS_SAMPLES_READY = 'CS_SAMPLES_READY'
CS_LAST_SWING    = 'CS_LAST_SWING'


class integrationTimeManager():
   """ Estimates the time the integration period has to be reduced to be able to setup urps copy samples etc"""
   def __init__(self, RHM):
      self.RHM = RHM
      self.last_start = None # of trigger next function

   def started_trigger_next(self):
      now = datetime.datetime.now()
      if self.last_start != None:
         nSeconds = (now - self.last_start).total_seconds()
         self.RHM.logger.info("Time with overhead for last integration period: {} s".format(nSeconds))
      self.last_start = now

   def get_usrp_delay_time(self): 
      int_time = self.RHM.commonChannelParameter['integration_period_duration']  
      if int_time <= 1:
         delay_time = INTEGRATION_PERIOD_SYNC_TIME_ONESEC
      else:
         delay_time = INTEGRATION_PERIOD_SYNC_TIME

      return delay_time



   def estimate_calc_time(self):
      int_time = self.RHM.commonChannelParameter['integration_period_duration']  
      # TODO optimize by tracking times of last periods
      if int_time == 3.5:
         overhead_time = 0.3
      elif int_time == 2.9:
         overhead_time = 0.5
      elif int_time == 3.2:
          overhead_time = 0.4
      elif int_time == 2.9:
         overhead_time = 0.05
      elif int_time == 1:
         overhead_time = 0.05 # TODO adjust and test
      else:
         overhead_time = 0.5
         error_str = "No overhead time defined for {} s, using 0.7  please add it...".format(int_time)
         self.RHM.logger.error(error_str)
#         raise ValueError(error_str)
      return overhead_time
    

 
   


class statusUpdater():
   " Class to a file every x minutes to allow checking usrp_status from outside"

   def __init__(self, RHM ):
      self.fileName = '../log/usrp_server_status.txt'
      self.RHM = RHM
      self.nSeconds_update_period = 5
      self.last_write = datetime.datetime.now()
      self.str_start = self.last_write.strftime("Start time: %Y-%m-%d %H:%M:%S\n")

   def create_status_information(self):
      status = self.str_start
      status += "USRPs: {} active, {} inactive\n".format(len(self.RHM.usrpManager.addressList_active), len(self.RHM.usrpManager.addressList_inactive))
      status += "Channels: {} active (of {})\n".format(self.RHM.nRegisteredChannels, len(self.RHM.channels))
      status += "Sequences per period: {}\n".format(self.RHM.nSequences_per_period)
      return status
      
      

   def update_advanced(self):
      """ Writes some information in the file"""

      nSeconds_since_last_write = (datetime.datetime.now() - self.last_write).total_seconds()
      if self.nSeconds_update_period < nSeconds_since_last_write:
         self.last_write = datetime.datetime.now()
#         if not os.path.isfile(self.fileName):
         with open(self.fileName, "w") as f:
            f.write(self.create_status_information())    


   def update(self):
      """ just updates the empty file """
      nSeconds_since_last_write = (datetime.datetime.now() - self.last_write).total_seconds()
      if self.nSeconds_update_period < nSeconds_since_last_write:
         self.last_write = datetime.datetime.now()
#         if not os.path.isfile(self.fileName):
         with open(self.fileName, "w") as f:
            f.write("")            # create empty file
#    disadvantage this time is not the default time shown by ls
#         else:
#            print("updating time ")
#            os.utime(self.fileName, None) # update the time
       

class usrpSockManager():
   def __init__(self, RHM):
      self.addressList_active     = [] # tuple of IP and port
      self.antennaList_active     = []
      self.hostnameList_active    = []
      self.addressList_inactive   = []
      self.antennaList_inactive   = []
      self.hostnameList_inactive  = []

      self.socks = []
      usrp_driver_base_port = int(RHM.ini_network_settings['USRPDriverPort'])
      self.RHM = RHM
      self.logger = logging.getLogger("usrpManager")      
      
      self.nUSRPs = len(RHM.ini_usrp_configs) # TODO should this be all USRPs or only active?
      self.fault_status = np.ones(self.nUSRPs)
      self.errors_in_a_row = 0
      self.error_limit = 15
      self.nSeconds_retry_reconnect = 60
      self.last_reconnection = datetime.datetime.now()
      
      # open each
      self.hostnameList_active = [] 
      for usrpConfig in RHM.ini_usrp_configs:
         try:
            if usrpConfig['usrp_hostname'] in self.hostnameList_active:
               self.logger.debug("Already connected to USRP {}".format(usrpConfig['usrp_hostname']))
               idx_usrp = self.hostnameList_active.index(usrpConfig['usrp_hostname'])
               self.antennaList_active[idx_usrp].append(usrpConfig['array_idx'])
               
            elif usrpConfig['usrp_hostname'] in self.hostnameList_inactive:
               self.logger.debug("Already failed to connected to USRP {}".format(usrpConfig['usrp_hostname']))
               idx_usrp = self.hostnameList_inactive.index(usrpConfig['usrp_hostname'])
               self.antennaList_inactive[idx_usrp].append(usrpConfig['array_idx'])
               
            else:
               port = int(usrpConfig['usrp_hostname'].split(".")[2]) + usrp_driver_base_port
               usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               connectPar = (usrpConfig['driver_hostname'], port)
               usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
               usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
               usrpsock.connect(connectPar)
               if USRP_SOCK_TIMEOUT != None:
                  usrpsock.settimeout(USRP_SOCK_TIMEOUT)
               self.socks.append(usrpsock)
               self.addressList_active.append(connectPar)
               self.antennaList_active.append([usrpConfig['array_idx']])
               self.hostnameList_active.append(usrpConfig['usrp_hostname'])
               self.logger.debug('connected to usrp driver on port {}'.format(port))

         except ConnectionRefusedError:
            self.logger.error('USRP server connection failed: {}:{}'.format(usrpConfig['driver_hostname'], port))

            if usrpConfig['usrp_hostname'] in self.hostnameList_inactive: 
               idx_usrp = self.hostnameList_inactive.index(usrpConfig['usrp_hostname'])
               self.antennaList_inactive[idx_usrp].append(usrpConfig['array_idx'])
            else:   
               self.addressList_inactive.append((usrpConfig['driver_hostname'], port ))
               self.antennaList_inactive.append([usrpConfig['array_idx']])
               self.hostnameList_inactive.append(usrpConfig['usrp_hostname'])

      if len(self.socks) ==0:
         self.logger.error("No connection to USRPs. Exit usrp_server.")
         RHM.exit() 


   def remove_sock(self, sock_to_remove):
       iSock = self.socks.index(sock_to_remove)
       self.logger.error("Removing usrp {} ({}:{}). ".format(self.hostnameList_active[iSock], self.addressList_active[iSock][0], self.addressList_active[iSock][1])) 
       self.addressList_inactive.append(self.addressList_active[iSock] )
       del self.addressList_active[iSock]

       lost_antennas = self.antennaList_active[iSock]
       self.antennaList_inactive.append(lost_antennas)
       for iSwing in range(nSwings):
          self.fill_shm_with_zeros(lost_antennas, iSwing, ['rx', 'tx'])
       del self.antennaList_active[iSock]

       self.hostnameList_inactive.append(self.hostnameList_active[iSock])
       del self.hostnameList_active[iSock]

       del self.socks[iSock]

       self.RHM.clearFreqRawDataManager.set_usrp_driver_connections(self.socks) 

   def get_all_main_antenna_socks(self):
       main_ant_sock_list = []
       for sock, ant in zip(self.socks, self.antennaList_active):
           if ant not in [16, 17, 18, 19]:
               main_ant_sock_list.append(sock)
       return main_ant_sock_list



   def eval_client_return(self, cmd, fcn=None):
      if fcn is None: # default receive function
         client_return = cmd.client_return()
      else:
         client_return = fcn()

      if CONNECTION_ERROR in client_return:
         offset = 0
         for iSock, singleReturn in enumerate(client_return):
            if singleReturn == CONNECTION_ERROR:
               self.logger.error("Connection lost to usrp {}:{}. Removing it from sock list. ".format(self.addressList_active[iSock-offset][0], self.addressList_active[iSock-offset][1])) 
               self.remove_sock(self.socks[iSock-offset])
               offset += 1 

      if len(self.socks) == 0:
         self.logger.error("No working USRPs left. Shutting down usrs_server...")
         self.RHM.exit()


      return client_return

#   def cleanup_usrp(self):
#      for usrpConfig in RHM.ini_usrp_configs:
#         try:
#            if usrpConfig['usrp_hostname'] in self.hostnameList_active:

   def fill_shm_with_zeros(self, antenna_list, swing, direction_list):
      side = 0
      # direction_list = ['rx', 'tx']
      nInts_shm = int(self.RHM.ini_shm_settings['rxshm_size']) / 2 # two bytes per int
      nZeros_per_block = 10000   #write zeros in blocks
      zeros_block = np.zeros(nZeros_per_block, dtype=np.int16).tobytes()
      nFullBlocks = int(nInts_shm / nZeros_per_block)
      nInts_rem   = nInts_shm % nZeros_per_block
      for antenna in antenna_list:
        for direction in direction_list:
           try:
              name = 'shm_{}_ant_{}_side_{}_swing_{}'.format(direction, int(antenna), int(side), int(swing))
              self.logger.debug("Filling SHM with zeros: {}".format(name))
              memory = posix_ipc.SharedMemory(name)
              mapfile = mmap.mmap(memory.fd, memory.size)
              mapfile.seek(0)
              for iBlock in  range(nFullBlocks): # TODO speed up by wrining more that one byte at a time?
                 mapfile.write(zeros_block)
              mapfile.write(zeros_block[0:int(2*nInts_rem)])
              memory.close_fd()
           except:
              self.logger.debug("Failed filling SHM with zeros: {}".format(name))



   def watchdog(self, all_usrps_report_failure):
      if all_usrps_report_failure:
         self.errors_in_a_row += 1
         self.logger.info("USRP watchdog: {} error in a row.".format(self.errors_in_a_row))
         if self.errors_in_a_row >= self.error_limit:
            self.logger.error("All USRPs reported error for GET_DATA {} times in a row. Shutting down usrp_server".format(self.errors_in_a_row))
            self.RHM.exit()
      else:
         if self.errors_in_a_row:
            self.logger.info("USRP watchdog: Reset errors_in_a_row to 0.")
            self.errors_in_a_row = 0

   def restore_lost_connections(self):
       
      nSeconds_since_last_try = (datetime.datetime.now() - self.last_reconnection).total_seconds()
      if self.nSeconds_retry_reconnect > nSeconds_since_last_try:
         return

  

      self.logger.info("Try to reconnect to USRPs")
      self.last_reconnection = datetime.datetime.now()

      tmp_address_list = self.addressList_inactive
      tmp_antenna_list = self.antennaList_inactive
      tmp_hostname_list = self.hostnameList_inactive
      self.addressList_inactive = [] 
      self.antennaList_inactive = [] 
      self.hostnameList_inactive = [] 
      do_resync = False
      for iUSRP,usrp in enumerate(tmp_address_list):

         if usrp[0] in self.hostnameList_active:
            self.logger.error(" Already connected to USRP {}, something went wrong!".format(usrp[0]))
            #idx_usrp = self.hostnameList_active.index(usrp[0])
            #self.antennaList_active[idx_usrp].append(usrpConfig['array_idx'])
            
         try: 
            usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
            usrpsock.connect(usrp)

            self.socks.append(usrpsock)
            self.addressList_active.append(usrp)
            self.antennaList_active.append(tmp_antenna_list[iUSRP])
            self.hostnameList_active.append(tmp_hostname_list[iUSRP])
            self.logger.info('reconnection to usrp  {} successful'.format(tmp_hostname_list[iUSRP]))
            do_resync = True

         except ConnectionRefusedError:
            self.logger.warning('reconnection to usrp {} failed'.format(tmp_hostname_list[iUSRP]))
            self.addressList_inactive.append(usrp)
            self.antennaList_inactive.append(tmp_antenna_list[iUSRP]) 
            self.hostnameList_inactive.append(tmp_hostname_list[iUSRP])


      # sync to other usrps
      if do_resync:
         self.RHM._resync_usrps()
         self.RHM.rxfe_init() 


class usrpMixingFreqManager():
    """ Manages usrp mixing frequency based on channels. Ensures that only one channel
        at a time can call add_new_freq_band().  """
  
    def __init__(self, cFreq, bandWidth):
       self.current_mixing_freq = cFreq       # in kHz (to be compatible with control program)
       self.usrp_bandwidth      = bandWidth - USRP_BANDWIDTH_RESTRICTION*2/1000   # in kHz (to be compatible with control program)
       self.semaphore = posix_ipc.Semaphore('usrp_mixing_freq', posix_ipc.O_CREAT)
       self.semaphore.release()
       self.channelRangeList    = []
       self.channelList         = []

    def add_new_freq_band(self, channel):
       """ Checks if new channel is covered with current mixing frequency and bandwidth. 
           Retruns True/False if channel can/cannot be added. If changing the mixing frequency
           allows to add the new channel, the new mixing frequency will be output argument.
       """

       RHM = channel.parent_RadarHardwareManager
       newLower, newUpper = self.get_range_of_channel(channel)
       if newLower <  RHM.hardwareLimit_freqRange[0] or newUpper > RHM.hardwareLimit_freqRange[1]:
          channel.logger.error("Channel bandwidth ({} MHz- {} MHz) is not covered by radar hardware limits ({} MHz - {} MHz)".format(newLower/1000, newUpper/1000, RHM.hardwareLimit_freqRange[0]/1000, RHM.hardwareLimit_freqRange[1]/1000))
          return False

       channel.logger.debug("ch {}: waiting for semaphore of usrpMixingFreqManager".format(channel.cnum))
       self.semaphore.acquire()
       channel.logger.debug("ch {}: acquired semaphore of usrpMixingFreqManager".format(channel.cnum))
   
       if newLower > (self.current_mixing_freq - self.usrp_bandwidth/2) and newUpper < (self.current_mixing_freq + self.usrp_bandwidth/2):
          channel.logger.debug("channel range is within USRP bandwidth")
          self.channelRangeList.append([newLower, newUpper])
          self.channelList.append(channel)
          result = True
       else:
          #determine range of all channels
          allCh_lower = newLower
          allCh_upper = newUpper
   
          for otherChRange in self.channelRangeList:
             allCh_lower = min(allCh_lower, otherChRange[0])
             allCh_upper = max(allCh_upper, otherChRange[1])


          if (allCh_upper - allCh_lower) > self.usrp_bandwidth:
             channel.logger.error("new channel can not be added. USPR bandwidth too small")
             result =  False
          else:
             newMixingFreq = (allCh_upper - allCh_lower)/2 + allCh_lower
             channel.logger.info("calculated new usrp mixing frequency: {} kHz (old was {} kHz)".format(newMixingFreq, self.current_mixing_freq))
             # adjust mixing freq that everything is in overall bandwidth
             newMixingFreq = max(newMixingFreq, RHM.hardwareLimit_freqRange[0]+self.usrp_bandwidth/2)
             newMixingFreq = min(newMixingFreq, RHM.hardwareLimit_freqRange[1]-self.usrp_bandwidth/2)
             self.current_mixing_freq = newMixingFreq
             result = newMixingFreq
          
     
       self.semaphore.release()
       channel.logger.debug("ch {}: released semaphore of usrpMixingFreqManager".format(channel.cnum))
       return result

    def get_range_of_channel(self, channel):
       if channel.scanManager.fixFreq in [ None, -1, 0]: 
          rangeList = channel.scanManager.clear_freq_range_list
          lower = rangeList[0][0] 
          upper = rangeList[0][1]
          for periodRange in rangeList[1:]:
             lower = min(lower, periodRange[0])
             upper = max(lower, periodRange[1])
       else:
          lower = channel.scanManager.fixFreq
          upper = channel.scanManager.fixFreq
       return lower, upper

class ClearFrequencyService():
    # TODO: Look into loading Constants by .ini or .env
    # from dotenv import load_dotenv
    # load_dotenv(".env")
    

    # Program Flags
    CLEAN_ON_INACTIVE   = False           # Cleans all semaphores and shared memory objects when there are no Active Clients
    soft_kill = False
    
    # Static Constants
    CHAR_SIZE = 1
    INT_SIZE = 4
    DOUBLE_SIZE = 8
    
    # Shared Memory Object and Semaphores Constants
    SAMPLES_NUM  = 10000
    ANTENNA_NUM = 16
    RESTRICT_NUM = 20
    META_ELEM    = 3                                    # 3 = 4 - 1 (fcenter has unique obj)
    CLR_BAND_MAX = 6
    
    
    SAMPLES_ELEM_NUM    = ANTENNA_NUM * SAMPLES_NUM * 2
    CLR_RANGE_ELEM_NUM  = 2
    RESTRICT_ELEM_NUM   = RESTRICT_NUM * 2
    META_ELEM_NUM       = META_ELEM + ANTENNA_NUM
    CLR_BANDS_ELEM_NUM  = 1 * 3                         # 3     = start & stop freqs and noise
    SITE_ID_ELEM_NUM    = 1 * 3                         # 1 * 3 = one instance of a 3 letter identifier
    
    SAMPLES_SHM_SIZE        = (ANTENNA_NUM * SAMPLES_NUM * 2 * INT_SIZE) 
    CLR_RANGE_SHM_SIZE      = (2 * INT_SIZE)
    FCENTER_SHM_SIZE        = (1 * INT_SIZE)
    BEAM_NUM_SHM_SIZE       = (1 * INT_SIZE)
    SAMPLE_SEP_SHM_SIZE     = (1 * INT_SIZE)
    RESTRICT_SHM_SIZE       = (RESTRICT_NUM * 2 * INT_SIZE)          # 2 = start and end freqs
    META_DATA_SHM_SIZE      = ((META_ELEM + ANTENNA_NUM) * DOUBLE_SIZE)
    ANTENNA_SHM_SIZE        = (1 * INT_SIZE)
    CLR_BANDS_SHM_SIZE      = (1 * INT_SIZE * 3)     # TODO: Round to convert freqs to int again 
    SITE_ID_SHM_SIZE        = (3 * CHAR_SIZE)
    ACTIVE_CLIENTS_SHM_SIZE = (1 * INT_SIZE)

    
    RETRY_ATTEMPTS = 3
    RETRY_DELAY = 2  # seconds
    
    # Shared Memory Object and Semaphores Names
    SAMPLES_SHM_NAME =          "/samples"          # For Data Transmission
    CLR_RANGE_SHM_NAME =        "/clear_freq_range"
    FCENTER_SHM_NAME =          "/fcenter"
    BEAM_NUM_SHM_NAME =         "/beam_num"
    SAMPLE_SEP_SHM_NAME =       "/sample_sep"
    RESTRICT_SHM_NAME =         "/restricted_freq"
    META_DATA_SHM_NAME =        "/meta_data"
    ANTENNA_SHM_NAME =          "/antenna_num"
    CLRFREQ_SHM_NAME =          "/clear_freq"
    ACTIVE_CLIENTS_SHM_NAME =   "/active_clients"   # For Debugging
    SITE_ID_SHM_NAME =          "/site_id"

    
    SAMPLE_PARAM_NUM =      4
    RESTRICT_PARAM_NUM =    2
    PARAM_NUM =             9
    
    SEM_F_CLIENT =  "/sf_client"               # For reserving client and server roles during data transfer
    SEM_F_SERVER =  "/sf_server"               # And for signalling specific data transfers 
    SEM_F_SAMPLES = "/sf_samples"
    SEM_F_INIT =    "/sf_init"           
    SEM_F_CLRFREQ = "/sf_clrfreq"              
    SEM_L_SAMPLES = "/sl_samples"              # For Data locking b/w write/reads
    SEM_L_INIT =    "/sl_init"                 # init = initialization
    SEM_L_CLRFREQ = "/sl_clrfreq"
    
    SEM_NUM =       8
    SL_NUM =        3
    
    # Initialization Check
    isRestricted = False
    
    semaphores = []
    shm_objects = []
    cur_antenna_num = ANTENNA_NUM
    
    log = logging.getLogger('clearFrequency')
    
    def __init__(self, sid = 'lab'):
        # Process Site ID during Sample Send 
        self.sid = sid
        
        try:
            # Shared Memory Object and Semaphores
            self.sf_client  = self.create_semaphore(self.SEM_F_CLIENT)
            self.sf_server  = self.create_semaphore(self.SEM_F_SERVER)
            self.sf_samples = self.create_semaphore(self.SEM_F_SAMPLES)
            self.sf_init    = self.create_semaphore(self.SEM_F_INIT)
            self.sf_clrfreq = self.create_semaphore(self.SEM_F_CLRFREQ)
            self.sl_samples = self.create_semaphore(self.SEM_L_SAMPLES)
            self.sl_init    = self.create_semaphore(self.SEM_L_INIT)
            self.sl_clrfreq = self.create_semaphore(self.SEM_L_CLRFREQ)
            self.semaphores = [
                self.sf_client,
                self.sf_server,
                self.sf_samples,
                self.sf_init,
                self.sf_clrfreq,
                self.sl_samples,
                self.sl_init,
                self.sl_clrfreq,
            ]
            self.shm_objects = [
                self.create_shm_obj(self.SAMPLES_SHM_NAME ,         self.SAMPLES_SHM_SIZE       , self.SAMPLES_ELEM_NUM), 
                self.create_shm_obj(self.CLR_RANGE_SHM_NAME,        self.CLR_RANGE_SHM_SIZE     , self.CLR_RANGE_ELEM_NUM), 
                self.create_shm_obj(self.FCENTER_SHM_NAME,          self.FCENTER_SHM_SIZE       , ),
                self.create_shm_obj(self.BEAM_NUM_SHM_NAME,         self.BEAM_NUM_SHM_SIZE      , ), 
                self.create_shm_obj(self.SAMPLE_SEP_SHM_NAME,       self.SAMPLE_SEP_SHM_SIZE    , ),
                self.create_shm_obj(self.RESTRICT_SHM_NAME,         self.RESTRICT_SHM_SIZE      , self.RESTRICT_ELEM_NUM), 
                self.create_shm_obj(self.META_DATA_SHM_NAME,        self.META_DATA_SHM_SIZE     , self.META_ELEM_NUM),
                self.create_shm_obj(self.ANTENNA_SHM_NAME,          self.ANTENNA_SHM_SIZE       , ),
                self.create_shm_obj(self.CLRFREQ_SHM_NAME,          self.CLR_BANDS_SHM_SIZE     , self.CLR_BANDS_ELEM_NUM), 
                self.create_shm_obj(self.SITE_ID_SHM_NAME,          self.SITE_ID_SHM_SIZE       , self.SITE_ID_ELEM_NUM),
                self.create_shm_obj(self.ACTIVE_CLIENTS_SHM_NAME,   self.ACTIVE_CLIENTS_SHM_SIZE, )
            ]

            for obj in self.shm_objects:
                obj['shm_fd'] = self.initialize_shared_memory(obj['name'])
                            
            self.active_clients_fd = None 
            self.initialize_active_clients_counter()
            print("[clearFrequencyService] Done Initializing...\n\n")
            self.log.debug("clearFrequencyService initialized")

        except ValueError:
            print("[ClearFrequencyService] Initialization Failed. Cleaning up SHM Objects and Semaphores...")
            self.soft_kill = True
            self.cleanup_shm()
        except KeyboardInterrupt:
            print("[CFS] Keyboard Interupt triggered during Initialization... Canceling and cleaning up...")
            self.soft_kill = True
            self.cleanup_shm()
            
        
    def create_shm_obj(self, name, size, elem_num=1):
        """ Returns a dictionary containing pre-filled fields for shared memory (SHM) object data.
            
        Args:
            name (string): name of the shared memory object
            size (integer): size of the file/SHM object

        Returns:
            dictionary: Contains commonly referenced info of a shared memory object
                Contains:
                    - 'name' 
                    - 'shm_ptr' or pointer 
                    - 'shm_fd' or file descriptor 
                    - 'size' of the file/SHM object
                    - 'elem_num' or number of elements the object contains
        """
        return {
            'name': name,
            'shm_ptr': None,
            'shm_fd': -1,
            'size': size,
            'elem_num': elem_num
        }
        
    def create_semaphore(self, name):
        return {
            'name': name,
            'sem':  self.initialize_semaphores(name)
        }

    def initialize_shared_memory(self, shm_name):
        """ Initialize Shared Memory Object for data transmission between Server 
            and Clients. Attempts to check for already initialized object (from 
            server).

        Returns:
            Integer: On success, returns file descriptor of shared memory object.
        """
        attempts = 0
        while attempts < self.RETRY_ATTEMPTS:
            try:
                print(f"[clearFrequencyService] Attempting to initialize {shm_name} Shared Memory Object (Attempt {attempts + 1}/{self.RETRY_ATTEMPTS})...")
                shm_fd = os.open(f"/dev/shm{shm_name}", os.O_RDWR)
                print(f"[clearFrequencyService] Created {shm_name} Shared Memory Object...")
                return shm_fd
            except FileNotFoundError:
                print("[clearFrequencyService] Shared Memory Object not found. Retrying...")
                attempts += 1
                time.sleep(self.RETRY_DELAY)
        print(f"[clearFrequencyService] Failed to initialize {shm_name} Shared Memory Object after multiple attempts. Exiting.")
        exit(1)

    def initialize_semaphores(self, name):
        """ Initializes Synchronization Semaphores. Attempts to check for already 
            initialized object (from server).

        Returns:
            void: On success, returns tuple of semaphores (sem_server, sem_client).
        """
        attempts = 0
        while attempts < self.RETRY_ATTEMPTS:
            try:
                print(f"[clearFrequencyService] Attempting to initialize Semaphore {name} (Attempt {attempts + 1}/{self.RETRY_ATTEMPTS})...")
                semaphore = posix_ipc.Semaphore(name)
                print(f"[clearFrequencyService] Semaphore {name} ready...")
                return semaphore
            except posix_ipc.ExistentialError:
                print("[clearFrequencyService] Semaphore not found. Retrying...")
            attempts += 1
            time.sleep(self.RETRY_DELAY)
        print(f"[clearFrequencyService] Failed to initialize Semaphore {name} after multiple attempts. Exiting.")
        exit(1)

    def initialize_active_clients_counter(self):
        attempts = 0
        while attempts < self.RETRY_ATTEMPTS:
            try:
                # Init counter
                print(f"[clearFrequencyService] Attempting to initialize Active Clients Counter (Attempt {attempts + 1}/{self.RETRY_ATTEMPTS})...")
                self.active_clients_fd = os.open(f"/dev/shm", os.O_RDWR | os.O_TMPFILE, 0o666)
                os.ftruncate(self.active_clients_fd, struct.calcsize('i'))  # Ensure the size of the shared memory object is large enough for an integer
                # If abnormal num of clients, set to 0
                with mmap.mmap(self.active_clients_fd, struct.calcsize('i'), mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE) as m:
                    m.seek(0)
                    current_value = struct.unpack('i', m.read(struct.calcsize('i')))[0]
                    if current_value < 0:  # Arbitrary threshold to detect abnormal num of clients
                        m.seek(0)
                        m.write(struct.pack('i', 0))
                    m.seek(0)
                    current_value = struct.unpack('i', m.read(struct.calcsize('i')))[0]
                print("[clearFrequencyService] Created Active Clients Counter... @ ", current_value)
                return 
            except FileNotFoundError:
                print("[clearFrequencyService] Active Clients Counter not found. Retrying...")
            except PermissionError:
                print("[clearFrequencyService] Permission error while accessing Active Clients Counter. Retrying...")
            except OSError as e:
                print(f"[clearFrequencyService] OS error while accessing Active Clients Counter: {e}. Retrying...")
            attempts += 1
            time.sleep(self.RETRY_DELAY)
        print("[clearFrequencyService] Failed to initialize Active Clients Counter after multiple attempts. Exiting.")
        exit(1)

    def increment_active_clients(self):
        with mmap.mmap(self.active_clients_fd, struct.calcsize('i'), mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE) as m:
            m.seek(0)
            active_clients = struct.unpack('i', m.read(struct.calcsize('i')))[0]
            active_clients += 1
            m.seek(0)
            m.write(struct.pack('i', active_clients))
            print(f"[clearFrequencyService] Incremented Active Clients Counter: {active_clients}")
            return active_clients

    def decrement_active_clients(self):
        with mmap.mmap(self.active_clients_fd, struct.calcsize('i'), mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE) as m:
            m.seek(0)
            active_clients = struct.unpack('i', m.read(struct.calcsize('i')))[0]
            active_clients -= 1
            m.seek(0)
            m.write(struct.pack('i', active_clients))
            print(f"[clearFrequencyService] Decremented Active Clients Counter: {active_clients}")
            return active_clients
      
              
    def detect_dtype(self, var):
        """Recursively detects whether the variable is an int (i) or float (d). 
        Note that float is considered 'd' so that it can be used for struct.pack()

        Args:
            var (any): Variable to be detected

        Raises:
            ValueError: If the variable doesn't contain either ints or floats.

        Returns:
            _type_: Either 'd' or 'i' for float or int respectively.
        """
        # Continuation
        if isinstance(var, (list, set)):
            return self.detect_dtype(var[0]) 
        
        # Break Condition and Break
        elif isinstance(var, float):
            return 'd'
        elif isinstance(var, int):
            return 'i'
        else:
            raise ValueError(f"var ({var}) is contains neither float nor integers.")
                   
    def find_list_of_lists(self, var):
        """Recursively finds the point in the variable where the following 
        is true:
        (list_of_list -> list -> elem)
        This point is hereby called list_of_lists (LoL) for simplicity and is used 
        to flatten these arrays where applicable. 
        
        Note this function only works for variables that contain either ints or floats!

        Args:
            var (any): Variable to be parsed for LoL.

        Raises:
            ValueError: If the variable doesn't contain either ints or floats.

        Returns:
            _type_: Can be a list (the list_of_lists), 'd' or 'i' if var is a singular var.
        """
        # Continuation
        if isinstance(var, (list, set)):
            result = self.find_list_of_lists(var[0])
            
            # Break Conditions (return list_of_list->list->elem)
            if result  == 'elem':
                return 'list'
            elif result == 'list':
                return var
        
        # Break
        elif isinstance(var, (int, float)):
            return 'elem'
        else:
            raise ValueError(f"var ({var}) is contains neither float nor integers.")
    
    def write_data(self, obj, array_data, atype=''):
        """Writes data from array_data onto the object's shared memory pointer. 
        "Sends data from array_data across the obj's channel"

        Args:
            obj (dict): Object Dictionary contain Shared Memory data for the object. 
            array_data (list): List of data points 
            complex (bool, optional): Flag to write and unpack array_data 
                from its complex notation. Defaults to False.

        Raises:
            ValueError: If the variable doesn't contain either ints or floats.
        """
        try:    
            flattened_data = []
            if atype == 'complex':
                # If complex, flatten and separate real and imaginary parts
                for antenna_sample in array_data:  
                    # print("[Frequency Client] sample_arr len: ", len(antenna_sample))
                    
                    # TODO: Time trial w/ zip(int(sample.real), int(sample.imag))
                    for sample in antenna_sample:
                        flattened_data.append(int(sample.real))
                        flattened_data.append(int(sample.imag))
                        
                        # Debug: Display samples
                        # print("[Frequency Client] Flattening samples: ", sample)
                        # print("[Frequency Client]                   : ", flattened_data[-2])
                        # print("[Frequency Client]                   : ", flattened_data[-1])
            elif atype == 'meta':
                flattened_data += array_data[0]
                # for ant in array_data[0]:
                    # flattened_data.append(ant.item())
                for i in range (1, len(array_data)):
                    flattened_data.append(array_data[i])
            elif atype == "sid":
                for letter in array_data:
                    flattened_data.append(bytes(letter, 'ascii'))
            else:
                # Otherwise, just flatten                
                list_of_lists = self.find_list_of_lists(array_data)
                
                # Element/1D List Found
                if type(list_of_lists) is str:
                    flattened_data = array_data
                # Greater-than-1D list Found
                elif type(list_of_lists) is list: 
                    print("2D list detected! Flattening...")
                    for row in list_of_lists:
                        flattened_data += row
                # Fail: Unexpected value found
                else: 
                    raise ValueError(f"An unexpected value occured: {list_of_lists}")
            
            
            dtype = 'i'
            # Determine dtype for Packing
            if atype == 'meta':
                dtype = 'd'
            elif atype == "sid":
                dtype = b'c'
            else: 
                dtype = self.detect_dtype(flattened_data)
            print(f"dtype: {dtype}, elem_num: {obj['elem_num']}, ")
                
            print(f"flattened array type: {type(flattened_data)}")
                
            # Pack and write data
            if type(flattened_data) is list or type(flattened_data) is str:  
                print("[Frequency Client] new_data len of: ", len(flattened_data))
                if atype == 'complex':
                    print("[Frequency Client] Writing data:\n", flattened_data[:1], "...")
                    
                else:
                    print("[Frequency Client] Writing data:\n", flattened_data)                
                    
                obj['shm_ptr'].seek(0)
                if atype == 'sid':
                    print(f"ascii bytes: {flattened_data}")
                    print(f"dtype argument: {dtype * obj['elem_num']}")
                #     obj['shm_ptr'].write(struct.pack(dtype * obj['elem_num'], bytes(flattened_data, 'ascii'))) 
                # else: 
                obj['shm_ptr'].write(struct.pack(dtype * obj['elem_num'], *flattened_data)) 
            else:
                print("[Frequency Client] new_data len of: ", 1)
                print("[Frequency Client] Writing data:\n", flattened_data)
                
                obj['shm_ptr'].seek(0)
                obj['shm_ptr'].write(struct.pack(dtype * 1, flattened_data))
        
        
        except AttributeError:
            print("[Frequency Client] ERROR: Element Size is incorrect. send()'s parameters were likely not assigned properly. Please verify...")

    def read_m_data(self, obj):
        """Reads in data from the shared memory file descriptor.

        Args:
            obj (dict): Data object containing file descriptor and shared memory size.
            data_size (int): Shared Memory Size.
            data_sub_size (int, optional): Shared Memory sub element size; used for 2D arrays. Defaults to 1.

        Returns:
            list: Contains 1D list of Shared Memory data.
        """
        obj['shm_ptr'].seek(0)
        read_data = struct.unpack('i' * obj['elem_num'], obj['shm_ptr'].read(obj['size']))
        
        # Debug: Verify format of data object's raw data
        # print("[clearFrequencyService] Data read from Shm: ", read_data[:5], "...")  # Print first 10 integers for brevity
        
        return read_data
    
    
    def repack_data(self, read_data, clr_freq = False, data_size = 1, data_sub_size = 1):
        """Repacks read data from SHM into its specified format. Currently repacks
        the following:
        - Clear Frequency

        Args:
            read_data (list): 1D list of Shared Memory data.
            data_size (int): Shared Memory Size.
            data_sub_size (int, optional): Shared Memory sub element size; used for 2D arrays. Defaults to 1.
            clr_freq (bool, optional): Interpret as Clear Frequency Flag (returns 
                centerFreq, Noise). Defaults to False.
        """
        packed_data = []
        if clr_freq:
            noise_data = []
            for start_freq, noise, end_freq in zip(read_data[::3], read_data[1::3], read_data[2::3]):
                # Return Center Freq and Noise
                packed_data.append(((start_freq + end_freq) / 2) / 1000)
                noise_data.append(noise)
            return packed_data, noise_data            
                
    def request_clr_freq(self, raw_samples, clr_range=None, fcenter=None, beam_num=None, sample_sep=None, restrict_data=None, meta_data=None, ):
        """ Waits for client requests, then processes server data, writes client 
            data, and requests server to process new data. When process is 
            terminated, the try/finally block cleans up.
        """
        input_data = [
            raw_samples, 
            clr_range, 
            fcenter, 
            beam_num,
            sample_sep, 
            restrict_data, 
            meta_data,
            self.sid,
        ]
        
        # Special: Halt all future ClearFreqService
        if self.soft_kill is True:
            return
                
        # Get in Queue
        active_clients = self.increment_active_clients()
        print(f"[clearFrequencyService] Active clients count: {active_clients}\n")
        
        try:
            ## Check for Premapped antenna num
            # Map shared memory object pointer for antenna num
            print(f"Mapping {self.shm_objects[7]['name']}")
            self.log.debug("[clearFrequencyService] Verifying Antenna and Sample size")
            self.shm_objects[7]['shm_ptr'] = mmap.mmap(self.shm_objects[7]['shm_fd'], self.shm_objects[7]['size'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            
            # Check if Antenna Num changed, update corresponding values before they're mapped
            temp_ant_num = self.read_m_data(self.shm_objects[7])[0]
            print("Antenna_num: ", temp_ant_num)
            if temp_ant_num != 0 and temp_ant_num != self.ANTENNA_NUM:
                print("Antenna_num has been changed, updating SHM values before further SHM mapping...")
                self.cur_antenna_num = temp_ant_num
                
                # Update meta SHM values
                meta_obj = self.shm_objects[6]
                meta_obj['elem_num'] = len(meta_data['antenna_list']) + self.META_ELEM
                meta_obj['size'] = meta_obj['elem_num'] * self.DOUBLE_SIZE
                os.ftruncate(meta_obj['shm_fd'], meta_obj['size'])
                
                # Update samples SHM values
                samples_obj = self.shm_objects[0]
                samples_obj['elem_num'] = len(meta_data['antenna_list']) * self.SAMPLES_NUM * 2
                samples_obj['size'] = samples_obj['elem_num'] * self.INT_SIZE
                os.ftruncate(samples_obj['shm_fd'], samples_obj['size'])
                        
            self.log.debug("[clearFrequencyService] Verifying Antenna and Sample size done")

                
            # Map shared memory object pointers
            print(f"Mapping Shared Memory for Objects...\n")
            self.log.debug("[clearFrequencyService] Mapping SHM")
            for obj in self.shm_objects:
                # Special: Skip Antenna_Num mapping
                if obj['name'] == '/antenna_num':
                    continue
                print(f"Mapping {obj['name']}")
                obj['shm_ptr'] = mmap.mmap(obj['shm_fd'], obj['size'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            self.log.debug("[clearFrequencyService] Mapping SHM done")
                            
            # Await for a Client Request
            print("[clearFrequencyService] Awaiting Client Request...\n")
            self.sf_client['sem'].acquire()
            print("[clearFrequencyService] Acquired Client Request...")
            
            # Check & Send Initialization Data
            if sample_sep is not None or restrict_data is not None or meta_data is not None:
                self.log.debug("[clearFrequencyService] Writing Initializaton data to SHM")
                
                print("[clearFrequencyService] Requesting Initialization Semaphore...")
                self.sl_init['sem'].acquire()
                print("[clearFrequencyService] Initialization Semaphore Acquired...")
                
                # # If restrict data present, send & overwrite 
                # if restrict_data is not None:    
                #     print(f"[Frequency Client] Data Write Progress: {self.shm_objects[5]['name']}")
                #     # self.write_m_data(self.shm_objects[5], [(int(x)) for x in restrict_data])
                    
                #     new_restrict_data = []
                #     # print("[Frequency Client] restrict_arr len: ", len(restrict_data))
                #     for restricted_freq in restrict_data:   
                #         new_restrict_data.append(int(restricted_freq[0]))
                #         new_restrict_data.append(int(restricted_freq[1]))
                        
                #     print("[Frequency Client] Writing restricted freq data:\n", restrict_data[:2], "...")
                #     self.shm_objects[5]['shm_ptr'].seek(0)
                #     self.shm_objects[5]['shm_ptr'].write(struct.pack('i' * (self.RESTRICT_NUM * 2), *new_restrict_data))
                
                # If sample separation present, send
                if sample_sep is not None:                
                    print(f"[Frequency Client] Data Write Progress: {self.shm_objects[4]['name']}")
                    self.write_data(self.shm_objects[4], sample_sep)
                                
                # TODO: write to meta_data after adding auto-padding to samples for num_antenna  
                # TODO: Implemented meta_data obj after smsep obj
                if meta_data is not None:
                    
                    # If antenna length has changed, send, set, and sync with server
                    if self.cur_antenna_num != len(meta_data['antenna_list']):
                        print(f"[Frequency Client] Antenna_num changed. Reallocating memory")
                        self.cur_antenna_num = len(meta_data['antenna_list'])
                        
                        # Send
                        print(f"[Frequency Client] Data Write Progress: {self.shm_objects[7]['name']}")
                        self.write_data(self.shm_objects[7], len(meta_data['antenna_list']))
                                                
                        # Reallocate meta SHM
                        meta_obj = self.shm_objects[6]
                        meta_obj['elem_num'] = len(meta_data['antenna_list']) + self.META_ELEM
                        meta_obj['size'] = meta_obj['elem_num'] * self.DOUBLE_SIZE
                        os.ftruncate(meta_obj['shm_fd'], meta_obj['size'])
                        meta_obj['shm_ptr'] = mmap.mmap(meta_obj['shm_fd'], meta_obj['size'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
                        
                        # Reallocate samples SHM
                        samples_obj = self.shm_objects[0]
                        samples_obj['elem_num'] = len(meta_data['antenna_list']) * self.SAMPLES_NUM * 2
                        samples_obj['size'] = samples_obj['elem_num'] * self.INT_SIZE
                        os.ftruncate(samples_obj['shm_fd'], samples_obj['size'])
                        samples_obj['shm_ptr'] = mmap.mmap(samples_obj['shm_fd'], samples_obj['size'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
                    
                    print(f"[Frequency Client] Data Write Progress: {self.shm_objects[6]['name']}")
                    
                    # Rearrange meta_data ordering
                    meta_data_list = [
                        meta_data['antenna_list'],
                        meta_data['number_of_samples'],
                        meta_data['x_spacing'],
                        meta_data['usrp_rf_rate'],
                    ]
                    self.write_data(self.shm_objects[6], meta_data_list, 'meta')
                    
                # Write Site ID (SID)
                print(f"[Frequency Client] Data Write Progress: {self.shm_objects[9]['name']}")
                print(f"    len of objects list is {len(self.shm_objects)}")
                self.write_data(self.shm_objects[9], self.sid, 'sid')
                    
                self.sl_init['sem'].release()
                self.sf_init['sem'].release()
                print("[clearFrequencyService] Initialization Semaphore Released ...")
                print("[clearFrequencyService] Server Initialization Flag raised ...")
                self.log.debug("[clearFrequencyService] Writing Initializaton data to SHM done")
                                
            if raw_samples is not None:
                self.log.debug("[clearFrequencyService] Writing Sample data to SHM")
                
                print("[clearFrequencyService] Awaiting Sample Semphore Lock...")
                self.sl_samples['sem'].acquire()

                # Write Sample data
                self.write_data(self.shm_objects[0], raw_samples, 'complex')
                                
                # If Sample-relevant Data given, write it
                for i in range(1, self.SAMPLE_PARAM_NUM):
                    print(f"[Frequency Client] Data Write Progress: {i}/{self.SAMPLE_PARAM_NUM - 1} {self.shm_objects[i]['name']}")
                    
                    # General: Write updated input data 
                    if input_data[i] is not None:
                        self.write_data(self.shm_objects[i], input_data[i])
                
                self.sl_samples['sem'].release()
                self.sf_samples['sem'].release()
                print("[Frequency Client] Done writing data to Shared Memory...")
                self.log.debug("[clearFrequencyService] Writing Sample data to SHM done")
                
                # Request Server 
                print("[clearFrequencyService] Requesting Server Response...")
                self.log.debug("[clearFrequencyService] Requesting Server response")
                self.sf_server['sem'].release()
                
                # Read-in Clear Freq data
                print("[clearFrequencyService] Awaiting Server Response...")
                self.sf_clrfreq['sem'].acquire()
                self.sl_clrfreq['sem'].acquire()
                print("[clearFrequencyService] Recieved Server Response. Reading Clear Freq data...")
                self.log.debug("[clearFrequencyService] Server response recieved")

                new_noise_data = []
                new_clrfreq_data = self.read_m_data(self.shm_objects[8])
                new_clrfreq_data, new_noise_data = self.repack_data(new_clrfreq_data, True)
                for clr_freq_and_noise in zip(new_clrfreq_data, new_noise_data):
                    print(f"[clearFrequencyService] Clear Freq Band: | {clr_freq_and_noise[0]} (kHz), {clr_freq_and_noise[1]} (N/A) |")
                clr_freq, noise = new_clrfreq_data[0], new_noise_data[0]
                
                self.sl_clrfreq['sem'].release()
                        
        except KeyboardInterrupt:
            print("[clearFrequencyService] Keyboard interrupt received. Exiting...")
        except posix_ipc.ExistentialError or ValueError or AttributeError:
                print("[clearFrequencyService] Shared memory has been delinked. Exiting...")
        finally:
            active_clients = self.decrement_active_clients()
                
        self.log.debug("[clearFrequencyService] clr_freq recieved...")
        return clr_freq, noise
    
    def cleanup_shm(self):
        if self.soft_kill is False or self.CLEAN_ON_INACTIVE is False:
            print("[clearFrequencyService] No active clients remaining, but not cleaning up shared resources to keep service idle.")
            try:
                posix_ipc.unlink_shared_memory(self.ACTIVE_CLIENTS_SHM_NAME)
                print(f"Unlinked shared memory {self.ACTIVE_CLIENTS_SHM_NAME}")
            except posix_ipc.ExistentialError or ValueError or AttributeError:
                print(f"Shared memory {self.ACTIVE_CLIENTS_SHM_NAME} does not exist")

        else:
            print("[clearFrequencyService] No active clients remaining; cleaning up shared resources.")
            for obj in self.shm_objects:
                try:
                    posix_ipc.unlink_shared_memory(obj['name'])
                    print(f"Unlinked shared memory {obj['name']}")
                except posix_ipc.ExistentialError or ValueError or AttributeError:
                    print(f"Shared memory {obj['name']} does not exist")
            try:
                posix_ipc.unlink_shared_memory(self.ACTIVE_CLIENTS_SHM_NAME)
                print(f"Unlinked shared memory {self.ACTIVE_CLIENTS_SHM_NAME}")
            except posix_ipc.ExistentialError or ValueError or AttributeError:
                print(f"Shared memory {self.ACTIVE_CLIENTS_SHM_NAME} does not exist")

            for sem in self.semaphores:
                try:
                    posix_ipc.unlink_semaphore(sem['name'])
                    print(f"Unlinked semaphore {sem['name']}")
                except posix_ipc.ExistentialError:
                    print(f"Semaphore {sem['name']} does not exist")
                                
    
class clearFrequencyRawDataManager():
    """ Buffers the raw clearfrequency data for all channels
    """
    def __init__(self, antenna_spacing, usrpManager):
        self.rawData    = None
        self.recordTime = None

        self.outstanding_request = False     # Flag set by the RadarChannelHandlers 
        self.repeat_request_for_2nd_period = False

        self.usrpManager = usrpManager # TODO change to take socks automatically form usrpManager
        self.usrp_socks = None
        self.center_freq = None
        self.sampling_rate = None
        self.number_of_samples = None
         
        self.metaData = {}
        self.rawData = None
        self.freq_occupied_by_other_channels = []
        self.get_raw_data_semaphore = threading.BoundedSemaphore()
        self.select_clear_freq = threading.BoundedSemaphore()
        

        self.metaData['x_spacing'] = antenna_spacing 

        self.logger = logging.getLogger('clearFrequency')
        self.logger.debug('clearFrequencyRawDataManager initialized')


    def set_usrp_driver_connections(self, usrp_driver_socks):
        self.usrp_socks = usrp_driver_socks
    
    def set_clrfreq_search_span(self, center_freq, clrfreq_sampling_rate, number_of_clrfreq_samples):
        self.center_freq = center_freq

        self.sampling_rate = clrfreq_sampling_rate
        self.number_of_samples = number_of_clrfreq_samples

        self.metaData['usrp_fcenter'] = self.center_freq 
        self.metaData['number_of_samples'] = self.number_of_samples 
        self.metaData['usrp_rf_rate'] = self.sampling_rate 

    def reset_occupied_freqs(self):
        self.freq_occupied_by_other_channels = []


    def update_auto_clear_freq_data(self, antenna_list, raw_data, meta_data_dict):
        """ Take the auto clear freq data collected at the end of tranmitting and update the clear freq class"""
        self.rawData = raw_data
        self.recordTime = meta_data_dict['record_time']
        self.antennaList = antenna_list
        self.sampling_rate = meta_data_dict['sampling_rate']
        self.center_freq = meta_data_dict['center_freq'] /1000
        self.logger.debug("Updated clear freq raw data with auto_clear_freq data")

        # # TODO get rid of two different meta_data dicts
        # self.metaData['usrp_fcenter'] = self.center_freq 
        # self.metaData['number_of_samples'] = len(self.rawData[0])
        # self.metaData['usrp_rf_rate'] = self.sampling_rate 
        # self.metaData['antenna_list'] = self.antennaList

    def record_new_data(self):
        assert self.usrp_socks != None, "no usrp drivers assigned to clear frequency search data manager"
        assert self.center_freq != None, "no center frequency assigned to clear frequency search manager"
        
        if self.recordTime == None:
            data_age = np.inf
            rec_new_samples = True
        else:
            data_age =  time.time() - self.recordTime 
            rec_new_samples = data_age > MAX_AGE_OF_AUTO_CLEAR_FREQ

        if rec_new_samples:
            self.logger.debug("clearFreqRawData: age of data is {:2.2f} s. Recoring new data ".format(data_age))
            self.logger.debug('start record_clrfreq_raw_samples')
            self.rawData, self.antennaList = record_clrfreq_raw_samples(self.usrpManager.get_all_main_antenna_socks(), self.number_of_samples, self.center_freq, self.sampling_rate)
            self.logger.debug('end record_clrfreq_raw_samples')

            self.metaData['antenna_list'] = self.antennaList
            self.logger.debug("recorded clear samples for clear frequency search, antenna list: {}".format(self.antennaList))
    
            # so, self.rawData is np.array(complex(nantennas, nsamples)
            self.recordTime = time.time()
    
            self.logger.debug("clrfreq record time: {}".format(self.recordTime))
        else:
            self.logger.debug("clearFreqRawData: age of data is {:2.2f} s. No need to record new data...".format(data_age))

        self.outstanding_request = False


    def get_raw_data(self):
        self.get_raw_data_semaphore.acquire()
        if self.rawData is None: 
           self.record_new_data()
        else:
           print("clearFreqDataManager: provide raw data (age {}), setting raw_rec request ".format(time.time() - self.recordTime))
           self.outstanding_request = True
          
        self.get_raw_data_semaphore.release()
        return self.rawData, self.metaData, self.recordTime

    def add_channel(self, freq, bandwidth):
        freq *= 1000
        self.freq_occupied_by_other_channels.append([freq - bandwidth*1.5, freq + bandwidth*1.5])

class swingManager():
    """ Class to handle which swing is active and processing """

    def __init__(self):
        self.activeSwing       = 0
        self.processingSwing   = 1

        # async buffers for control program handlers
        # a/p swing status is switched in main loop async from crtl progam. this var is used for GetDataHandler to get the correct swing no matter when handler is called
        self.nextSwingToTrigger = self.activeSwing
    def reset(self):
        """ This should only be called if no other channels are active"""
        self.__init__() # for now reset is the same as init

    def switch_swings(self):
        self.activeSwing     = 1 - self.activeSwing
        self.processingSwing = 1 - self.processingSwing

class scanManager():
    """ Class to handle
        - last recorded clear frequency search raw data
        - keep track of beam numbers and transmit frequencies
        - times when periods starts 
        created for each RadarChannelHandler """
        

    def __init__(self, restricted_frequency_list, channel):
        self.scan_beam_list        = []
        self.clear_freq_range_list = []
        self.fixFreq = None
        
        self.current_period = 0
        self.repeat_clrfreq_recording = False # 2nd period is triggered automatically before ROS finishes 1st. if CLR_FRQ was requested for 1st => also do record on 2nd
       
        self.channel = channel
        self.RHM = channel.parent_RadarHardwareManager
        self.clearFreqService = ClearFrequencyService()
        self.beamSep = self.RHM.array_beam_sep
        self.numBeams = self.RHM.array_nBeams

        self.current_clrFreq_result = None
        self.next_clrFreq_result    = None 
        self.isPrePeriod = True # is vert first trigger_next_period() call that just triggers first period but does not collect cuda data
        self.isPostLast = False # to handle last trigger_next_swing() call
        self.logger = logging.getLogger('scanManager')

 ###       self.get_clr_freq_raw_data  = None # handle to RHM:ClearFrequencyRawDatamanager.get_raw_data()
        self.logger.debug("in scanManager __init__. Setting isInitSetParameter true")
        self.isInitSetParameter = True
        self.restricted_frequency_list = restricted_frequency_list

        self.syncBeams  = False
        self.beam_times = None
        self.scan_duration = None
        self.integration_duration = None 
        self.camping = False
 
        self.isFirstPeriod = True

    def get_time_in_scan(self):
        """ Returns the time in seconds from the scheduled start of the scan. Used to set start period. """

        current_time = datetime.datetime.now().time()
        nSeconds_in_this_hour = current_time.minute*60 + current_time.second + current_time.microsecond/1e6
        seconds_in_this_scan = nSeconds_in_this_hour % self.scan_duration
        return seconds_in_this_scan

    def set_start_period(self): 
        """  OLD: start beam is now calculated by control program """
        """ Sets the current period to the start periods depending on the current time. 
            Corresponds to the skip variable of the old ontrol program """
        return 
        time_in_scan = self.get_time_in_scan()
        current_time = time_in_scan + self.integration_duration - 0.1 # taken over from old control program code 
        iPeriod = np.floor((current_time % self.scan_duration) / self.integration_duration) 
        if iPeriod > (self.numBeams - 1) or iPeriod < 0:
            iPeriod = 0
        self.current_period = int(iPeriod)
        self.logger.info("Starting scan with period  {}.".format(iPeriod))

    def wait_for_next_trigger(self):
        if self.syncBeams:
           time_to_wait = self.beam_times[self.current_period] - self.get_time_in_scan() - self.integration_time_manager.get_usrp_delay_time()
           if time_to_wait > 0:
              self.logger.debug("Waiting for {} s".format(time_to_wait))
              time.sleep(time_to_wait)
           else:
              self.logger.debug("No waiting. ({} + {}) s too late.".format(time_to_wait + self.integration_time_manager.get_usrp_delay_time(), self.integration_time_manager.get_usrp_delay_time()))
            
           
    def init_new_scan(self, freq_range_list, scan_beam_list, fixFreq, scan_times_list, scan_duration, integration_duration, start_period):
  
        # list of [fstart, fstop] lists in Hz, desired frequency range for each period
        self.clear_freq_range_list = freq_range_list

        # list of [bmnum, bmnum..] 
        self.scan_beam_list = scan_beam_list
        self.camping = len(scan_beam_list) == 1

        # sync paramater
        self.syncBeams  = scan_times_list != None
        self.beam_times = scan_times_list
        self.scan_duration   = scan_duration
        self.integration_duration = integration_duration 
    
        self.fixFreq = fixFreq

        # rest all other parameter
        self.current_period         = start_period
        self.current_clrFreq_result = None
        self.next_clrFreq_result    = None 
        self.isPrePeriod            = True # is vert first trigger_next_period() call that just triggers first period but does not collect cuda data
        self.isPostLast             = False # to handle last trigger_next_swing() call
        self.logger.debug("in init_new_scan. Setting isInitSetParameter true")
        self.isInitSetParameter     = True
        self.isFirstPeriod          = True

    def get_nSec_to_scan_boundary(self, time_now):
        """ To limit the time of integration periods  """
        nSec_left = self.scan_duration - ((time.gmtime(time_now).tm_min*60 + time.gmtime(time_now).tm_sec + time_now % 1) % self.scan_duration )
        return nSec_left


    # XXX store current freqencies in clrfreq manager,
    # add to restricted
    def period_finished(self):
      #  print("swing manager period finished... ")
        self.current_clrFreq_result = self.next_clrFreq_result
        self.next_clrFreq_result = None
        if self.isPrePeriod:
           self.isPrePeriod = False
           return

        if self.camping:
            self.logger.debug("Camping beam, no need to increasing current_period.")
        elif not self.isLastPeriod:
            self.current_period += 1
            self.logger.debug("Increasing current_period to {}".format(self.current_period))
        else:
            self.logger.debug("Last period current_period is still {}".format(self.current_period))
            self.isPostLast = True

        
    def status(self):
        print("current period: {: >2d}/{}, beam: {} ".format(self.current_period, len(self.scan_beam_list), self.current_beam))
    
    @property        
    def current_beam(self):
       # print("Requesting current beam for period {}".format(self.current_period))
        return self.scan_beam_list[self.current_period]
        
    @property        
    def next_beam(self):
        if self.camping:
            return self.scan_beam_list[0]

        if self.current_period == len(self.scan_beam_list) -1:
            return None
        else:
            return self.scan_beam_list[self.current_period+1]

    def get_current_clearFreq_result(self):
        if self.current_clrFreq_result is None:
           if self.fixFreq > 0: # it looks like control program could use -1 and 0 to disable it
               self.current_clrFreq_result = [self.fixFreq, 0, 0]
               self.logger.debug("Using fixed frequency of {} kHz for current period".format(self.fixFreq))
           else:
               self.logger.debug("  calc current clr_freq (ch {}, period {})".format(self.channel.cnum, self.current_period))
               self.current_clrFreq_result = self.evaluate_clear_freq(self.current_period, self.current_beam)
        return self.current_clrFreq_result
        
    def get_next_clearFreq_result(self):
        if self.next_clrFreq_result is None:
           if self.fixFreq != -1 and self.fixFreq != 0: # it looks like control program could use -1 and 0 to disable it
               self.next_clrFreq_result = [self.fixFreq, 0, 0]
               self.logger.debug("Using fixed frequency of {} kHz for next period".format(self.fixFreq))
           else:
               self.logger.debug("  calc next clr_freq (ch {}, period {})".format(self.channel.cnum, self.current_period+1))
               # print("  calc next clr_freq (period {})".format(self.current_period+1))
               if self.camping:
                   next_period = self.current_period
               else:
                   next_period = self.current_period + 1

               self.next_clrFreq_result = self.evaluate_clear_freq(next_period ,self.next_beam)
        return self.next_clrFreq_result        
        
    def evaluate_clear_freq(self, iPeriod, beamNo):
        # TODO make sure this is function is only called once at a time
     ### rawData, metaData, recordTime = self.get_clr_freq_raw_data()
        RHM = self.RHM
        rawData, metaData, recordTime = RHM.clearFreqRawDataManager.get_raw_data()
    
        beam_angle = calc_beam_azm_rad(self.numBeams, beamNo, self.beamSep)
        RHM.clearFreqRawDataManager.select_clear_freq.acquire()
        self.logger.debug("clear_freq_range: {} on beam {} angle {}".format(self.clear_freq_range_list[iPeriod], beamNo, beam_angle))
   
        all_restricted_freq = self.restricted_frequency_list + RHM.clearFreqRawDataManager.freq_occupied_by_other_channels
        self.logger.debug('start calc_clear_freq_on_raw_samples')
        # clearFreq, noise = calc_clear_freq_on_raw_samples(rawData, metaData, all_restricted_freq, self.clear_freq_range_list[iPeriod], beam_angle, self.channel.raw_export_data['smsep'])
        
        clear_freq_range = []
        for freq in self.clear_freq_range_list[iPeriod]: 
            clear_freq_range.append(int(freq))
        print(f"smsep:      {int(self.channel.raw_export_data['smsep'])}")
        print(f"fcenter:    {int(metaData['usrp_fcenter'])}")
        print(f"beamNo:     {int(beamNo)}")
        print(f"antenna sample sets: { len(rawData) }")
        clearFreq, noise = self.clearFreqService.request_clr_freq(rawData, clear_freq_range, int(metaData['usrp_fcenter']), int(beamNo), int(self.channel.raw_export_data['smsep']), meta_data=metaData)
        
        
        self.logger.debug('end calc_clear_freq_on_raw_samples')
        if 'baseband_samplerate' in RHM.commonChannelParameter: 
           bandwidth = RHM.commonChannelParameter['baseband_samplerate'] 
        else:    # first call before channel details are known
           bandwidth = 3333

        RHM.clearFreqRawDataManager.add_channel(clearFreq, bandwidth)

        self.logger.debug("clear freq result for channel {}: selected {} , noise level {:2.1f}".format(self.channel.cnum, clearFreq, noise))
        RHM.clearFreqRawDataManager.select_clear_freq.release()

        return (clearFreq, noise, recordTime)


    @property
    def isForelastPeriod(self):
        if self.camping:
           return False
        return self.current_period + 2 == len(self.scan_beam_list) or len(self.scan_beam_list) == 1 # for one beam scan: first is also forelast
        
    @property
    def isLastPeriod(self):
        if self.camping:
           return False
        return self.current_period + 1 == len(self.scan_beam_list) and len(self.scan_beam_list) != 1 # for one beam scan: first is not last
       
# handle arbitration with multiple channels accessing the usrp hardware
# track state of a grouping of usrps
# merge information from multiple control programs, handle disparate settings
# e.g, ready flags and whatnot
class RadarHardwareManager:
    def __init__(self, port):
        self.client_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        self.client_sock.bind(('localhost', port))

        self.logger = logging.getLogger('HwManager')
        self.logger.info('listening on port ' + str(port) + ' for control programs')

        self.exit_usrp_server = False
 
        self.ini_file_init()
        self.usrp_init()
        self.rxfe_init()
        #self.test_rxfe_control() # toggle all amp and att stages on and off for testing
        self.cuda_init()

        self.nRegisteredChannels = 0  # number of channels after compatibility check
        self.n_SetParameterHandlers_active = 0 
        self.nControlPrograms  = 0  # number of control programs, also include unregistered channels
        self.channel_manager_consecutive_number = 10 # serial number shown in logger of channel_manager

        self.clearFreqRawDataManager = clearFrequencyRawDataManager(self.array_x_spacing, self.usrpManager)
        self.clearFreqRawDataManager.set_usrp_driver_connections(self.usrpManager.socks) # TODO check if this also works after reconnection to a usrp (copy or reference?)

        self.clearFreqRawDataManager.set_clrfreq_search_span(self.mixingFreqManager.current_mixing_freq, self.usrp_rf_rx_rate, self.usrp_rf_rx_rate / CLRFREQ_RES)
        self.active_channels     = []   # list of channels where ROS called SET_ACTIVE
        self.channels            = []   # all channels that are really transmitting
        self.newChannelList      = []   # waiting list for channels to be added at the right time (between two trigger_next() calls)
###        self.record_new_data     = self.clearFreqRawDataManager.record_new_data
        self.swingManager        = swingManager()

        self.set_par_semaphore = threading.BoundedSemaphore()
        self.processing_swing_invalid = False
        self.trigger_next_function_running = False
        self.commonChannelParameter = {}
        self.integration_time_manager = integrationTimeManager(self)
        self.nSequences_per_period = 0

        self.auto_collect_clrfrq_after_rx = True

    def run(self):
        def spawn_channel(conn):
            # start new radar channel handler
            self.nControlPrograms += 1
            channel = RadarChannelHandler(conn, self)
            self.channel_manager_consecutive_number += 1
            try:
                channel.run()
            except socket.error:
                self.logger.error("RadarChannelHandler: Socket error => Deleting channel... ")
                self.unregister_channel_from_HardwareManager(channel)
            self.nControlPrograms -= 1

        # not working, unused...
        def radar_main_control_loop_environment():
            "Run main_contol loop and catch errors"
            # TODO shut down everything and ALSO print the error!
           # try:
           #    radar_control_loop()
           # except:
           #    e = sys.exc_info()[0]
           #    self.logger.error(e.__str__())
           #    self.logger.error("Error in mail contol loop. Shutting down usrp_server...")
           #    self.exit()

        # TODO: add lock support
        def radar_main_control_loop():
            controlLoop_logger = logging.getLogger('Control Loop')
            controlLoop_logger.info('Starting RHM.radar_main_control_loop() ')
            statusFile = statusUpdater(self)
            sleepTime = 0.01 # used if control loop waits for one channel
           
            while True:

                # statusFile.update()
                statusFile.update_advanced()

                if self.exit_usrp_server:
                   self.logger.info("ending main_control_loop ")
                   break

                # set start time of integration period (will be overwriten if not triggered)
                self.starttime_period = time.time() # TODO change this to refence clock and scan times

                # check if there are any disconnected URSPs
                if len(self.usrpManager.addressList_inactive):
                    self.usrpManager.restore_lost_connections()

                # CLEAR FREQ SEARCH: recoring when ever requested (independent of swing, state or channel)
                if self.clearFreqRawDataManager.outstanding_request:
                    controlLoop_logger.debug('start self.clearFreqRawDataManager.record_new_data()')
                    self.clearFreqRawDataManager.record_new_data()

                    # check if CLR_FREQ has to be repeated
                    if CS_INACTIVE in [ch.active_state for ch in self.channels]:
                       controlLoop_logger.debug("Repeating CLR_FREQ for next integation period")
                       self.clearFreqRawDataManager.repeat_request_for_2nd_period = True              

                    controlLoop_logger.debug('end self.clearFreqRawDataManager.record_new_data()')


                # FRIST CUDA_ADD FOR NEW CHANNELS
                if len(self.newChannelList) != 0:                   
                   self.logger.debug("active_channel list: {}".format([active_ch.cnum for active_ch in self.active_channels]))
                   self.logger.debug("channel list: {}".format([ch.cnum for ch in self.channels]))
                   self.logger.debug("new channel list: {}".format([ch.cnum for ch in self.newChannelList]))

                   for active_ch in self.active_channels:
                      while (active_ch not in (self.channels + self.newChannelList)):
                         self.logger.info("Waiting for ch {} to be added to newChannelList".format(active_ch.cnum))
                         time.sleep(0.01)
                   while( self.n_SetParameterHandlers_active):
                      self.logger.debug("Waiting for all {} SetParameterHandlers to finish before initializing new channels".format(self.n_SetParameterHandlers_active))
                      time.sleep(0.001)
                   controlLoop_logger.info('initializing channel')
                   self.initialize_channel()
                           

                # TRIGGER if any channel is in CS_TRIGGER
                if CS_TRIGGER in [ch.active_state for ch in self.channels]:
                    # wait for all channels to be in TRIGGER state
                    executeTrigger = True
                    for ch in self.channels:
                        if  ch.active_state not in [CS_TRIGGER, CS_INACTIVE]:
                            controlLoop_logger.debug('remaining in TRIGGER because channel {} state is {} (active swing is {})'.format(ch.cnum, ch.active_state, ch.swingManager.activeSwing))
                            time.sleep(sleepTime)
                            executeTrigger = False

                    # if all channels are TRIGGER, then TRIGGER
                    if executeTrigger:
                        controlLoop_logger.debug('start RHM.trigger_next_swing()')
                        self.trigger_next_swing()
                        controlLoop_logger.debug('end RHM.trigger_next_swing()')

                else:
                    time.sleep(RADAR_STATE_TIME) # sleep to reduce load of this while loop

        # end of radar_main_control_loop()


        self.client_sock.listen(MAX_CHANNELS)
        client_threads = []
        usrp_server_logger = logging.getLogger('usrp_server')

        ct = threading.Thread(target=radar_main_control_loop)
        ct.start()
        while True:
            usrp_server_logger.info('waiting for control program')
            client_conn, addr = self.client_sock.accept()

            if self.exit_usrp_server:
               self.logger.info("ending control program sock loop ")
               break

            usrp_server_logger.info('connection from control program, spawning channel handler thread')
            ct = threading.Thread(target=spawn_channel, args=(client_conn,), daemon=False)
            client_threads.append(ct)
            ct.start()
       
            # remove threads that are not alive
            # TODO: the client_sock.accept is probably blocking, this will only execute after connection of a new control program. is that the intended behavior? -jtk
            client_threads = [iThread for iThread in client_threads if iThread.is_alive()]

        self.client_sock.close()


    # read in ini config files..
    def ini_file_init(self):
        # READ driver_config.ini
        driver_config = configparser.ConfigParser()
        driver_config.read('../driver_config.ini')
        self.ini_shm_settings     = driver_config['shm_settings']
        self.ini_cuda_settings    = driver_config['cuda_settings']
        self.ini_network_settings = driver_config['network_settings']

        # READ usrp_config.ini
        usrp_config = configparser.ConfigParser()
        usrp_config.read('../usrp_config.ini')
        usrp_configs = []
        self.antenna_idx_list_main = []
        self.antenna_idx_list_back = []
        for usrp in usrp_config.sections():
            usrp_configs.append(usrp_config[usrp])
            if usrp_config[usrp]['mainarray'].lower() in ['true', 1]:
               self.antenna_idx_list_main.append(int(usrp_config[usrp]['array_idx']))
            else:
               self.antenna_idx_list_back.append(int(usrp_config[usrp]['array_idx']))
            
        self.ini_usrp_configs = usrp_configs

        # READ array_config.ini
        array_config = configparser.ConfigParser()
        array_config.read('../array_config.ini')
        self.ini_rxfe_settings  = array_config['rxfe']
        self.scaling_factor_tx_total = float(array_config['gain_control']['scaling_factor_tx_total'])
        self.scaling_factor_rx_bb    = float(array_config['gain_control']['scaling_factor_rx_bb'])
        self.scaling_factor_rx_if    = float(array_config['gain_control']['scaling_factor_rx_if'])
        self.apply_normalization     = array_config.getboolean('gain_control','use_var_normalization')
        mute_antenna_str = array_config['gain_control']['mute_antenna_idx'] 
        if len(mute_antenna_str):
            self.mute_antenna_list = [int(x) for x in mute_antenna_str.split(",")]
        else:
            self.mute_antenna_list = []

        if self.apply_normalization:
            self.logger.info("Normalizing is active.")
        if len(self.mute_antenna_list):
            self.logger.info("Mute antennas before beamforming: {}".format(self.mute_antenna_list))


        self.ini_array_settings = array_config['array_info']
        self.array_beam_sep  = float(self.ini_array_settings['beam_sep'] ) # degrees
        self.array_nBeams    = int(  self.ini_array_settings['nbeams'] )
        self.array_x_spacing = float(self.ini_array_settings['x_spacing'] ) # meters 
        self.hardwareLimit_freqRange = [float(array_config['hardware_limits']['minimum_tfreq'] ) /1000, float(array_config['hardware_limits']['maximum_tfreq'] )/1000] # converted to kHz

    def usrp_init(self):
        self.usrpManager = usrpSockManager(self)
        self.usrp_rf_tx_rate   = int(self.ini_cuda_settings['FSampTX'])
        self.usrp_rf_rx_rate   = int(self.ini_cuda_settings['FSampRX'])
        self.mixingFreqManager = usrpMixingFreqManager(DEFAULT_USRP_MIXING_FREQ, self.usrp_rf_tx_rate/1000)

        self._resync_usrps(first_sync = True)


    def send_cuda_setup_command(self):
      if self.commonChannelParameter == {}:
         self.logger.debug("Skipping call of cuda_setup because up/down samplingRates are unknown.")
      else:
         self.logger.debug("start CUDA_SETUP")
         cmd = cuda_setup_command(self.cudasocks, self.commonChannelParameter['upsample_rate'],self.commonChannelParameter['downsample_rates'][0],self.commonChannelParameter['downsample_rates'][1], self.mixingFreqManager.current_mixing_freq*1000 )
         cmd.transmit()
         cmd.client_return() 
         self.logger.debug("end CUDA_SETUP")

    def _resync_usrps(self, first_sync = False):
        usrps_synced = False
        iResync = 1
        
        if not len(self.usrpManager.socks):
            self.logger.error('no USRPs drivers are connected to usrp_server! exiting..')
            sys.exit(0)

        while not usrps_synced:
            cmd = usrp_sync_time_command(self.usrpManager.socks)
            cmd.transmit()
            self.usrpManager.eval_client_return(cmd)

            # once USRPs are connected, synchronize clocks/timers 
            cmd = usrp_get_time_command(self.usrpManager.socks)
            cmd.transmit() 

            usrptimes = []
            for iUSRP, usrpsock in enumerate(self.usrpManager.socks):
                try:
                    usrptimes.append(cmd.recv_time(usrpsock))
                except:
                    self.logger.error("Error in sync USRPs for {}. Removing it...".format(self.usrpManager.hostnameList_active[iUSRP]))
                    self.usrpManager.remove_sock(usrpsock)
           
            self.usrpManager.eval_client_return(cmd)
     
            # check if sync succeeded..
            if max(np.abs(np.array(usrptimes) - usrptimes[0])) < .5:
                usrps_synced = True
                print('USRPs synchronized, approximate times: ' + str(usrptimes))
            else:
                # TODO: why does USRP synchronization fail?
                self.logger.info("USRP times: {}".format(usrptimes))
                self.logger.warning('_resync_USRP USRP syncronization failed, trying again ({}) ...'.format(iResync))
                iResync += 1 
                time.sleep(0.2)
        if not first_sync:        
            self.clearFreqRawDataManager.set_usrp_driver_connections(self.usrpManager.socks) 


    #@timeit
    def rxfe_init(self):
        amp1 = self.ini_rxfe_settings.getboolean('enable_amp1')
        amp2 = self.ini_rxfe_settings.getboolean('enable_amp2')

        att = float(self.ini_rxfe_settings['attenuation'])
        if att < 0:
           self.logger.warning('attenuation for rxfe in array.ini is defnined positive, but given value is negative ({} dB). correcting that to {} dB...'.format(att, att*(-1)))
           att *= -1

        if att > 31.5:
           self.logger.warning('attenuation ({}) for rxfe in array.ini is > 31.5 dB. using maximum atenuation of 31.5 dB'.format(att))
           att = 31.5

        self.logger.info("Setting RXFE: Amp1={}, Amp2={}, Attenuation={} dB".format(amp1, amp2, att)) 
        cmd = usrp_rxfe_setup_command(self.usrpManager.socks, amp1, amp2, att*2) # *2 since LSB is 0.5 dB 
        cmd.transmit()
        self.usrpManager.eval_client_return(cmd)


    def test_rxfe_control(self):
        """ Function that steps through all amp and att stages of the rxfe board """

        self.logger.warning("Starting RXFE test: walk through all bits:") 

        testParSet = [[False, False, 0], [True, False, 0], [True, True, 0], [False, True, 0], [True, True, 31.5]] + [[False, False, 2**i/2] for i in range(6) ]
        
        nSets = len(testParSet)
        #while True:

#<<<<<<< HEAD
            #print("flasing wax off")
            #cmd = usrp_rxfe_setup_command(self.usrpManager.socks, False, False, 0) # *2 since LSB is 0.5 dB 
            #cmd.transmit()
            #cmd.client_return()
            #time.sleep(1)
            #print("flasing wax on")
            #cmd = usrp_rxfe_setup_command(self.usrpManager.socks, True, True, 63) # *2 since LSB is 0.5 dB 
            #cmd.transmit()
            #cmd.client_return()
            #time.sleep(1)
#=======
#>>>>>>> updated dio worker to work with mcm style phasing cards for transmit and sync pulses (untested)

        for iSet in range(nSets):
            amp1 = testParSet[iSet][0]
            amp2 = testParSet[iSet][1]
            att  = testParSet[iSet][2]

            cmd = usrp_rxfe_setup_command(self.usrpManager.socks, amp1, amp2, att*2) # *2 since LSB is 0.5 dB 
            cmd.transmit()
            cmd.client_return()
            self.logger.warning("Current settings: Amp1={}, Amp2={}, Attenuation={} dB".format(amp1, amp2, att)) 
            #npt = input('  Press Enter for next chage... ')
            time.sleep(2)

        print("Finished testing RXFE!")

    def cuda_init(self):
        #time.sleep(.05)

        # connect cuda_driver servers
        cuda_driver_socks = []

        cuda_driver_port = int(self.ini_network_settings['CUDADriverPort'])
        cuda_driver_hostnames = [name.strip() for name in self.ini_network_settings['CUDADriverHostnames'].split(',')]

        for c in cuda_driver_hostnames:
           try:
                self.logger.debug('connecting to cuda driver on {}:{}'.format(c, cuda_driver_port))
                cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                cudasock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                cudasock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
                cudasock.connect((c, cuda_driver_port))
                cuda_driver_socks.append(cudasock)
           except ConnectionRefusedError:
                self.logger.error("cuda server connection failed on {}".format(c))
                sys.exit(1)

        if len(cuda_driver_socks) == 0:
           self.logger.error("No cuda connection available. Exiting usrp_server")
           sys.exit(1)

        self.cudasocks = cuda_driver_socks
    
    def setup(self, chan):
        pass
  
    def initialize_channel(RHM):
        """ Adds first period of channel for new channel or after CS_INACTIVE. Also appends channel to RHM.channels if not already done."""
        wait_start_time = time.time()
        while (time.time() - wait_start_time < 0.1) and (RHM.nControlPrograms > len(RHM.channels) + len(RHM.newChannelList) ):
           RHM.logger.debug("initialize_channel: waiting 5 ms for other control program to SET_PARAMETER")
           time.sleep(0.005)

        RHM.set_par_semaphore.acquire()
        RHM.logger.debug("start initialize_channel")
        newChannelList = RHM.newChannelList.copy()  #  make a copy in case another channel is added during this function call
        
        nChannelsNew = 0
        for ch2add in newChannelList:
           if ch2add not in RHM.channels:
              nChannelsNew += 1
        RHM.apply_channel_scaling(nChannelsWillBeAdded=nChannelsNew)

        RHM._calc_period_details(newChannels=newChannelList) # TODO only if this is first channel?
        for channel in newChannelList:
     
            # CUDA_ADD_CHANNEL in first period
            cmd = cuda_add_channel_command(RHM.cudasocks, sequence=channel.get_current_sequence(), swing = channel.swingManager.activeSwing)
            RHM.logger.debug('calling CUDA_ADD_CHANNEL at initialize_channel() (cnum {}, swing {}, beam {})'.format(channel.cnum, channel.swingManager.activeSwing, channel.scanManager.current_beam))
            cmd.transmit()
            cmd.client_return()      
            if channel.active_state == CS_INACTIVE: 
               RHM.logger.debug("initialize_channel() is setting ch {} swing {} from CS_INACTIVE to CS_READY".format(channel.cnum,  channel.swingManager.activeSwing))
               channel.active_state = CS_READY # channel not really ready until CUDA_GENERATE, but there will be no trigger in parallel to this function
            else:
               RHM.logger.debug("initialize_channel() ch {} state stays {}".format(channel.cnum, channel.active_state))


            if channel not in RHM.channels:
               RHM.channels.append(channel)
               RHM.logger.debug("Adding channel {} to RHM.channels".format(channel.cnum))
            RHM.newChannelList.remove(channel)

        # CUDA_GENERATE for first period
        RHM.logger.debug('start CUDA_GENERATE_PULSE swing {} (1st period) '.format(RHM.swingManager.activeSwing))
        cmd = cuda_generate_pulse_command(RHM.cudasocks, RHM.swingManager.activeSwing, RHM.mixingFreqManager.current_mixing_freq*1000)
        cmd.transmit()
        cmd.client_return()
        RHM.logger.debug('end CUDA_GENERATE_PULSE (1st period)')
        RHM.logger.debug("end initialize_channel")

        RHM.set_par_semaphore.release()


    def unregister_channel_from_HardwareManager(self, channelObject):
        iRun = 0
        while self.trigger_next_function_running:
            if (iRun % 100000) == 0:
                self.logger.debug("Waiting for trigger_next_swing() to finish before deleting channel...")
           # no time.sleep() here because there is not much time between two trigger calls...
            iRun += 1
        
        if channelObject in self.active_channels:
           self.active_channels.remove(channelObject)

        if channelObject in self.newChannelList:
           self.newChannelList.remove(channelObject)

        if channelObject in self.channels:
       # this is only called if something went wrong or crtl program quit => so don't care about channel states ? 
       #    # don't delete channel in middle of trigger, pretrigger, or ....
       #     channelObject._waitForState([CS_READY, CS_INACTIVE])  
            self.logger.info('unregister_channel_from_HardwareManager() removing channel {} from HardwareManager'.format(self.channels.index(channelObject)))
            self.channels.remove(channelObject)

            # remove channel from cuda
            self.logger.debug('send CUDA_REMOVE_CHANNEL')
            for iSwing in range(nSwings):
               try:
                   cmd = cuda_remove_channel_command(self.cudasocks, sequence=channelObject.get_current_sequence(remove_channel=True), swing = iSwing)
                   cmd.transmit()
                   cmd.client_return()
               except AttributeError:
                   # catch errors where channel.getSequence() fails because npulses_per_sequence is uninitialized
                   # TODO: discover why this happens..
                   self.logger.error('unregister_channel_from_HardwareManager() failed to remove channel from HardwareManager')
                
            self.logger.debug('RHM:unregister_channel_from_HardwareManager {} channels left'.format(len(self.channels)))

            self.nRegisteredChannels -= 1
            if self.nRegisteredChannels == 0:  
                self.commonChannelParameter = {}

        else:
            self.logger.warning('unregister_channel_from_HardwareManager() channel already deleted')


        

    def exit(self):
        self.logger.warning("Entering RadarHardwareManager.exit() for clean exit")
        self.disconnect_driver_and_clean_up()
     #   self.logger.debug("Closing client socket... ")
     #   self.client_sock.close()
        self.logger.debug("Settinge exit flag... ")
        self.exit_usrp_server = True

        # conntect to ros port overcome blocking          
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect(('localhost', RMSG_PORT))
        sock.close()
        # sys.exit(0)

    def disconnect_driver_and_clean_up(self):
        self.logger.warning("Entering RadarHardwareManager.disconnect_driver_and_clean_up() ")
        # clean up and exit
        self.client_sock.close()

        if hasattr(self, 'usrpManager'):
           cmd = usrp_exit_command(self.usrpManager.socks)
           cmd.transmit()
           for sock in self.usrpManager.socks:
               sock.close()

        if hasattr(self, 'cudasocks'): 
            cmd = cuda_exit_command(self.cudasocks)
            cmd.transmit()

            for sock in self.cudasocks:
                sock.close()
        
        # clean up server semaphores
       # if hasattr(self, 'set_par_semaphore'):
       #    self.set_par_semaphore.release()
         #  self.set_par_semaphore.unlink()
        if hasattr(self, 'mixingFreqManager'):
           self.mixingFreqManager.semaphore.release()
           self.mixingFreqManager.semaphore.unlink()
        
     

    def _calc_period_details(self, newChannels=[]):
        """ calculate details for integration period and save it in channel objects"""
       
        self.logger.debug("_calc_period_details: samplingRate _bb={}, number_of_samples={}".format(self.commonChannelParameter['baseband_samplerate'], self.commonChannelParameter['number_of_samples']))
        # calculate the pulse sequence period with padding
    #    nSamples_per_sequence = self.commonChannelParameter['number_of_samples'] + int(PULSE_SEQUENCE_PADDING_TIME * self.commonChannelParameter['baseband_samplerate'])
        nSamples_per_sequence = max(self.commonChannelParameter['number_of_samples']+2 , int(np.ceil((self.commonChannelParameter['pulse_sequence_offsets_vector'][-1]+ self.commonChannelParameter['pulseLength']/1e6 + PULSE_SEQUENCE_PADDING_TIME) * self.commonChannelParameter['baseband_samplerate'])))
        pulse_sequence_period = nSamples_per_sequence / self.commonChannelParameter['baseband_samplerate']  

        self.logger.debug("nSamples_per_sequence: {}, pulse_sequence_period: {}".format(nSamples_per_sequence, pulse_sequence_period))

        self.logger.debug("self.starttime_period: {}".format(self.starttime_period))
        self.logger.debug("self.commonChannelParameter['integration_period_duration: {}".format(self.commonChannelParameter['integration_period_duration']))

        # to find out how much time is available in an integration period for pulse sequences, subtract out startup delay
       # transmitting_time_left = self.starttime_period + self.commonChannelParameter['integration_period_duration'] - time.time() - self.integration_time_manager.get_usrp_delay_time() - self.integration_time_manager.estimate_calc_time()

        time_now = time.time()
        nSec_to_end_of_period =  self.commonChannelParameter['integration_period_duration']  - time_now  + self.starttime_period
        # reduce time if there is a scan boundary
        for ch in self.channels+newChannels:
            nSec_to_end_of_period = min(nSec_to_end_of_period, ch.scanManager.get_nSec_to_scan_boundary(time_now))
        
        transmitting_time_left = nSec_to_end_of_period  - self.integration_time_manager.estimate_calc_time()  - self.integration_time_manager.get_usrp_delay_time()


        if transmitting_time_left <= 0:
            transmitting_time_left = 0
            self.logger.warning("no time is left in integration period for sampling!")


        # calculate the number of pulse sequences that fit in the available time within an integration period
        nSequences_per_period = int(transmitting_time_left / pulse_sequence_period)
        nSequences_per_period_max = int((self.commonChannelParameter['integration_period_duration']  - self.integration_time_manager.get_usrp_delay_time() - self.integration_time_manager.estimate_calc_time() ) / pulse_sequence_period)
        ### sampling_duration = pulse_sequence_period * nSequences_per_period   # just record full number of sequences


        # calculate the number of RF transmit and receive samples
        downsamplingRates =  self.commonChannelParameter["downsample_rates"]
        if nSequences_per_period == 0:
           nSamples_per_sequence_if = nSequences_per_period  
           num_requested_rx_samples = nSequences_per_period
        else:
           nSamples_per_sequence_if =  int(downsamplingRates[1])* ((nSamples_per_sequence*nSequences_per_period) - 1 ) +  int(downsamplingRates[1]*2) # assumes fixed nTaps for filter = 2*downsampling 
           num_requested_rx_samples =  int(downsamplingRates[0])* (nSamples_per_sequence_if                      - 1 ) +  int(downsamplingRates[0]*2) # assumes fixed nTaps for filter = 2*downsampling 

        self.logger.debug("RFIFRATE: {}, IFBBRATE: {}, nSamples_per_sequence_if: {}, nSamples_per_sequence: {}, nSequences_per_period: {}, NTapsRX_ifbb: {}, NTapsRX_rfif: {}".format( \
                downsamplingRates[0], downsamplingRates[1], nSamples_per_sequence_if, nSamples_per_sequence, nSequences_per_period, downsamplingRates[0]*2, downsamplingRates[1]*2))
        

        self.logger.info("Effective integration time: {:0.3f}s = {} sequences ({}s) swing {}".format(num_requested_rx_samples /self.usrp_rf_tx_rate, nSequences_per_period,  self.commonChannelParameter['integration_period_duration'], self.swingManager.activeSwing))


        self.nsamples_per_sequence     = pulse_sequence_period * self.usrp_rf_tx_rate

        # then calculate sample indicies at which pulse sequences start within a pulse sequence
        nPulses_per_sequence           = self.commonChannelParameter['npulses_per_sequence']
        pulse_sequence_offsets_samples = self.commonChannelParameter['pulse_sequence_offsets_vector'] * self.usrp_rf_tx_rate

        # then, calculate sample indicies at which pulses start within an integration period (all possible for cuda, only trasmitted for usrp_driver) 
        # TODO if the pulse offsets change in one scan, this has to be changed (calc next pulse period for cuda, current for usrp driver)
        all_possible_integration_period_pulse_sample_offsets = np.zeros(nPulses_per_sequence *  nSequences_per_period_max, dtype=np.uint64)
        for iSequence in range(nSequences_per_period_max):
            for iPulse in range(nPulses_per_sequence):
                all_possible_integration_period_pulse_sample_offsets[iSequence * nPulses_per_sequence + iPulse] = np.round(iSequence * self.nsamples_per_sequence + pulse_sequence_offsets_samples[iPulse])
        integration_period_pulse_sample_offsets = all_possible_integration_period_pulse_sample_offsets[:nSequences_per_period*nPulses_per_sequence]

        self.all_possible_integration_period_pulse_sample_offsets = all_possible_integration_period_pulse_sample_offsets 
        self.nPulses_per_integration_period = nPulses_per_sequence * nSequences_per_period
     
        if True:
           self.logger.debug(" > nSamples_per_sequence (bb) {} ".format(nSamples_per_sequence ))
           self.logger.debug(" > nSamples_per_sequence (if) {} ".format(nSamples_per_sequence_if ))
           self.logger.debug(" > num_requested_rx_samples {}".format(num_requested_rx_samples ))
           self.logger.debug(" > nSequences_per_period {} ".format(nSequences_per_period ))

        # inform all channels with the number of pulses per integration period        
        for ch in self.channels+newChannels:
            ch.nSequences_per_period                 = nSequences_per_period
            ch.nrf_rx_samples_per_integration_period = num_requested_rx_samples
            ch.nbb_rx_samples_per_sequence           = nSamples_per_sequence  
            assert abs(ch.nbb_rx_samples_per_sequence - pulse_sequence_period * self.commonChannelParameter['baseband_samplerate']) < 1e4 / self.commonChannelParameter['baseband_samplerate'], 'pulse sequences lengths must be a multiple of the baseband sampling rate'
            ch.integration_period_pulse_sample_offsets = integration_period_pulse_sample_offsets
        self.nSequences_per_period = nSequences_per_period
            
        

    #@timeit
    def trigger_next_swing(self):
        self.trigger_next_function_running = True
        self.logger.debug('running RHM.trigger_next_swing()')
        self.integration_time_manager.started_trigger_next()
 ##       swingManager = self.swingManager
     
        self.apply_channel_scaling()
        
        self._calc_period_details()
        trigger_next_period = self.nSequences_per_period != 0 # don't triger if no time left

        nSamples_per_pulse = int(np.round((self.commonChannelParameter['pulseLength'] / 1e6 * self.usrp_rf_tx_rate) + 2 * (self.commonChannelParameter['tr_to_pulse_delay']/1e6 * self.usrp_rf_tx_rate)))
        for ch in self.channels:
            self.logger.debug('cnum {}: tfreq={}, rfreq={}, usrp_center={} rx_rate={}MHz '.format(ch.cnum, ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq'], self.mixingFreqManager.current_mixing_freq, self.usrp_rf_tx_rate/1e6))
            if not (ch.ctrlprm_struct.payload['tfreq'] == ch.ctrlprm_struct.payload['rfreq']):
                    self.logger.warning('tfreq (={}) != rfreq (={}) !'.format( ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq']))
        
        # look for one active channel
        transmittingChannelAvailable = False
        for tmpChannel in self.channels:
           if (tmpChannel is not None) and (not tmpChannel.scanManager.isLastPeriod): 
              channel = tmpChannel
              transmittingChannelAvailable = True
              break

        self.swingManager.nextSwingToTrigger = self.swingManager.processingSwing
        self.logger.debug("setting nextSwingToTrigger to swing {}".format(self.swingManager.nextSwingToTrigger))

        if transmittingChannelAvailable:
           if trigger_next_period:
              # USRP SETUP
              self.logger.debug('triggering period no {}, swing {}'.format(channel.scanManager.current_period + 1 - channel.scanManager.isPrePeriod, self.swingManager.activeSwing))
              self.logger.debug("start USRP_SETUP")
              if self.auto_collect_clrfrq_after_rx:
                  nSamples_clear_freq = self.clearFreqRawDataManager.number_of_samples 
                  nSamples_pause_before_autoclearfreq = int(PAUSE_TIME_BEFORE_AUTO_CLEAR_FREQ * self.usrp_rf_rx_rate)
                  collect_auto_clear_freq_samples = True
                  auto_clear_freq_meta_data = dict(sampling_rate=self.usrp_rf_rx_rate, center_freq=self.mixingFreqManager.current_mixing_freq*1000)
              else:
                  nSamples_clear_freq = 0
                  nSamples_pause_before_autoclearfreq = 0
                  collect_auto_clear_freq_samples = False
                  auto_clear_freq_meta_data = None

              cmd = usrp_setup_command(self.usrpManager.socks, self.mixingFreqManager.current_mixing_freq*1000, self.mixingFreqManager.current_mixing_freq*1000, self.usrp_rf_tx_rate, self.usrp_rf_rx_rate, \
                                       self.nPulses_per_integration_period,  channel.nrf_rx_samples_per_integration_period, nSamples_pause_before_autoclearfreq, nSamples_clear_freq, nSamples_per_pulse, channel.integration_period_pulse_sample_offsets, self.swingManager.activeSwing)
              cmd.transmit()
              self.usrpManager.eval_client_return(cmd)
              self.logger.debug("end USRP_SETUP")

              # wait if periods should be time synchronized  
              for tmpChannel in self.channels:
                 if (tmpChannel is not None) and (not tmpChannel.scanManager.isLastPeriod): 
                    tmpChannel.scanManager.wait_for_next_trigger()
 
 
              # USRP_TRIGGER
              self.logger.debug("start USRP_GET_TIME")
              cmd = usrp_get_time_command(self.usrpManager.socks[0]) # grab current usrp time from one usrp_driver 
              cmd.transmit()
             
              # TODO: tag time using a better source? this will have a few hundred microseconds of uncertainty
              # maybe measure offset between usrp time and computer clock time somewhere, then calculate from there
              usrp_time = cmd.recv_time(self.usrpManager.socks[0])
              cmd.client_return()
              self.logger.debug("end USRP_GET_TIME")

           usrp_integration_period_start_clock_time = time.time() + self.integration_time_manager.get_usrp_delay_time()
           nSamples_rx_requested_of_last_trigger = channel.nrf_rx_samples_per_integration_period
           if transmittingChannelAvailable and trigger_next_period and self.auto_collect_clrfrq_after_rx:
              auto_clear_freq_meta_data['record_time'] = usrp_integration_period_start_clock_time + (nSamples_rx_requested_of_last_trigger / self.usrp_rf_rx_rate)

           # calculate sequence times for control program
           n_sequence_times = max(self.nSequences_per_period,1) # at least one time to transmit to control program
           sequence_start_time_secs  = np.zeros(n_sequence_times, dtype=np.uint64)
           sequence_start_time_usecs = np.zeros(n_sequence_times, dtype=np.uint32)
           for iSequence in range(n_sequence_times):
               pulse_start_time = usrp_integration_period_start_clock_time + iSequence * self.nsamples_per_sequence / self.usrp_rf_rx_rate
               sequence_start_time_secs[iSequence]  = int(pulse_start_time) 
               sequence_start_time_usecs[iSequence] = int(( pulse_start_time - int(pulse_start_time) ) *1e6)

           # save data for returning results to control program
           resultDict = {}
           resultDict['sequence_start_time_secs']      = sequence_start_time_secs
           resultDict['sequence_start_time_usecs']     = sequence_start_time_usecs
           resultDict['number_of_samples']             = self.commonChannelParameter['number_of_samples']
           resultDict['nSequences_per_period']         = self.nSequences_per_period       
           resultDict['pulse_sequence_offsets_vector'] = self.commonChannelParameter['pulse_sequence_offsets_vector'] 
           resultDict['npulses_per_sequence']          = self.commonChannelParameter['npulses_per_sequence']
           resultDict['results_are_valid']             = True
           for channel in self.channels: 
              if (channel is not None) and (not channel.scanManager.isLastPeriod): 
                  resultDict['nbb_rx_samples_per_sequence'] = channel.nbb_rx_samples_per_sequence
                  resultDict['pulse_lens']                  = channel.pulse_lens    
                  channel.resultDict_list.insert(0,resultDict.copy())

           if trigger_next_period: 
              # broadcast the start of the next integration period to all usrp
              self.logger.debug('start USRP_TRIGGER')
              trigger_time = usrp_time + self.integration_time_manager.get_usrp_delay_time()
              cmd = usrp_trigger_pulse_command(self.usrpManager.socks, trigger_time, self.commonChannelParameter['tr_to_pulse_delay'], self.swingManager.activeSwing) 
              self.logger.debug('sending trigger pulse command for swing {}'.format(self.swingManager.activeSwing))
              cmd.transmit()
              self.logger.debug('current usrp time: {}, trigger time of: {}'.format(usrp_time, trigger_time))
           else:
              self.logger.info("No time left, not triggering this swing.")

           # set state of channel to CS_PROCESSING
           for ch in self.channels:
               if ch.active_state == CS_TRIGGER:
                  ch.active_state = CS_PROCESSING
                  ch.logger.debug("Changing active channel state from CS_TRIGGER to CS_PROCESSING (cnum: {}, swing {}, period {})".format(ch.cnum, self.swingManager.activeSwing, ch.scanManager.current_period))
           if trigger_next_period: 
              self.logger.debug('waiting for trigger return')
              returns = self.usrpManager.eval_client_return(cmd)

              if TRIGGER_BUSY in returns:
                  self.logger.error('could not trigger, usrp driver is busy')
                  pdb.set_trace()
              self.logger.debug('end USRP_TRIGGER')
        else:
           self.logger.debug('No tranmitting channles available. Skipping USRP_TRIGGER')

        allProcessingChannelStates = [ch.processing_state for ch in self.channels]
        if self.processing_swing_invalid: # TODO check if this works (in control program or data files)
           self.logger.warning("Last swing has been invalid. Preparing 0 sequences to transmit")
           for iChannel, channel in enumerate(self.channels):
              if channel.processing_state is CS_PROCESSING:
                 channel.update_ctrlprm_class("current")
                 channel.resultDict_list[-1]['ctrlprm_dataqueue'] = copy.deepcopy(channel.ctrlprm_struct.dataqueue)
                 channel.resultDict_list[-1]['results_are_valid'] = False
                 channel.resultDict_list[-1]['nSequences_per_period'] = 0
 
                 for item in channel.ctrlprm_struct.dataqueue:
                    if item.name == 'rbeam':
                       channel.logger.debug("saving dataqueue to resultDict (rbeam={})".format(item.data))

           # save BB samples if usrp live view is active
           if os.path.isfile("./bufferLiveData.flag"):
              self.logger.info("Buffering raw data to disk.")
              chResExportList = [ ch.resultDict_list[-1] for ch in self.channels if ch.processing_state == CS_PROCESSING]
              with open('tmpRawData.pkl', 'wb') as f:
                 pickle.dump([[], [], chResExportList, self.antenna_idx_list_main, self.antenna_idx_list_back],f,  pickle.HIGHEST_PROTOCOL)
              os.rename("tmpRawData.pkl", "liveRawData.pkl")
              os.remove("./bufferLiveData.flag")

           self.processing_swing_invalid = False
        else:
           if CS_PROCESSING in allProcessingChannelStates:
               # CUDA_GET_DATA
               self.logger.debug('start CUDA_GET_DATA')
               cmd = cuda_get_data_command(self.cudasocks, self.swingManager.processingSwing)
               cmd.transmit()

               nMainAntennas = len(self.antenna_idx_list_main)
               nBackAntennas = len(self.antenna_idx_list_back)
               main_samples = None


               for cudasock in self.cudasocks:
                   nAntennas = recv_dtype(cudasock, np.uint32)
                   for iChannel,channel in enumerate(self.channels):
                       if channel.processing_state == CS_PROCESSING:
                           channel.logger.debug("Receiving {} antennas for channel {}".format(nAntennas, channel.cnum))
                           transmit_dtype(cudasock, channel.cnum, np.int32)

                           for iAntenna in range(nAntennas):
                               antIdx = recv_dtype(cudasock, np.uint16)
                               nSamples_bb = int(recv_dtype(cudasock, np.uint32) / 2)
                               # self.logger.debug("Receiving {} bb samples. (Channel {}, ant idx {})".format(nSamples_bb, channel.cnum, antIdx))
                               if main_samples is None:
                                  main_samples = np.zeros((len(self.channels), nMainAntennas, nSamples_bb), dtype=np.complex64)
                                  back_samples = np.zeros((len(self.channels), nBackAntennas, nSamples_bb), dtype=np.complex64)

                               samples = recv_dtype(cudasock, np.float32, nSamples_bb * 2)
                               samples = (samples[0::2] + 1j * samples[1::2]) * self.scaling_factor_rx_bb # unpacked interleaved i/q
                               
                               if antIdx in self.antenna_idx_list_main:
                                   iAntenna = self.antenna_idx_list_main.index(antIdx)
                                   main_samples[iChannel][iAntenna] = samples[:]
                               elif antIdx in self.antenna_idx_list_back:
                                   iAntenna = self.antenna_idx_list_back.index(antIdx)
                                   back_samples[iChannel][iAntenna] = samples[:]
                               else:
                                   self.logger.error("Cuda tranmitted antenna ({}) that is not in main array list ({}) and back array list ({}). (Maybe differnt antenna definietions in usrp_config.ini on both computers?)".format(antIdx, self.antenna_idx_list_main, self.antenna_idx_list_back))
                                   sys.exit(1)
                       else:
                          channel.logger.debug("Receiving NOTHING for channel {} because processing_state is {}".format( channel.cnum, channel.processing_state))
                    
             
                   transmit_dtype(cudasock, -1, np.int32) # to end transfer process
                   
               cmd.client_return()
               self.logger.debug('end CUDA_GET_DATA')


               # BEAMFORMING
               self.logger.debug('start rx beamforming')
               antenna_scale_factors = self.calc_normalize_and_mute_factors( main_samples, back_samples)
               
               beamformed_main_samples, beamformed_back_samples = self.calc_beamforming( main_samples, back_samples, antenna_scale_factors)
               for iChannel, channel in enumerate(self.channels):
                  if channel.processing_state == CS_PROCESSING:
                     # copy samples and ctrlprm to transmit later to control program
                     if 'main_beamformed' in channel.resultDict_list[-1]:
                        channel.logger.error("Main beamformed data already exist. Overwriting it. This is not correct. GetDataHandler too slow??")
                     channel.resultDict_list[-1]['main_beamformed'] = beamformed_main_samples[iChannel]
                     channel.resultDict_list[-1]['back_beamformed'] = beamformed_back_samples[iChannel]
                     channel.update_ctrlprm_class("current")
                     channel.resultDict_list[-1]['ctrlprm_dataqueue'] = copy.deepcopy(channel.ctrlprm_struct.dataqueue)

                     
                     channel.raw_export_data['beam']  = channel.ctrlprm_struct.payload['rbeam'] 
                     channel.raw_export_data['rfreq'] = channel.ctrlprm_struct.payload['rfreq']

                     for item in channel.ctrlprm_struct.dataqueue:
                        if item.name == 'rbeam':
                           channel.logger.debug("saving dataqueue to resultDict (rbeam={})".format(item.data))
     
               # save BB samples to file
                     if os.path.isfile("./save.raw.bb"):
                           self.logger.info("Saving raw bb data to disk.")
             #            for iChannel, channel in enumerate(self.channels):
                           channel.bb_export = dict()
                           channel.bb_export["main_samples"] = main_samples[iChannel] 
                           channel.bb_export["back_samples"] = back_samples[iChannel] 
                           channel.bb_export["nMainAntennas"] = nMainAntennas 
                           channel.bb_export["nBackAntennas"] = nBackAntennas 
                           channel.bb_export['number_of_samples'] = channel.resultDict_list[-1]['number_of_samples']  
                           channel.bb_export['number_of_sequences'] = channel.resultDict_list[-1]['npulses_per_sequence']
                           channel.bb_export["antenna_list"] = self.antenna_idx_list_main + self.antenna_idx_list_back
                           channel.bb_export['nSamples'] = nSamples_bb

                           channel.bb_export['nSequences_per_period'] = channel.resultDict_list[-1]['nSequences_per_period']
                           channel.bb_export['sequence_start_time_secs'] = channel.resultDict_list[-1]['sequence_start_time_secs']
                           channel.bb_export['sequence_start_time_usecs'] = channel.resultDict_list[-1]['sequence_start_time_usecs']
                           channel.bb_export['nbb_rx_samples_per_sequence']  = channel.resultDict_list[-1]['nbb_rx_samples_per_sequence']



                           channel.write_bb_data()
               self.logger.debug('end rx beamforming')

               # save BB samples if usrp live view is active
               if os.path.isfile("./bufferLiveData.flag"):
                  self.logger.info("Buffering raw data to disk.")
                  chResExportList = [ ch.resultDict_list[-1] for ch in self.channels if ch.processing_state == CS_PROCESSING]
                  with open('tmpRawData.pkl', 'wb') as f:
                     pickle.dump([main_samples, back_samples,chResExportList, self.antenna_idx_list_main, self.antenna_idx_list_back],f,  pickle.HIGHEST_PROTOCOL)
                  os.rename("tmpRawData.pkl", "liveRawData.pkl")
                  os.remove("./bufferLiveData.flag")


               # save IF raw data
               for channel in self.channels:
                  if channel.processing_state == CS_PROCESSING and  os.path.isfile("/collect.if.{:c}".format(97+channel.cnum)):
                     channel.logger.warning("Channel {} saving raw IF samples.".format(channel.cnum))
                     channel.get_if_data()
                     channel.write_if_data()
                     
           else:
              self.logger.debug('No processing channles available. Skipping CUDA_GET_DATA and rx beamforming')


        # PERIOD FINISHED        
        self.next_period_RHM()
        self.clearFreqRawDataManager.reset_occupied_freqs()

        # update (next) states
        for iChannel, channel in enumerate(self.channels):
           if channel.processing_state is CS_PROCESSING:

              # determine next state after samples are returned to control program
              if channel.scanManager.isPostLast: # or channel.scanManager.isForelastPeriod:
                 channel.next_processing_state = CS_INACTIVE
                 channel.active_state          = CS_INACTIVE
                 channel.next_active_state     = CS_INACTIVE
                 # self.nRegisteredChannels -= 1 
                 channel.logger.debug('last period finished, setting active and next processing state to CS_INACTIVE')
              elif channel.scanManager.isLastPeriod:
                 channel.next_processing_state = CS_LAST_SWING
              else:
                 channel.next_processing_state = CS_READY 
              channel.logger.debug("Switching next processing state (swing {}) of cnum {} to {}".format(self.swingManager.processingSwing, channel.cnum, channel.next_processing_state )) 

              channel.logger.debug("Switching processing state (swing {}) state of cnum {} from CS_PROCESSING to CS_SAMPLES_READY".format(self.swingManager.processingSwing, channel.cnum )) 
              channel.processing_state = CS_SAMPLES_READY
 
        # CUDA_ADD & CUDA_GENGERATE for processingSwing 
        for channel in self.channels:
            if channel.scanManager.isLastPeriod: # or channel.scanManager.isForelastPeriod:
               self.logger.debug("start CUDA_REMOVE_CHANNEL")
               cmd = cuda_remove_channel_command(self.cudasocks, sequence=channel.get_current_sequence(remove_channel=True), swing = self.swingManager.processingSwing) 
               self.logger.debug('send CUDA_REMOVE_CHANNEL (cnum {}, swing {})'.format(channel.cnum, self.swingManager.processingSwing))
               cmd.transmit()
               cmd.client_return()      
               self.logger.debug("end CUDA_REMOVE_CHANNEL")
               if channel.scanManager.isPostLast:

                  if channel in self.channels:
                     self.channels.remove(channel)
                  if channel in self.active_channels:
                     self.active_channels.remove(channel)

            else:
               if channel.active:
                  self.logger.debug("start CUDA_ADD_CHANNEL")
                  cmd = cuda_add_channel_command(self.cudasocks, sequence=channel.get_next_sequence(), swing = self.swingManager.processingSwing) 
                  self.logger.debug('send CUDA_ADD_CHANNEL (cnum {}, swing {}, beam {})'.format(channel.cnum, self.swingManager.processingSwing, channel.scanManager.current_beam))
                  cmd.transmit()
                  cmd.client_return()      
                  self.logger.debug("end CUDA_ADD_CHANNEL")
   
                  if channel.processing_state == CS_INACTIVE: # first use of swing 1
                      channel.processing_state = CS_READY
                      channel.logger.debug("Switching processing state (swing {}) state of cnum {} to CS_READY (first use of swing)".format(self.swingManager.processingSwing, channel.cnum )) 
               else:
                  self.logger.debug("ch {}: channel not active => not calling CUDA_ADD".format(channel.cnum))
                  self.logger.error("When is this happening and is this okay???")
 

        # CUDA_GENERATE for next period
        self.logger.debug('start CUDA_GENERATE_PULSE')
        cmd = cuda_generate_pulse_command(self.cudasocks, self.swingManager.processingSwing, self.mixingFreqManager.current_mixing_freq*1000)
        cmd.transmit()
        cmd.client_return()
        self.logger.debug('end CUDA_GENERATE_PULSE')

        if transmittingChannelAvailable and trigger_next_period: 
           # USRP_READY_DATA for activeSwing 
           self.logger.debug('start USRP_READY_DATA')
           cmd = usrp_ready_data_command(self.usrpManager.socks, self.swingManager.activeSwing)
           cmd.transmit()
    
           # check status of usrp drivers
           self.logger.debug('start receiving all USRP status')
           payloadList = self.usrpManager.eval_client_return(cmd, fcn=cmd.receive_all_metadata)
           self.logger.debug('end receiving all USRP status')
       
           all_usrps_report_failure = True
           antenna_list_offset = 0
           for iUSRP, ready_return in enumerate(payloadList):
               if ready_return == CONNECTION_ERROR:
                  self.usrpManager.fault_status[iUSRP] = True
                  self.logger.error('connection to USRP broke in GET_DATA')
                  antenna_list_offset += 1
               else: 
                  rx_status                = ready_return['status']
                  if rx_status < 0:
                     rx_error_codes = dict(ERROR_CODE_NONE = 0x0 , ERROR_CODE_TIMEOUT = 0x1, ERROR_CODE_LATE_COMMAND = 0x2, ERROR_CODE_BROKEN_CHAIN = 0x4, ERROR_CODE_OVERFLOW = 0x8, ERROR_CODE_ALIGNMENT = 0xc, ERROR_CODE_BAD_PACKET = 0xf, WRONG_NUMBER_OF_SAMPLES = 100)
                     
                     error_code = - rx_status
                     print_name = 'unknown'
                     if error_code % 1000 in rx_error_codes.values():
                         for err_name, err_value in rx_error_codes.items():
                             if err_value == (error_code % 1000):
                                 print_name = "UHD::" + err_name
                                 break
                     # out of sequence flag adds (-) 1000 to error code
                     if error_code >= 1000:  
                         print_name += " and out_of_sequence=1"
                     self.logger.error("Error: {}  (code {}) occurred in rx_worker for antennas {}. ".format(print_name, rx_status, self.usrpManager.antennaList_active[iUSRP-antenna_list_offset]))

                  else:
                     all_usrps_report_failure = False

                  self.usrpManager.fault_status[iUSRP] = ready_return["fault"]
   
                  self.logger.debug('GET_DATA rx status {}'.format(rx_status))
                  if rx_status != 2:
                      self.logger.error('USRP driver status {} in GET_DATA'.format(rx_status))
                      #status = USRP_DRIVER_ERROR # TODO: understand what is an error here..
           self.usrpManager.watchdog(all_usrps_report_failure)
              
           self.logger.debug('start waiting for USRP_DATA return')
           self.usrpManager.eval_client_return(cmd)
           self.logger.debug('end waiting for USRP_DATA return')

           self.logger.debug('end USRP_READY_DATA')


        # SWITCH SWINGS
        self.swingManager.switch_swings()
        self.logger.debug('switching swings to: active={}, processing={}'.format(self.swingManager.activeSwing, self.swingManager.processingSwing))
 
        # START CUDA_PROCESS
        if transmittingChannelAvailable:
           if trigger_next_period:
              # CUDA_PROCESS for processingSwing
              self.logger.debug('start CUDA_PROCESS')
              cmd = cuda_process_command(self.cudasocks, swing=self.swingManager.processingSwing, nSamples=nSamples_rx_requested_of_last_trigger)
              cmd.transmit()
              cmd.client_return()
              self.logger.debug('end CUDA_PROCESS')

           # repeat CLR_FREQ record for 2nd period (if executed for 1st)
           if self.clearFreqRawDataManager.repeat_request_for_2nd_period:
              self.logger.debug("Setting outstanding_request for CLR_FREQ for 2nd period.")
              self.clearFreqRawDataManager.repeat_request_for_2nd_period = False
              self.clearFreqRawDataManager.outstanding_request = True
              
           # automatic trigger of second period (without ROS:SET_READY)
           for channel in  self.channels:
              if channel.scanManager.isFirstPeriod: 
                 channel.logger.debug('setting active state (cnum {}, swing {}) to CS_TRIGGER to start second period'.format(channel.cnum, self.swingManager.activeSwing))
                 channel.active_state = CS_TRIGGER
                 channel.triggered_swing_list.insert(0, self.swingManager.nextSwingToTrigger)

                 channel.scanManager.isFirstPeriod = False

        # GET AUTO CLEAR FREQ DATA
        if transmittingChannelAvailable and trigger_next_period and self.auto_collect_clrfrq_after_rx:
            cmd = usrp_get_auto_clear_freq_command(self.usrpManager.socks)
            cmd.transmit()
            antenna_list, clr_samples = cmd.recv_all()
            self.clearFreqRawDataManager.update_auto_clear_freq_data(antenna_list, clr_samples, auto_clear_freq_meta_data)
            cmd.client_return()


        if not trigger_next_period:
           self.logger.info("This swing has not been triggered, setting processing_swing_invalid.")
           self.processing_swing_invalid = True # set for next call of trigger_next_period 

        self.trigger_next_function_running = False
        

    def next_period_RHM(self):
        ch_do_not_increase_period = []
        for ch in self.channels:
            if ch is not None:
                  ch_do_not_increase_period.append(ch.scanManager.isPrePeriod or ch.scanManager.isLastPeriod)
                  ch.scanManager.period_finished()
   
    def apply_channel_scaling(self, nChannelsWillBeAdded=0):

        # self.gain_control_divide_by_nChannels(self, nChannelsWillBeAdded)
        self.no_channel_gain_control()


    def no_channel_gain_control(self, nChannelsWillBeAdded=0):
        self.logger.debug("No channel scaling. Global factor for all channels is: totalScaligFactor  = {} ".format(self.scaling_factor_tx_total ))
        for ch in self.channels + self.newChannelList:
            ch.channelScalingFactor = self.scaling_factor_tx_total


    def gain_control_divide_by_nChannels(self, nChannelsWillBeAdded=0):
        nChannels = len(self.channels) + nChannelsWillBeAdded
        self.logger.debug("Setting channel scaling factor to: totalScaligFactor / nChannels = {}/ {} ".format(self.scaling_factor_tx_total, nChannels))
        for ch in self.channels + self.newChannelList:
            ch.channelScalingFactor = self.scaling_factor_tx_total
#            ch.channelScalingFactor = 1 / nChannels * self.scaling_factor_tx_total

    # normalize
    def calc_normalize_and_mute_factors(RHM, main_samples, back_samples):
        antenna_scale_factors = np.ones(max( RHM.antenna_idx_list_main + RHM.antenna_idx_list_back)+1)
        for ant_to_mute in RHM.mute_antenna_list:
            antenna_scale_factors[ant_to_mute] = 0
        antenna_scale_factors = [antenna_scale_factors for i in range(len(RHM.channels))]

        if RHM.apply_normalization:
            RHM.logger.info("start normalizing rx samples")
            debugPlot = False
            nChannels, nAntennas_main, nSamples = main_samples.shape
            nAntennas_back = back_samples.shape[1]
            bb_samplingRate = RHM.commonChannelParameter['baseband_samplerate'] 
            offset = int(np.round(900e-6*bb_samplingRate))
            channel = RHM.channels[0]
            pulse_offsets = RHM.all_possible_integration_period_pulse_sample_offsets
            pulse_offsets = np.array(np.round(pulse_offsets/RHM.usrp_rf_rx_rate*bb_samplingRate), dtype=np.int32)
            pulse_offsets -= pulse_offsets[0]
            rx_idx = []
            for iPulse in range(len(pulse_offsets)-1):
                rx_idx += range(pulse_offsets[iPulse]+offset, pulse_offsets[iPulse+1]-offset)
            rx_idx += range(pulse_offsets[iPulse+1]+offset, nSamples)
            rx_idx = np.array(rx_idx, dtype=np.int32)
            rx_idx = rx_idx[rx_idx<nSamples-1]
            if debugPlot:
                import matplotlib.pyplot as plt

 
            for iChannel,channel in enumerate(RHM.channels):
                var_list = []
                for iAntenna in range(nAntennas_main):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[iAntenna]]:
                        curr_variance =   np.var(np.real(main_samples[iChannel][iAntenna][rx_idx]))
                    else: # don't calculated if antenna is muted 
                        curr_variance = 1

                    var_list.append(curr_variance)
                    if debugPlot and iAntenna < 8:
                        plt.subplot(8,2,iAntenna*2+1)
                        plt.plot(20*np.log10(np.abs(main_samples[iChannel][iAntenna])/2**0))
                        plt.title("var = {:2.3f}".format(curr_variance))

                # back array
                for iAntenna in range(nAntennas_back):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[iAntenna]]:
                        curr_variance =   np.var(np.real(back_samples[iChannel][iAntenna][rx_idx]))
                    else: # don't calculated if antenna is muted 
                        curr_variance = 1
                    var_list.append(curr_variance)

                max_var= max(var_list)
                var_threshold = max_var * 10 ** (-30/10)
                RHM.logger.info("max var = {:2.3f} = {:2.3f} **2, var_threshold = {} (-30 dB) ".format(max_var,  np.sqrt(max_var), var_threshold))
                for iAntenna in range(nAntennas_main):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[iAntenna]]:
                        if var_list[iAntenna] > var_threshold:
                            scale_factor = np.sqrt(max_var/var_list[iAntenna]) * antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[iAntenna]]


                            antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[iAntenna]]  =  scale_factor
                            RHM.logger.info("scaling antenna {} with factor {:}".format(RHM.antenna_idx_list_main[iAntenna], scale_factor))
                        else:
                            RHM.logger.info("not scaling antenna {} because of small variance: {} (< threshold)".format(RHM.antenna_idx_list_main[iAntenna], var_list[iAntenna]))
                            scale_factor = 1
                    else:
                        RHM.logger.info("muting antenna {} (defined in array_config.ini)".format(RHM.antenna_idx_list_main[iAntenna], var_list[iAntenna]))

                    if debugPlot and iAntenna < 8:
                        plt.subplot(8,2,iAntenna*2+2)
                        plt.plot(20*np.log10(np.abs(main_samples[iChannel][iAntenna])/2**0))
                        plt.title("factor: {:2.3f}".format(scale_factor))
                if debugPlot:
                   plt.show()      

                # back array
                for iAntenna in range(nAntennas_back):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[iAntenna]]:
                        if var_list[iAntenna+nAntennas_main] > var_threshold:
                            scale_factor = np.sqrt(max_var/var_list[iAntenna+nAntennas_main]) * antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[iAntenna]]

                            antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[iAntenna]]  =  scale_factor
                            RHM.logger.info("scaling antenna {} with factor {:}".format(RHM.antenna_idx_list_back[iAntenna], scale_factor))
                        else:
                            RHM.logger.info("not scaling antenna {} because of small variance: {} (< threshold)".format(RHM.antenna_idx_list_back[iAntenna], var_list[iAntenna+nAntennas_main]))
                            scale_factor = 1
                    else:
                        RHM.logger.info("muting antenna {} (defined in array_config.ini)".format(RHM.antenna_idx_list_back[iAntenna], var_list[iAntenna+nAntennas_main]))


#                print("list var: {}".format(np.sqrt(var_list)))
            RHM.logger.info("end normalizing rx samples")

        return antenna_scale_factors


 
    # BEAMFORMING
    def calc_beamforming(RHM, main_samples, back_samples, antenna_scale_factors):
        nSamples = main_samples.shape[2]
        beamformed_main_samples = np.zeros((len(RHM.channels), nSamples), dtype=np.uint32)
        beamformed_back_samples = np.zeros((len(RHM.channels), nSamples), dtype=np.uint32)
        debugPlot = False

        maxInt16value = np.iinfo(np.int16).max # +32767
        minInt16value = np.iinfo(np.int16).min # -32768
    
        for iChannel, channel in enumerate(RHM.channels):
            if channel.processing_state is CS_PROCESSING:
                cur_beam=channel.ctrlprm_struct.payload['rbeam']
                cur_freq=channel.ctrlprm_struct.payload['rfreq']
                # bmazm         = calc_beam_azm_rad(RHM.array_nBeams, channel.scanManager.current_beam, RHM.array_beam_sep)    # calculate beam azimuth from transmit beam number          
                bmazm         = calc_beam_azm_rad(RHM.array_nBeams, cur_beam, RHM.array_beam_sep)    # calculate beam azimuth from transmit beam number          
                channel.logger.debug("rx beamforming: ch {}, beam {}".format(channel.cnum, channel.scanManager.current_beam))
                clrFreqResult = channel.scanManager.get_current_clearFreq_result()
                # pshift        = calc_phase_increment(bmazm, clrFreqResult[0] * 1000., RHM.array_x_spacing)       # calculate antenna-to-antenna phase shift for steering at a frequency        
                pshift        = calc_phase_increment(bmazm, cur_freq * 1000., RHM.array_x_spacing)       # calculate antenna-to-antenna phase shift for steering at a frequency        
                channel.logger.debug("rx beamforming: ch {}, frequency {}".format(channel.cnum, cur_freq, clrFreqResult[0]))


                
                # MAIN ARRAY
                first_pol_ant_idx = [ant_idx for ant_idx in RHM.antenna_idx_list_main if ant_idx < 20]
                first_pol_matrix_idx = [RHM.antenna_idx_list_main.index(ant_idx) for ant_idx in first_pol_ant_idx] 
                phasing_matrix = np.matrix([rad_to_rect(ant_idx * pshift)*antenna_scale_factors[iChannel][ant_idx] for ant_idx in first_pol_ant_idx])  # calculate a complex number representing the phase shift for each antenna
                print("main")
                print(phasing_matrix)
                s_matrix = np.matrix([antenna_scale_factors[iChannel][ant_idx] for ant_idx in first_pol_ant_idx])  # calculate a complex number representing the phase shift for each antenna
                print(s_matrix)
                complex_float_samples = phasing_matrix * np.matrix(main_samples[iChannel])[first_pol_matrix_idx,:] 
                real_mat = np.real(complex_float_samples)
                imag_mat = np.imag(complex_float_samples)
                abs_max_value = max(abs(real_mat).max(),  abs(imag_mat).max())
                RHM.logger.info("Abs max_value is {} (int16_max= {}, max_value / int16_max = {} ) ".format(abs_max_value, maxInt16value, abs_max_value / maxInt16value ))                

                real_mx = np.max(np.abs(real_mat))
                imag_mx = np.max(np.abs(imag_mat))
                if (real_mx > maxInt16value) or (imag_mx > maxInt16value ):
                   OverflowError("calc_beamforming: overflow error in casting data to complex int")
                   scale_value = maxInt16value/np.max([real_mx,imag_mx])
                   real_mat = scale_value*real_mat
                   imag_mat = scale_value*imag_mat

                # # check for clipping
                # if (real_mat > maxInt16value).any() or (real_mat < minInt16value).any() or (imag_mat > maxInt16value).any() or (imag_mat < minInt16value).any():
                #    RHM.logger.info("Overflow error while casting beamformed rx samples to complex int16s.")
        
                #    OverflowError("calc_beamforming: overflow error in casting data to complex int")
                #    real_mat = np.clip(real_mat, minInt16value, maxInt16value)
                #    imag_mat = np.clip(imag_mat, minInt16value, maxInt16value)

                complexInt32_pack_mat = (np.uint32(np.int16(real_mat)) << 16) + np.uint16(imag_mat) 
                beamformed_main_samples[iChannel] = complexInt32_pack_mat.tolist()[0]

                if debugPlot:
                   import matplotlib.pyplot as plt
                  # plt.figure()
                  # plt.plot(np.transpose(main_samples[iChannel]))
                  # plt.title("raw")
                   plt.figure()
                   plt.subplot(2,1,1)
                   plt.plot(real_mat.tolist()[0])
                   plt.plot(imag_mat.tolist()[0])
                   plt.title("Main array")

                # BACK ARRAY (same as middle of main array, ant 16 = ant 6, ...)
                phasing_matrix = np.matrix([rad_to_rect((ant_idx-10) * pshift)* antenna_scale_factors[iChannel][ant_idx] for ant_idx in RHM.antenna_idx_list_back])  # calculate a complex number representing the phase shift for each antenna
                s_matrix = np.matrix([antenna_scale_factors[iChannel][ant_idx] for ant_idx in RHM.antenna_idx_list_back])  # calculate a complex number representing the phase shift for each antenna
                print("back")
                print(phasing_matrix)
                print(s_matrix)

                complex_float_samples = phasing_matrix * np.matrix(back_samples[iChannel]) 
                real_mat = np.real(complex_float_samples)
                imag_mat = np.imag(complex_float_samples)
                if (real_mat > maxInt16value).any() or (real_mat < minInt16value).any() or (imag_mat > maxInt16value).any() or (imag_mat < minInt16value).any():
                   RHM.logger.error("Overflow error while casting beamformed rx samples to complex int16s.")
                   OverflowError("calc_beamforming: overflow error in casting data to complex int")
                   real_mat = np.clip(real_mat, minInt16value, maxInt16value)
                   imag_mat = np.clip(imag_mat, minInt16value, maxInt16value)
                complexInt32_pack_mat = (np.uint32(np.int16(real_mat)) << 16) + np.int16(imag_mat) 
                beamformed_back_samples[iChannel] = complexInt32_pack_mat.tolist()[0]
                if debugPlot:
                   import matplotlib.pyplot as plt
                   plt.subplot(2,1,2)
                   plt.plot(real_mat.tolist()[0])
                   plt.plot(imag_mat.tolist()[0])
                   plt.title("Back array")
                   plt.show()


        return beamformed_main_samples, beamformed_back_samples


class RadarChannelHandler:
    def __init__(self, conn, parent_RadarHardwareManager):
        self.conn = conn
        self.update_channel = True # flag indicating a new beam or pulse sequence 
        self.parent_RadarHardwareManager = parent_RadarHardwareManager
        self.logger = logging.getLogger("ChManager #{}".format(parent_RadarHardwareManager.channel_manager_consecutive_number ))
        self.state      = [CS_INACTIVE, CS_INACTIVE]
        self.next_state = [CS_INACTIVE, CS_INACTIVE]

        self.ctrlprm_struct = ctrlprm_struct(self.conn)
        self.seqprm_struct  = seqprm_struct(self.conn)
        self.clrfreq_struct = clrfreqprm_struct(self.conn)
        self.rprm_struct    = rprm_struct(self.conn)
        self.dataprm_struct = dataprm_struct(self.conn)

        self.channelScalingFactor = 0
        self.cnum = 'unknown'
        self.resultDict_list = []

        self.scanManager  = scanManager(read_restrict_file(RESTRICT_FILE), self)
  ###      self.scanManager.get_clr_freq_raw_data = self.parent_RadarHardwareManager.clearFreqRawDataManager.get_raw_data
        self.swingManager = parent_RadarHardwareManager.swingManager # reference to global swingManager of RadarHardwareManager
        self.triggered_swing_list = []

        self.received_first_SETPAR = False # to handle first GET_PAR before SET_PAR
        

# QUICK ACCESS TO CURRENT/NEXT ACTIVE/PROCESSING STATE
    @property
    def active_state(self):
        return self.state[self.swingManager.activeSwing]
    @active_state.setter
    def active_state(self,value):
        self.state[self.swingManager.activeSwing] = value
    @property
    def processing_state(self):
        return self.state[self.swingManager.processingSwing]
    @processing_state.setter
    def processing_state(self, value):
        self.state[self.swingManager.processingSwing] = value
    @property
    def next_active_state(self):
        return self.next_state[self.swingManager.activeSwing]
    @next_active_state.setter
    def next_active_state(self,value ):
        self.next_state[self.swingManager.activeSwing] = value
    @property
    def next_processing_state(self):
        return self.next_state[self.swingManager.processingSwing]
    @next_processing_state.setter
    def next_processing_state(self, value):
        self.next_state[self.swingManager.processingSwing] = value 
 
    def run(self):
        rmsg_handlers = {\
            SET_RADAR_CHAN       : self.SetRadarChanHandler,\
            SET_INACTIVE         : self.SetInactiveHandler,\
            SET_ACTIVE           : self.SetActiveHandler,\
            QUERY_INI_SETTINGS   : self.QueryIniSettingsHandler,\
        #   GET_SITE_SETTINGS    : self.GetSiteSettingsHandler, \
        #   UPDATE_SITE_SETTINGS : self.UpdateSiteSettingsHandler,\
            GET_PARAMETERS       : self.GetParametersHandler,\
            SET_PARAMETERS       : self.SetParametersHandler,\
            SET_PARAMETERS_I     : self.SetParametersHandler,\
            PING                 : self.PingHandler,\
        #   OKAY                 : self.OkayHandler,\
        #   NOOP                 : self.NoopHandler,\
            QUIT                 : self.QuitHandler,\
            ExitServer           : self.ExitServerHandler,\
            REGISTER_SEQ         : self.RegisterSeqHandler,\
        #   REMOVE_SEQ           : self.RemoveSeqHandler,\
            REQUEST_ASSIGNED_FREQ: self.RequestAssignedFreqHandler,\
            REQUEST_CLEAR_FREQ_SEARCH: self.RequestClearFreqSearchHandler,\
            LINK_RADAR_CHAN      : self.LinkRadarChanHandler,\
            SET_READY_FLAG       : self.SetReadyFlagHandler,\
            UNSET_READY_FLAG     : self.UnsetReadyFlagHandler,\
        #   SET_PROCESSING_FLAG  : self.SetProcessingFlagHandler,\
        #   UNSET_PROCESSING_FLAG: self.UnsetProcessingFlagHandler,\
        #   WAIT_FOR_DATA        : self.WaitForDataHandler,\
            GET_DATA             : self.GetDataHandler}


        while True:
            rmsg = rosmsg_command(self.conn)
            status = RMSG_FAILURE

            self.logger.debug('ch {}: waiting for command'.format(self.cnum))
            rmsg.receive(self.conn)
            command = chr(rmsg.payload['type'] & 0xFF) # for some reason, libtst is sending out 4 byte commands with junk..
            try:
                self.logger.debug('ch {}: received command (ROS=>USRP_Server): {}, {}'.format(self.cnum, command, RMSG_COMMAND_NAMES[command]))
            except KeyError:
                self.logger.error(command)
                self.logger.error('unrecognized command! {}'.format(rmsg.payload))
                self.close()
                break
                  

            try:
               if command in rmsg_handlers:
                   status = rmsg_handlers[command](rmsg)
               else:
                   status = self.DefaultHandler(rmsg)
            except:
                self.logger.error('ch {}: Error while command {} ({}). Removing this channel'.format(self.cnum,  RMSG_COMMAND_NAMES[command], command))
                self.logger.error("Error: {}".format(sys.exc_info()[0]))
                print(sys.exc_info()[0])
                raise
                self.close()
                break

            if status == 'exit_channel': # output of QuitHandler
                break
 
            if status == 'exit_server': # output of ExitServer
                sys.exit(0)
 
            rmsg.set_data('status', status)
            rmsg.set_data('type', rmsg.payload['type'])
            rmsg.transmit()


    def close(self):
        self.conn.close()
        self.logger.debug('Deleting channel {}'.format(self.cnum))
        RHM = self.parent_RadarHardwareManager
        RHM.unregister_channel_from_HardwareManager(self)
        cnum = self.cnum
        del self # TODO close thread ?!?
        RHM.logger.info('Deleted channel {}.'.format(cnum))


    # busy wait until state enters desired state
    # useful for waiting for
    def _waitForState(self, swing, state):
        if type(state) is not list:
            state = [state]

        wait_start = time.time()
        counter = 0
        while self.state[swing] not in state:
            counter = (counter + 1) % 10000
#            if state[0] == CS_SAMPLES_READY and counter == 1:
            if counter == 1:
               self.logger.debug("ch {}:_waitForState {}. state is {} (swing {})".format(self.cnum, state, self.state[swing], swing))
            time.sleep(RADAR_STATE_TIME)
            if time.time() - wait_start > CHANNEL_STATE_TIMEOUT:
                self.logger.error('CHANNEL STATE TIMEOUT for channel {}'.format(self.cnum))
                self.close()
                break
    
    def update_ctrlprm_class(self, period):
        if len(self.scanManager.scan_beam_list) == 0:
           self.logger.warning("scan_beam_list is empty. unable to update ctrlprm_class!")
           return

        if period == "current":
           beam = self.scanManager.current_beam
           freq = self.scanManager.get_current_clearFreq_result()[0]
        elif period == "next":
           beam = self.scanManager.next_beam
           freq = self.scanManager.get_next_clearFreq_result()[0]
        else:
            self.logger.error("unknown period specifier: {} (valid: current or next )".format(period)) 
       
        parNameList  = ['rbeam', 'tbeam', 'rfreq', 'tfreq'] 
        parValueList = [ beam  ,  beam  ,  freq  ,  freq  ]
     
        for iPar, par in enumerate(parNameList):
           self.ctrlprm_struct.set_data(par, parValueList[iPar])
           self.ctrlprm_struct.payload[par] = parValueList[iPar]

    # return a sequence object, used for passing pulse sequence and channel infomation over to the CUDA driver
    def get_current_sequence(self,remove_channel=False):
        if not remove_channel:
            self.update_ctrlprm_class('current')
            self.logger.debug("Getting current sequence with {} samples (305x1500x {}) rbeam {}".format(self.nrf_rx_samples_per_integration_period, self.nrf_rx_samples_per_integration_period/305/1500, self.ctrlprm_struct.payload['rbeam']))

        seq = sequence(self.npulses_per_sequence,  self.tr_to_pulse_delay, self.parent_RadarHardwareManager.all_possible_integration_period_pulse_sample_offsets, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor,  self.ctrlprm_struct.payload )

       # seq = sequence(self.npulses_per_sequence,  self.tr_to_pulse_delay, self.pulse_sequence_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor,  self.ctrlprm_struct.payload )
        return seq

    def get_next_sequence(self):
        self.update_ctrlprm_class('next')
        seq = sequence(self.npulses_per_sequence,  self.tr_to_pulse_delay, self.parent_RadarHardwareManager.all_possible_integration_period_pulse_sample_offsets, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor,  self.ctrlprm_struct.payload )
        return seq

    def DefaultHandler(self, rmsg):
        self.logger.error("Unexpected command: {}".format(chr(rmsg.payload['type'])))
        pdb.set_trace()
        return RMSG_FAILURE

    def ExitServerHandler(self, rmsg):
        self.logger.debug("Entering Exit Server Handler...")
        self.parent_RadarHardwareManager.exit()
        return 'exit_server'

    def QuitHandler(self, rmsg):
        self.logger.debug("Entering Quit handler...")
        # TODO: close down stuff cleanly
        #rmsg.set_data('status', RMSG_FAILURE)
        #rmsg.set_data('type', rmsg.payload['type'])
        #rmsg.transmit()
        self.close()
        return 'exit_channel'

    def PingHandler(self, rmsg):
        return RMSG_SUCCESS
    
    #@timeit
    def RequestAssignedFreqHandler(self, rmsg):
        # wait for clear frequency search to end, hardware manager will set channel state to WAIT
        # self._waitForState(STATE_WAIT) 
        clrFreqResult = self.scanManager.get_current_clearFreq_result()

        transmit_dtype(self.conn, clrFreqResult[0], np.int32)
        transmit_dtype(self.conn, clrFreqResult[1], np.float32)

        self.logger.info('ch {}: clr frequency search raw data age: {:2.1f} s'.format(self.cnum, time.time() - clrFreqResult[2]))
        return RMSG_SUCCESS

    #@timeit
    def RequestClearFreqSearchHandler(self, rmsg):
        self.clrfreq_struct.receive(self.conn)
        if self.scanManager.fixFreq <= 0:
            # set request flat from RadarHardwareManager:clearFreqRawDatamanager
            self.parent_RadarHardwareManager.clearFreqRawDataManager.outstanding_request = True
            self.logger.debug("RequestClearFreqSearchHandler: setting request CLR_FREQ flag in clearFreqRawDataManager (caused by ch {})".format(self.cnum))
        else:
            self.logger.debug("RequestClearFreqSearchHandler: ignoring because of fixfreq ( ch {})".format(self.cnum))

        return RMSG_SUCCESS

    def UnsetReadyFlagHandler(self, rmsg):
        return RMSG_SUCCESS


    def SetReadyFlagHandler(self, rmsg):
        # ROS calls it ready, we call it trigger
        self.logger.debug("ch {}: SetReadyFlagHandler: waiting for nextSwingToTrigger (swing {}) become  CS_READY or CS_LAST_SWING".format(self.cnum, self.swingManager.nextSwingToTrigger))
        self._waitForState(self.swingManager.nextSwingToTrigger, [CS_READY, CS_LAST_SWING])
 #       transmit_dtype(self.conn, self.nSequences_per_period, np.uint32) # TODO mgu transmit here nSeq ?     
        self.logger.debug("ch {}: SetReadyFlagHandler: setting nextSwingToTrigger state (swing {}) to CS_TRIGGER".format(self.cnum, self.swingManager.nextSwingToTrigger))
        self.state[self.swingManager.nextSwingToTrigger] = CS_TRIGGER
        self.triggered_swing_list.insert(0, self.swingManager.nextSwingToTrigger)
        # send trigger command
        return RMSG_SUCCESS
    
    #@timeit
    def RegisterSeqHandler(self, rmsg):
        # function to get the indexes of rising edges going from zero to a nonzero value in array ar

        self.logger.debug('Entering RegisterSeqHandler for channel {}'.format(self.cnum))
        def _rising_edge_idx(ar):
            ar = np.insert(ar, 0, -2)
            edges = np.array([ar[i+1] * (ar[i+1] - ar[i] > 1) for i in range(len(ar)-1)])
            return edges[edges > 0]

        # returns the run length of a pulse in array ar starting at index idx
        def _pulse_len(ar, idx):
            runlen = 0
            for element in ar[idx:]:
                if not element:
                    break
                runlen += 1
            return runlen
        # site libraries appear to not initialize the status, so a nonzero status here is normall.

        self.seqprm_struct.receive(self.conn)
        self.seq_rep = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
        self.seq_code = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
    
        self.logger.debug('RegisterSeqHandler, received sequence data from control program')
        intsc = recv_dtype(self.conn, np.int32)
        intus = recv_dtype(self.conn, np.int32)

        self.raw_export_data = dict()
        self.raw_export_data['nrang'] = recv_dtype(self.conn, np.int32)
        self.raw_export_data['mpinc'] = recv_dtype(self.conn, np.int32)
        self.raw_export_data['smsep'] = recv_dtype(self.conn, np.int32)
        self.raw_export_data['lagfr'] = recv_dtype(self.conn, np.int32)
        self.raw_export_data['mppul'] = recv_dtype(self.conn, np.int32)
        pat = []
        for iBaudCode in range(self.raw_export_data['mppul']):
           pat.append(recv_dtype(self.conn, np.int32))
        self.raw_export_data['ppat'] = pat

        self.raw_export_data['nbaud'] = recv_dtype(self.conn, np.int32)
        pcode = []
        for iBaudCode in range(self.raw_export_data['nbaud']):
           pcode.append(recv_dtype(self.conn, np.int32))
        self.raw_export_data['pcode'] = pcode

#        print(self.raw_export_data)
      
        self.logger.debug('RegisterSeqHandler, received intsc: {}, intus: {}'.format(intsc, intus))
        self.integration_period_duration = intsc + (intus / 1e6)

        tx_tsg_idx = self.seqprm_struct.get_data('index')
        tx_tsg_len = self.seqprm_struct.get_data('len')
        tx_tsg_step = self.seqprm_struct.get_data('step')
        
        # ratio between tsg step (units of microseconds) to baseband sampling period
        # TODO: calculate this from TXUpsampleRate, FSampTX in cuda_config.ini
        # however, it should always be 1..
        tsg_bb_per_step = 1

        # psuedo-run length encoded tsg
        tx_tsg_rep = self.seq_rep
        tx_tsg_code = self.seq_code

        seq_buf = []
        for i in range(tx_tsg_len):
            for j in range(0, np.int32(tsg_bb_per_step * tx_tsg_rep[i])):
                seq_buf.append(tx_tsg_code[i])
        seq_buf = np.uint8(np.array(seq_buf))

        # extract out pulse information...
        S_BIT = np.uint8(0x01) # sample impulses
        R_BIT = np.uint8(0x02) # tr gate, use for tx pulse times
        X_BIT = np.uint8(0x04) # transmit path, use for bb
        A_BIT = np.uint8(0x08) # enable attenuator
        P_BIT = np.uint8(0x10) # phase code (BPSK)

        # create masks
        samples    = seq_buf & S_BIT
        tr_window  = seq_buf & R_BIT
        rf_pulse   = seq_buf & X_BIT
        atten      = seq_buf & A_BIT
        phase_mask = (seq_buf & P_BIT) >> int(np.log2(P_BIT))

        # extract and number of samples
        sample_idx = np.nonzero(samples)[0]
        assert len(sample_idx) > 3, 'register empty sequence'

        nbb_samples = len(sample_idx)

        # extract pulse start timing
        tr_window_idx = np.nonzero(tr_window)[0]
        tr_rising_edge_idx = _rising_edge_idx(tr_window_idx)
        pulse_sequence_offsets_vector = tr_rising_edge_idx * tx_tsg_step

        # extract tr window to rf pulse delay
        rf_pulse_idx = np.nonzero(rf_pulse)[0]
        rf_pulse_edge_idx = _rising_edge_idx(rf_pulse_idx)
        tr_to_pulse_delay = (rf_pulse_edge_idx[0] - tr_rising_edge_idx[0]) * tx_tsg_step
        npulses_per_sequence = len(rf_pulse_edge_idx)

        # extract per-pulse phase coding and transmit pulse masks
        # indexes are in microseconds from start of pulse
        phase_masks = []
        pulse_masks = []
        pulse_lens = []

        for i in range(npulses_per_sequence):
            pstart = rf_pulse_edge_idx[i]
            pend = pstart + _pulse_len(rf_pulse, pstart)
            phase_masks.append(phase_mask[pstart:pend])
            pulse_masks.append(rf_pulse[pstart:pend])
            pulse_lens.append((pend - pstart) * tx_tsg_step)

        self.npulses_per_sequence = npulses_per_sequence
        self.pulse_sequence_offsets_vector = pulse_sequence_offsets_vector / 1e6
        self.pulse_lens = pulse_lens # length of pulses, in seconds
        self.phase_masks = phase_masks # phase masks are complex number to multiply phase by, so
        self.pulse_masks = pulse_masks
        self.tr_to_pulse_delay = tr_to_pulse_delay
        self.tx_time = self.pulse_lens[0] + 2 * self.tr_to_pulse_delay

      #  print("phase masks (len {}) (len(phase_masks[0]) = {}):".format(len(phase_masks), len(phase_masks[0])))
      #  print(phase_masks)
      #  print("pulse masks (len {}):".format(len(pulse_masks)))
      #  print(pulse_masks)

        self.logger.debug("pulse0 length: {} us, tr_pulse_delay: {} us, tx_time: {} us".format(self.pulse_lens[0], tr_to_pulse_delay,  self.pulse_lens[0] + 2 * self.tr_to_pulse_delay))
        if npulses_per_sequence == 0:
            raise ValueError('number of pulses per sequence must be greater than zero!')
        if nbb_samples == 0:
            raise ValueError('number of samples in sequence must be nonzero!')

        return RMSG_SUCCESS
    def get_if_data(channel):
      RHM = channel.parent_RadarHardwareManager
      # CUDA_GET_IF_DATA
      channel.logger.debug('start CUDA_GET_IF_DATA')
      cmd = cuda_get_if_data_command(RHM.cudasocks, RHM.swingManager.processingSwing)
      cmd.transmit()

      if_samples = None
      all_antenna_list = RHM.antenna_idx_list_main + RHM.antenna_idx_list_back
      for cudasock in RHM.cudasocks:
          nAntennas = recv_dtype(cudasock, np.uint32)

          if channel.processing_state == CS_PROCESSING:
              transmit_dtype(cudasock, channel.cnum, np.int32)

              for iAntenna in range(nAntennas):
                  antIdx = recv_dtype(cudasock, np.uint16)
                  nSamples_if = int(recv_dtype(cudasock, np.uint32) )
                  channel.logger.debug("Receiving {} if samples.".format(nSamples_if))
                  if if_samples is None:
                     if_samples = np.zeros(( nAntennas, nSamples_if), dtype=np.float32)

                  samples = recv_dtype(cudasock, np.float32, nSamples_if )
#                  samples = samples[0::2] + 1j * samples[1::2] # TODO change to match export format. i/q int32 ????
                  
                  # TODO add back array
                  iAntenna = all_antenna_list.index(antIdx)
                  if_samples[iAntenna] = samples[:]
      
          transmit_dtype(cudasock, -1, np.int32) # to end transfer process
                   
          cmd.client_return()
          channel.raw_export_data['data'] = if_samples * RHM.scaling_factor_rx_if
          channel.raw_export_data['nAntennas'] = nAntennas
          channel.raw_export_data['nSamples'] = nSamples_if
          channel.logger.debug('end CUDA_GET_IF_DATA')


    def write_bb_data(channel):
        channel.logger.debug('start saving BB samples')
        time_now = datetime.datetime.utcnow()
        version = 3 
        hardwareManager = channel.parent_RadarHardwareManager
        
        savePath = "/data/image_samples/bb_data"
        if not os.path.isdir(savePath):
            os.mkdir(savePath)
                    
        fileName = '{:04d}{:02d}{:02d}{:02d}{:02d}.{:d}.iraw.{:c}'.format(time_now.year, time_now.month, time_now.day, time_now.hour, time_now.minute, channel.rnum, 96+channel.cnum)

        exportList = []
        exportList = []
        exportList.append( version )
        exportList.append( time_now.year )
        exportList.append( time_now.month )
        exportList.append( time_now.day )
        exportList.append( time_now.hour )
        exportList.append( time_now.minute )
        exportList.append( time_now.second )
        exportList.append( time_now.microsecond *1000 )
        exportList.append( channel.raw_export_data['nrang'])
        exportList.append( channel.raw_export_data['mpinc'])
        exportList.append( channel.raw_export_data['smsep'])
        exportList.append( channel.raw_export_data['lagfr'])
        exportList.append( hardwareManager.commonChannelParameter['pulseLength'])  # in micro sec
        exportList.append( channel.raw_export_data['beam'])
        exportList.append( channel.raw_export_data['rfreq']) # in kHz
        exportList.append( channel.raw_export_data['mppul'])
        exportList +=  channel.raw_export_data['ppat']
        exportList.append( channel.raw_export_data['nbaud'])
        exportList +=  channel.raw_export_data['pcode']
#       exportList.append(channel.bb_export['nSamples']) 
        exportList.append(channel.bb_export['number_of_samples']) 
        exportList.append(channel.bb_export['nSequences_per_period']) 
        exportList.append(channel.bb_export['nMainAntennas'] + channel.bb_export['nBackAntennas'])
        exportList += channel.bb_export["antenna_list"]

        # 
        float_main_data = np.complex64(channel.bb_export["main_samples"])
        float_back_data = np.complex64(channel.bb_export["back_samples"])

        export_samples = b''
        for iSequence in range(channel.bb_export['nSequences_per_period']):
            export_samples += np.uint32(channel.bb_export['sequence_start_time_secs'][iSequence]).tobytes()
            export_samples += np.uint32(channel.bb_export['sequence_start_time_usecs'][iSequence]).tobytes()

            pulse_sequence_start_index = iSequence * channel.bb_export['nbb_rx_samples_per_sequence']
            pulse_sequence_end_index = pulse_sequence_start_index + channel.bb_export['number_of_samples']
            for iAntenna in range(channel.bb_export['nMainAntennas']):
                   export_samples += float_main_data[iAntenna][pulse_sequence_start_index:pulse_sequence_end_index].tobytes()
            for iAntenna in range(channel.bb_export['nBackAntennas']):
                   export_samples += float_back_data[iAntenna][pulse_sequence_start_index:pulse_sequence_end_index].tobytes()
      

        rawFile = open(os.path.join(savePath, fileName), "ba")
        rawFile.write(np.array(exportList, dtype=np.int32))
        rawFile.write(export_samples)
        rawFile.close()
        channel.logger.debug('end saving BB samples')


    def write_if_data(channel):
        channel.logger.debug('start saving IF samples')
        time_now = datetime.datetime.now()
        version = 2 
        RECV_SAMPLE_HEADER = 0 # TODO is this an offset???
        hardwareManager = channel.parent_RadarHardwareManager
        
        savePath = "/data/image_samples/if_data"
        if not os.path.isdir(savePath):
            os.mkdir(savePath)
                    

        fileName = '{:04d}{:02d}{:02d}{:02d}{:02d}.{:d}.iraw.{:c}'.format(time_now.year, time_now.month, time_now.day, time_now.hour, time_now.minute, channel.rnum, 96+channel.cnum)


        exportList = []
        exportList.append( version )
        exportList.append( time_now.year )
        exportList.append( time_now.month )
        exportList.append( time_now.day )
        exportList.append( time_now.hour )
        exportList.append( time_now.minute )
        exportList.append( time_now.second )
        exportList.append( time_now.microsecond *1000 )
        exportList.append( channel.raw_export_data['nrang'])
        exportList.append( channel.raw_export_data['mpinc'])
        exportList.append( channel.raw_export_data['smsep'])
        exportList.append( channel.raw_export_data['lagfr'])
        exportList.append( hardwareManager.commonChannelParameter['pulseLength'])  # in micro sec
        exportList.append( channel.raw_export_data['beam'])
        exportList.append( channel.raw_export_data['rfreq']) # in kHz
        exportList.append( channel.raw_export_data['mppul'])
        exportList +=  channel.raw_export_data['ppat']
        exportList.append( channel.raw_export_data['nbaud'])
        exportList +=  channel.raw_export_data['pcode']
        exportList.append( RECV_SAMPLE_HEADER)
        exportList.append(channel.raw_export_data['nSamples']) 
        exportList.append(channel.raw_export_data['nAntennas'])
        print(exportList)
        for iAntenna in range(channel.raw_export_data['nAntennas']):
            exportList += channel.raw_export_data['data'][iAntenna].tolist()

        rawFile = open(os.path.join(savePath, fileName), "ba")
        rawFile.write(np.array(exportList, dtype=np.int32))

        print(exportList)

        for iAntenna in range(channel.oversample_export_data['nAntennas']):
            channel.oversample_export_data['data'][iAntenna].tofile(rawFile)

        rawFile.close()
        time_end = datetime.datetime.now()

        channel.logger.debug('end saving IF samples (it took {} seconds)'.format(str(time_end - time_now)))

    
    # receive a ctrlprm struct
    #@timeit
    def SetParametersHandler(self, rmsg):
        # TODO: check if new freq is possible with usrp_centerFreq
        # TODO divide compatibiliti check in sequence and ctrlprm check?
        # TODO add compatibility check in parameter prediction function
        self.received_first_SETPAR = True
        RHM = self.parent_RadarHardwareManager
        RHM.n_SetParameterHandlers_active += 1

        if self.scanManager.isPrePeriod:
            current_swing = self.swingManager.nextSwingToTrigger
        else:
            # swing with this parameters has already been triggered, just compare parameter
            current_swing = 1 - self.swingManager.nextSwingToTrigger

        if self.scanManager.isInitSetParameter:
           self.scanManager.isInitSetParameter = False
           self.ctrlprm_struct.receive(self.conn)
           self.logger.debug("ch {}: Received from ROS for swing {} (init SetPar is only stored): tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.cnum, current_swing, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))
           RHM.n_SetParameterHandlers_active -= 1
           return RMSG_SUCCESS


#        self.ctrlprm_struct.receive(self.conn)
        self.logger.debug("ch {}: Received from ROS SetParameter for swing {} : tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.cnum, current_swing, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))
        self.logger.debug("swing state {}".format(self.state[current_swing]))
        # wait if RHM.trigger_next_swing() is slower... 
        self._waitForState(current_swing, [CS_INACTIVE, CS_PROCESSING, CS_LAST_SWING])   

        # period not jet triggered
        if self.state[current_swing] == CS_INACTIVE:# or self.active_state == CS_READY:#  not needed with change of site.c

           self.logger.debug("Ch {} waiting for Parameter semaphore...".format(self.cnum)) 
           RHM.set_par_semaphore.acquire()
           self.logger.debug("Ch {} acquired semaphore, setting parameter".format(self.cnum)) 
           
           if self.state[current_swing] == CS_READY:
              self.logger.debug("Channel already initialized, but not triggered, Reinitializing it...")
              self.state[current_swing] = CS_INACTIVE
              
           self.ctrlprm_struct.receive(self.conn)
           self.logger.debug("ch {}: Received from ROS: tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.cnum, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))
           
           if not self.CheckChannelCompatibility(): # TODO  for two swings and reset after transmit?
              self.logger.debug("CheckChannelCompatability FAIL")
              RHM.n_SetParameterHandlers_active -= 1
              return RMSG_FAILURE
              
           if self not in self.parent_RadarHardwareManager.newChannelList:
              self.parent_RadarHardwareManager.newChannelList.append(self)
              self.logger.debug("Adding ch {} to newChannelList ".format(self.cnum))
           else:
              self.logger.debug("Ch {} already in newChannelList ".format(self.cnum))

           RHM.set_par_semaphore.release()
           self.logger.debug("Ch {} released semaphore".format(self.cnum)) 
 
        # in middle of scan, period already triggerd. only compare with prediction
        elif self.state[current_swing] == CS_PROCESSING or self.state[current_swing] == CS_LAST_SWING: 
           # TODO something here is wrong: uafscan with --onesec has CS_LAST_SWING but --fast not
           self.update_ctrlprm_class("current")
           ctrlprm_old = copy.deepcopy(self.ctrlprm_struct.payload)
           
           # compare received with predicted parameter
           self.ctrlprm_struct.receive(self.conn)
           for key in ctrlprm_old.keys():
              if np.any(ctrlprm_old[key] != self.ctrlprm_struct.payload[key]):
                  self.logger.debug("ch {} rreceived new ctrl_prm {} ({}) old ctrl_prm ({})".format(self.cnum, key, self.ctrlprm_struct.payload[key], ctrlprm_old[key] ))
                  if key == "tfreq" and self.ctrlprm_struct.payload[key] == 12000: # control program always sends 2 SET_PAR. 1st one with tfreq 12MHz
                      continue
                  self.logger.error("ch {}: received ctrlprm_struct for {} ({}) is not equal with prediction ({})".format(self.cnum, key,self.ctrlprm_struct.payload[key], ctrlprm_old[key] ))
                  # TODO return RMSG_FAILURE
              #else:
               #  self.logger.debug("ch {}: received ctrlprm_struct for {} ({}) IS     equal with prediction ({})".format(self.cnum, key,self.ctrlprm_struct.payload[key], ctrlprm_old[key] ))
        else:
           self.logger.error("ROS:SetParameter: Active state is {} (current_swing={}, activeSwing={} ). Dont know what to do...".format(self.state[current_swing], self.swingManager.activeSwing,  self.active_state))
           self.logger.error("ROS:SetParameter: Exit usrp_server...")
           RHM.n_SetParameterHandlers_active -= 1
           return RMSG_FAILURE
           self.parent_RadarHardwareManager.exit()

        
        if (self.rnum < 0 or self.cnum < 0):
           self.logger.error("SET_PARAMETER: Invalid radar or channel number: rnum={}, cnum={}".format(self.rnum, self.cnum))
           RHM.n_SetParameterHandlers_active -= 1
           return RMSG_FAILURE

        RHM.n_SetParameterHandlers_active -= 1
        return RMSG_SUCCESS

    def CheckChannelCompatibility(self):
        self.logger.debug('checking channel compatibility for channel {}'.format(self.cnum))
        hardwareManager = self.parent_RadarHardwareManager
        commonParList_ctrl = ['number_of_samples', 'baseband_samplerate' ]
        commonParList_seq  = [ 'npulses_per_sequence', 'pulse_sequence_offsets_vector',  'tr_to_pulse_delay', 'integration_period_duration', 'tx_time']
        if all([self.pulse_lens[0]==self.pulse_lens[i] for i in range(1,len(self.pulse_lens))]):
            pulseLength = self.pulse_lens[0]
        else:
            self.logger.error("Pulse lengths in one sequence have to be the equal! ") # TODO raise error?
            pdb.set_trace()
            return False
        if hardwareManager.nRegisteredChannels == 1 and (len(hardwareManager.channels) == 0 or hardwareManager.channels[0] == self): 
           self.logger.info("Compatibility check: This channel is already registered at HardwareManager and is the only one. Renewing registration.")
           hardwareManager.nRegisteredChannels = 0

        if hardwareManager.nRegisteredChannels == 0:  # this is the first channel
            hardwareManager.commonChannelParameter = {key: getattr(self, key) for key in commonParList_seq}
            hardwareManager.commonChannelParameter.update( {key: self.ctrlprm_struct.payload[key] for key in commonParList_ctrl})
            hardwareManager.commonChannelParameter.update({'pulseLength':pulseLength})
            
            # upsampling rates
            #  it looks like tx_bb_samplingRate has to be 100 kHz for phase coding (but there is no documentation...)
            upsample_rate = hardwareManager.usrp_rf_tx_rate / 100000
            hardwareManager.commonChannelParameter.update({"upsample_rate":upsample_rate})
            self.logger.debug("Setting cuda upsampling rate to {}".format(upsample_rate))

            # determine downsample rates
            #   bb_samplinRate = 3e8/2/rsep
            #    rsep=45km => 3.333 kHz  (default)
            #    rsep=15km => 10 kHz    
            #    rsep=9km  => 16.667 kHz    
            #    rsep=6km  => 25 kHz     

            goodDownsampleRates = [[20, 75],  # 5M => 3.333k
                                   [20, 25],  # 5M => 10k 
                                   [15 ,20],  # 5M => 16.67k 
                                   [10 ,20],  # 5M => 25k 
                                   [10 ,75],  # 2.5M => 3.333k 
                                   [30 ,100], # 10M => 3.333k 
                                   [20, 50],  # 10M => 10k 
                                   [20, 30],  # 10M => 16.67k 
                                   [20 ,20],  # 10M => 25k 
            ]
         
            total_downsample_rate = hardwareManager.usrp_rf_rx_rate / hardwareManager.commonChannelParameter['baseband_samplerate']
            downSampleRates = None
            for rate in goodDownsampleRates:
               if np.abs(rate[0]*rate[1] - total_downsample_rate) < 0.01:
                  downSampleRates = rate
                  break
            if downSampleRates is None:
               errorMsg ="No downsample rates are defined for downsampling from {} to {}.".format(hardwareManager.usrp_rf_rx_rate, hardwareManager.commonChannelParameter['baseband_samplerate']) 
               self.logger.error(errorMsg)
               assert downSampleRates != None, errorMsg
            else:
               self.logger.debug("Setting cuda downsampling ratios to {} and {}".format(downSampleRates[0], downSampleRates[1]))
               hardwareManager.commonChannelParameter.update({"downsample_rates":downSampleRates})
               hardwareManager.send_cuda_setup_command()

           

            hardwareManager.nRegisteredChannels = 1
            return True

        else:   # not first channel => check if new parameters are compatible
            
            parCompatibleList_seq  = [hardwareManager.commonChannelParameter[parameter] == getattr(self, parameter) for parameter in commonParList_seq]
            parCompatibleList_ctrl = [hardwareManager.commonChannelParameter[parameter] == self.ctrlprm_struct.payload[parameter] for parameter in commonParList_ctrl]

            idxOffsetVec = commonParList_seq.index('pulse_sequence_offsets_vector')  # convert vector of bool to scalar
            parCompatibleList_seq[idxOffsetVec] = parCompatibleList_seq[idxOffsetVec].all()
 
         #   pdb.set_trace()
            if (not all(parCompatibleList_seq)) or (not  all(parCompatibleList_ctrl)) or (pulseLength != hardwareManager.commonChannelParameter['pulseLength']):
                self.logger.error('Unable to add new channel. Parameters not compatible with active channels.')
                for iPar,isCompatible in enumerate(parCompatibleList_seq):
                     if not all(isCompatible):
                        self.logger.error(" Not compatible sequence parameter: {}   old channel(s): {} , new channel: {}".format(commonParameterList_seq[iPar], hardwareManager.commonChannelParameter[commonParameterList_seq[iPar]] , getattr(self, commonParameterList_seq[iPar])))
                for iPar,isCompatible in enumerate(parCompatibleList_ctrl):
                     if not isCompatible:
                        self.logger.error(" Not compatible ctrlprm: {}   old channel(s): {} , new channel: {}".format(commonParameterList_ctrl[iPar], hardwareManager.commonChannelParameter[commonParameterList_ctrl[iPar]] , self.ctrlprm_struct.payload[commonParameterList_ctrl[iPar]]))
                if pulseLength != hardwareManager.commonChannelParameter['pulseLength']:
                    self.logger.error(" Pulse length of new channel ({}) is not compatible to old channel(s) ({})".format(pulseLength, hardwareManager.commonChannelParameter['pulseLength'])) 
                return False
            else:
                if self not in hardwareManager.channels:
                   hardwareManager.nRegisteredChannels += 1
                return True
                    

        # TODO change usrp_xx_cfreq somewhere if possible        
        assert np.abs((ch.ctrlprm_struct.payload['tfreq'] * 1e3) - self.mixingFreqManager.current_mixing_freq*1e6) < (self.usrp_rf_tx_rate / 2), 'transmit frequency outside range supported by sampling rate and center frequency'


    # send ctrlprm struct
    #@timeit
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
#        if self.received_first_SETPAR:
        self.update_ctrlprm_class("current")
        self.ctrlprm_struct.transmit()
        self.logger.debug("ch {}: sending current ctrlprm_struct (tfreq={}, rfreq={},tbeam={},rbeam={})".format(self.cnum, self.ctrlprm_struct.get_data('tfreq'), self.ctrlprm_struct.get_data('rfreq'), self.ctrlprm_struct.get_data('tbeam'), self.ctrlprm_struct.get_data('rbeam') ))
        return RMSG_SUCCESS
    
    #@timeit
    def GetDataHandler(self, rmsg):
        self.logger.debug('start channelHanlder:GetDataHandler ch: {}'.format(self.cnum))
        self.update_ctrlprm_class("current")
        self.dataprm_struct.set_data('samples', self.ctrlprm_struct.payload['number_of_samples'])

        self.dataprm_struct.transmit() # only 'samples' of dataprm is ever changed TODO check other parameter such as event_secs....
        self.logger.debug('ch {}: sending dprm struct'.format(self.cnum))

        if self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        # TODO investigate possible race conditions

        finishedSwing = self.triggered_swing_list.pop() 
        self.logger.debug('ch {}: channelHanlder:GetDataHandler waiting for channel to idle before GET_DATA (finished swing is {})'.format(self.cnum, finishedSwing))
        self.logger.debug("start waiting for CS_SAMPLES_READY")
        self._waitForState(finishedSwing, CS_SAMPLES_READY)
        self.logger.debug("end waiting for CS_SAMPLES_READY")

        self.logger.debug('ch {}: channelHanlder:GetDataHandler returning samples'.format(self.cnum))
#        transmit_dtype(self.conn, self.parent_RadarHardwareManager.resultData_nSequences_per_period, np.uint32)  
        self.send_results_to_control_program()

        self.logger.debug('ch {}: channelHanlder:GetDataHandler finished returning samples. setting state to {}  (swing {})'.format(self.cnum, self.next_state[finishedSwing], finishedSwing))
        self.state[finishedSwing] = self.next_state[finishedSwing]
        self.logger.debug('end channelHanlder:GetDataHandler ch: {}'.format(self.cnum))

        return RMSG_SUCCESS

    def send_results_to_control_program(self):
        # interact with site library's SiteIntegrate loop
        # send metadata for integration period
        # currently assuming pulse sequences are uniform within an integration period

        rd_shallow = self.resultDict_list[-1]
        resultDict = copy.deepcopy(self.resultDict_list.pop())

        transmit_dtype(self.conn, resultDict['nSequences_per_period'], np.uint32)  
        self.logger.debug("transmitting number of sequences in period: {}".format(resultDict['nSequences_per_period']))

        badtrdat_start_usec = resultDict['pulse_sequence_offsets_vector'] * 1e6 # convert to us
        transmit_dtype(self.conn, resultDict['npulses_per_sequence'], np.uint32)
        transmit_dtype(self.conn, badtrdat_start_usec,                np.uint32) # length badtrdat_len
        transmit_dtype(self.conn, resultDict['pulse_lens'],           np.uint32) # length badtrdat_len

        # stuff these with junk, they don't seem to be used..
        num_transmitters = self.parent_RadarHardwareManager.usrpManager.nUSRPs   # TODO update for polarization?
        txstatus_agc = self.parent_RadarHardwareManager.usrpManager.fault_status # TODO is this the right way to return fault status????
        txstatus_lowpwr = np.zeros(num_transmitters)
        if txstatus_agc.any():
            self.logger.warning('Following USRPs report Fault:  {} (usrp index)'.format([k for k in range(txstatus_agc.size) if txstatus_agc[k] != 0]))

        transmit_dtype(self.conn, num_transmitters, np.int32)
        transmit_dtype(self.conn, txstatus_agc,     np.int32) # length num_transmitters
        transmit_dtype(self.conn, txstatus_lowpwr,  np.int32) # length num_transmitters
       

        # transmitt trigger time
        time_struct = time.gmtime(resultDict['sequence_start_time_secs'][0])
        transmit_dtype(self.conn, time_struct.tm_year, np.int32)
        transmit_dtype(self.conn, time_struct.tm_mon, np.int32)
        transmit_dtype(self.conn, time_struct.tm_mday, np.int32)
        transmit_dtype(self.conn, time_struct.tm_hour, np.int32)
        transmit_dtype(self.conn, time_struct.tm_min, np.int32)
        transmit_dtype(self.conn, time_struct.tm_sec, np.int32)
        transmit_dtype(self.conn, resultDict['sequence_start_time_usecs'][0], np.int32)


        # print main info of sequence
        for item in resultDict['ctrlprm_dataqueue']:
            item.transmit(self.ctrlprm_struct.clients[0])
            if item.name in ["rbeam", "tbeam", "tfreq", "rfreq"]:
                   self.logger.debug("Sending to ROS: {}={}".format(item.name, item.data))
    
        # send back samples with pulse start times 
        self.logger.debug('GET_DATA returning samples for {} pulses'.format(resultDict['nSequences_per_period']))
    
        for iSequence in range(resultDict['nSequences_per_period']):
            #self.logger.debug('GET_DATA returning samples from pulse {}'.format(iSequence))
            
            #self.logger.debug('GET_DATA sending sequence start time')
            transmit_dtype(self.conn, resultDict['sequence_start_time_secs'][iSequence],  np.uint32)
            transmit_dtype(self.conn, resultDict['sequence_start_time_usecs'][iSequence], np.uint32)

            # calculate the baseband sample index for the start and end of a pulse sequence
            # within a block of beamformed samples over the entire integration period
            # assuming that the first sample is aligned with the center of the first transmit pulse
            # and all sequences within the integration period have the same length

            pulse_sequence_start_index = iSequence * resultDict['nbb_rx_samples_per_sequence']
            pulse_sequence_end_index = pulse_sequence_start_index + resultDict['number_of_samples']
            #self.logger.debug("Number of samples if {} (no deepcopy version is {}), main beamformed shape: {}".format(resultDict['number_of_samples'], rd_shallow['number_of_samples'], resultDict['main_beamformed'].shape))
            #self.logger.debug("start index: {}, end index: {}".format(pulse_sequence_start_index, pulse_sequence_end_index))
        
            # send the packed complex int16 samples to the control program.. 
            #self.logger.debug('GET_DATA sending main samples')
            transmit_dtype(self.conn, resultDict['main_beamformed'][pulse_sequence_start_index:pulse_sequence_end_index], np.uint32)

            #self.logger.debug('GET_DATA sending back samples')
            transmit_dtype(self.conn, resultDict['back_beamformed'][pulse_sequence_start_index:pulse_sequence_end_index], np.uint32)
            
            # wait for confirmation before sending the next antenna..
            # if we start catching this assert or timing out, maybe add some more error handling here
            #self.logger.debug('GET_DATA waiting on ack from site library')
            sample_send_status = recv_dtype(self.conn, np.int32)
            assert sample_send_status == iSequence 

        self.logger.warning('GET_DATA: sending back array samples, but not sure if this is correct!')


    
    #@timeit
    def SetRadarChanHandler(self, rmsg):
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)

        if self.cnum in [ch.cnum for ch in self.parent_RadarHardwareManager.channels if ch is not None and ch is not self]:
           self.logger.error("New channel (cnum {}) can not be added beause channel with this cnum already active.".format(self.cnum))
           return RMSG_FAILURE
        
        
        self.ctrlprm_struct.set_data('channel', self.cnum)
        self.ctrlprm_struct.set_data('radar',  self.rnum)

        # TODO: how to handle channel contention?
        # self.logger.name = "ChManager {}".format(self.cnum)
        self.logger.debug('radar num: {}, radar chan: {}'.format(self.rnum, self.cnum))

        # TODO: set RMSG_FAILURE if radar channel is unavailable
        # rmsg.set_data('status', RMSG_FAILURE)
        return RMSG_SUCCESS

    def LinkRadarChanHandler(self, rmsg):
        rnum = recv_dtype(self.conn, np.int32)
        cnum = recv_dtype(self.conn, np.int32)
        self.logger.error('link radar chan is unimplemented!')
        pdb.set_trace()
        return RMSG_SUCCESS


    def QueryIniSettingsHandler(self, rmsg):
        # TODO: don't hardcode this if I find anything other than ifmode querying..
        data_length = recv_dtype(self.conn, np.int32)
        ini_name = recv_dtype(self.conn, str, nitems=data_length)
        requested_type = recv_dtype(self.conn, np.uint8)

        # hardcode to reply with ifmode is false
        assert ini_name == b'site_settings:ifmode\x00'

        payload = 0 # assume always false

        transmit_dtype(self.conn, requested_type, np.uint8)
        transmit_dtype(self.conn, data_length, np.int32) # appears to be unused by site library
        transmit_dtype(self.conn, payload, np.int32)

        return 1 # TODO: Why does the ini handler expect a nonzero response for success?

    def SetActiveHandler(self, rmsg):
        # called by site library at the start of a scan 
        self.active = True
        if self not in self.parent_RadarHardwareManager.active_channels:
           self.parent_RadarHardwareManager.active_channels.append(self)
           self.logger.debug("Added ch {} to RHM.active_channels list".format(self.cnum))
           
        self.received_first_SETPAR = False
        self.logger.debug('SetActiveHandler starting')

        # receive all data from control program        
        scan_num_beams         = recv_dtype(self.conn, np.int32)
        fixFreq                = recv_dtype(self.conn, np.int32)
        clrfreq_start_list     = recv_dtype(self.conn, np.int32, nitems = scan_num_beams)
        clrfreq_bandwidth_list = recv_dtype(self.conn, np.int32, nitems = scan_num_beams)
        scan_beam_list         = recv_dtype(self.conn, np.int32, nitems = scan_num_beams)
        syncBeams              = recv_dtype(self.conn, np.int32)
        scan_time_sec          = recv_dtype(self.conn, np.int32)  
        scan_time_us           = recv_dtype(self.conn, np.int32) 
        integration_time_sec   = recv_dtype(self.conn, np.int32)  
        integration_time_us    = recv_dtype(self.conn, np.int32)  
        start_period           = recv_dtype(self.conn, np.int32)
 
        integration_time = integration_time_sec + integration_time_us/1e6
        scan_time = scan_time_sec + scan_time_us/1e6 

        if scan_num_beams == 1: # make sure this variables are list even for one beam per scan
           clrfreq_start_list = [clrfreq_start_list]
           clrfreq_bandwidth_list = [clrfreq_bandwidth_list]
           scan_beam_list = [scan_beam_list]

        self.logger.debug('SetActiveHandler number of beams per scan: {}'.format(scan_num_beams))
        self.logger.debug('SetActiveHandler fixFreq: {}'.format(fixFreq))
        self.logger.debug('SetActiveHandler clear frequency search start frequencies: {}'.format(clrfreq_start_list))
        self.logger.debug('SetActiveHandler clear frequency search bandwidths (Hz): {}'.format(clrfreq_bandwidth_list))
        self.logger.debug('SetActiveHandler scan beam list: {}'.format(scan_beam_list))
        self.logger.debug('SetActiveHandler scan_duration: {}'.format(scan_time))
        self.logger.debug('SetActiveHandler integration_duration: {}'.format(integration_time))
         
        if syncBeams == 1:
           scan_times_list = recv_dtype(self.conn, np.int32, nitems = scan_num_beams) / 1000
           self.logger.debug('SetActiveHandler scan_times_list: {}'.format(scan_times_list))
        else:
           scan_times_list  = None       
           self.logger.debug('SetActiveHandler: no time sync of beams')

        if self.scanManager.camping: 
            # if camping==True control progam uses SetActive for every new beam but no paramater should change
            # in this case skip reset of scanManager and swingManager to be able to use two swings parallel
            assert(fixFreq == self.scanManager.fixFreq)
            assert(scan_beam_list == self.scanManager.scan_beam_list)
            self.logger.info("SetActiveHandler: Skipping reset of scanManager and swingManager because nothing changed with camping beam.")
            return RMSG_SUCCESS 


        if self.parent_RadarHardwareManager.trigger_next_function_running:
           self.logger.debug('start SetActiveHandler: waiting for trigger_next() to finish')
           while self.parent_RadarHardwareManager.trigger_next_function_running:
              time.sleep(0.0005)
           self.logger.debug('end SetActiveHandler: waiting for trigger_next() to finish')

        freq_range_list = [[clrfreq_start_list[i], clrfreq_start_list[i] + clrfreq_bandwidth_list[i]] for i in range(scan_num_beams)]

        self.logger.debug('SetActiveHandler updating swingManager with new freq/beam lists')
        self.scanManager.init_new_scan(freq_range_list, scan_beam_list, fixFreq, scan_times_list, scan_time, integration_time, start_period)
        self.triggered_swing_list = []
  
        addFreqResult = self.parent_RadarHardwareManager.mixingFreqManager.add_new_freq_band(self)
    
        if addFreqResult == True:
            #self.swingManager.reset()
            #self.logger.debug("Resetting swing manager (active={}, processing={})".format(self.swingManager.activeSwing, self.swingManager.processingSwing ))
            return RMSG_SUCCESS
        elif addFreqResult == False:
            self.logger.error("Freq range of new channel (cnum {}) is not in USRP bandwidth. (freq_range_list[0][0] = {} ".format(self.cnum, freq_range_list[0][0]))
            self.scanManager.clear_freq_range_list = None 
            self.scan_beam_list = None
            self.fixFreq = None
            return RMSG_FAILURE
        else: # new mixing freq
            self.parent_RadarHardwareManager.send_cuda_setup_command()
            self.parent_RadarHardwareManager.clearFreqRawDataManager.center_freq = self.parent_RadarHardwareManager.mixingFreqManager.current_mixing_freq 
            self.parent_RadarHardwareManager.clearFreqRawDataManager.metaData['usrp_fcenter'] = self.parent_RadarHardwareManager.mixingFreqManager.current_mixing_freq 
            #self.swingManager.reset()
            #self.logger.debug("Resetting swing manager (active={}, processing={})".format(self.swingManager.activeSwing, self.swingManager.processingSwing ))
            return RMSG_SUCCESS
        
 
    def SetInactiveHandler(channelObject, rmsg):
        RHM = channelObject.parent_RadarHardwareManager

        if channelObject in RHM.active_channels:
            RHM.logger.info('ROS:SET_INACTVIVE removing channel {} from RHM.active_channels'.format(RHM.channels.index(channelObject)))
            RHM.active_channels.remove(channelObject)
           
        if channelObject in RHM.channels:
            RHM.logger.info('ROS:SET_INACTVIVE removing channel {} from HardwareManager'.format(RHM.channels.index(channelObject)))
            RHM.channels.remove(channelObject)

            RHM.nRegisteredChannels -= 1
            if RHM.nRegisteredChannels == 0: 
                RHM.logger.debug("No channels left, removing commonChannelParameter") 
                RHM.commonChannelParameter = {}

        channelObject.active = False
        # TODO: return failure status if the radar or channel number is invalid?
        return RMSG_SUCCESS


def main():
    # maybe switch to multiprocessing with manager process
    
    now = datetime.datetime.now()
    now_string = now.strftime("__%Y%m%d_%H%M%S")
    logging_usrp.initLogging('server' + now_string + '.log')
    logging.info('Strating main() of usrp_server')


    radar = RadarHardwareManager(RMSG_PORT)
    radar.run()


if __name__ == '__main__':
    main()
