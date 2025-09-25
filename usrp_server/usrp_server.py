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
# This branch is set up for two-radar operation at dual sites. Select the config/driver_config__two_radar.ini to be driver_config.ini

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
from collections import Counter

sys.path.insert(0, '../python_include')

from phasing_utils import *
from socket_utils import *
from rosmsg import *
from drivermsg_library import *
from radar_config_constants import *
from clear_frequency_search import record_clrfreq_raw_samples
from profiling_tools import *
import logging_usrp

radar_active=np.zeros(2,dtype=bool)

RMSG_PORT = 45000
USRP_SERVER_HOST = 'localhost'
#USRP_SERVER_HOST = '192.168.100.1'

MAX_CHANNELS = 4
USRP_BANDWIDTH_RESTRICTION = 300000 # in Hz. No channels allowed on both edges of the URSP bandwidth to avoid aliasing
USRP_SOCK_TIMEOUT = 7 # sec

RMSG_SUCCESS = 0
RMSG_FAILURE = -1

RADAR_STATE_TIME = .0001
CHANNEL_STATE_TIMEOUT = 120
# TODO: move this out to a config file
nSwings = 2

debug = True

DEFAULT_USRP_MIXING_FREQ = 13000

# channel states (CS) for each channel
CS_INACTIVE      = 'CS_INACTIVE'
CS_READY         = 'CS_READY'
CS_TRIGGER       = 'CS_TRIGGER'
CS_PROCESSING    = 'CS_PROCESSING'
CS_SAMPLES_READY = 'CS_SAMPLES_READY'
CS_LAST_SWING    = 'CS_LAST_SWING'


class integrationTimeManager():
   """ Estimates the time the integration period has to be reduced to be able to setup usrp copy samples etc"""
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
      if self.RHM.N_RADARs > 1:
         overhead_time = 0.50
      else:
         overhead_time = 0.30

      return overhead_time


class statusUpdater():
   " Class to a file every x minutes to allow checking usrp_status from outside"

   def __init__(self, RHM):
      self.fileName = '/data/log/usrp_server_status.txt'
      self.RHM = RHM
      self.nSeconds_update_period = 5
      self.last_write = datetime.datetime.now(datetime.UTC)
      self.str_start = self.last_write.strftime("Start time: %Y-%m-%d %H:%M:%S\n")


   def create_status_information(self):
      status = self.str_start
      status += self.last_write.strftime("Last time: %Y-%m-%d %H:%M:%S\n")
      for jrad in range(self.RHM.N_RADARs):
         status += "Radar {}: USRPs: {} active, {} inactive\n".format(jrad, len(self.RHM.usrpManager.addressList_active[jrad]), len(self.RHM.usrpManager.addressList_inactive[jrad]))
         status += "Radar {}: Channels: {} active (of {})\n".format(jrad, len(self.RHM.active_channels[jrad]), len(self.RHM.channels[jrad]))
      status += "Sequences per period: {}\n".format(self.RHM.nSequences_per_period)
      return status


   def update_advanced(self):
      """ Writes some information in the file"""

      nSeconds_since_last_write = (datetime.datetime.now(datetime.UTC) - self.last_write).total_seconds()
      if self.nSeconds_update_period < nSeconds_since_last_write:
         self.last_write = datetime.datetime.now(datetime.UTC)
#         if not os.path.isfile(self.fileName):
         with open(self.fileName, "w") as f:
            f.write(self.create_status_information())


   def update(self):
      """ just updates the empty file """
      nSeconds_since_last_write = (datetime.datetime.now(datetime.UTC) - self.last_write).total_seconds()
      if self.nSeconds_update_period < nSeconds_since_last_write:
         self.last_write = datetime.datetime.now(datetime.UTC)
#         if not os.path.isfile(self.fileName):
         with open(self.fileName, "w") as f:
            f.write("")            # create empty file
#    disadvantage this time is not the default time shown by ls
#         else:
#            print("updating time")
#            os.utime(self.fileName, None) # update the time


class usrpSockManager():
   def __init__(self, RHM):
      self.addressList_active     = [[] for jrad in range(RHM.N_RADARs)] # tuple of IP and port
      self.antennaList_active     = [[] for jrad in range(RHM.N_RADARs)]
      self.hostnameList_active    = [[] for jrad in range(RHM.N_RADARs)]
      self.driverHostnameList_active = [[] for jrad in range(RHM.N_RADARs)]
      self.addressList_inactive   = [[] for jrad in range(RHM.N_RADARs)]
      self.antennaList_inactive   = [[] for jrad in range(RHM.N_RADARs)]
      self.hostnameList_inactive  = [[] for jrad in range(RHM.N_RADARs)]
      self.driverHostnameList_inactive = [[] for jrad in range(RHM.N_RADARs)]

      self.socks = [[] for jrad in range(RHM.N_RADARs)]
      usrp_driver_base_port = int(RHM.ini_network_settings['USRPDriverPort'])
      self.RHM = RHM
      self.logger = logging.getLogger("usrpManager")

      self.nUSRPs = len(RHM.ini_usrp_configs) # TODO should this be all USRPs or only active?
      self.fault_status = np.zeros(self.nUSRPs)
      self.errors_in_a_row = 0
      self.error_limit = 15
      self.nSeconds_retry_reconnect = 60
      self.last_reconnection = datetime.datetime.now()

      # open each
      for usrpConfig in RHM.ini_usrp_configs:
         self.logger.debug("USRP {} {}".format(usrpConfig['usrp_hostname'],usrpConfig['array_idx']))
         jrad = int(usrpConfig["radar"])
         try:
            if usrpConfig['usrp_hostname'] in self.hostnameList_active[jrad]:
               self.logger.debug("Already connected to USRP {}".format(usrpConfig['usrp_hostname']))
               idx_usrp = self.hostnameList_active[jrad].index(usrpConfig['usrp_hostname'])
               self.antennaList_active[jrad][idx_usrp].append(usrpConfig['array_idx'])

            elif usrpConfig['usrp_hostname'] in self.hostnameList_inactive[jrad]:
               self.logger.debug("Already failed to connect to USRP {}".format(usrpConfig['usrp_hostname']))
               idx_usrp = self.hostnameList_inactive[jrad].index(usrpConfig['usrp_hostname'])
               self.antennaList_inactive[jrad][idx_usrp].append(usrpConfig['array_idx'])

            else:
               port = int(usrpConfig['usrp_hostname'].split(".")[2]) + usrp_driver_base_port
               connectPar = (usrpConfig['driver_hostname'], port)
               usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
               usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
               time.sleep(0.002)
               usrpsock.connect(connectPar)
               self.logger.debug('USRP server connection {}:{}'.format(usrpConfig['driver_hostname'], port))

               if USRP_SOCK_TIMEOUT != None:
                  usrpsock.settimeout(USRP_SOCK_TIMEOUT)
               self.socks[jrad].append(usrpsock)
               self.addressList_active[jrad].append(connectPar)
               self.antennaList_active[jrad].append([usrpConfig['array_idx']])
               self.hostnameList_active[jrad].append(usrpConfig['usrp_hostname'])
               self.driverHostnameList_active[jrad].append(usrpConfig['driver_hostname'])
               self.logger.debug('connected to usrp driver on port {}'.format(port))

         except ConnectionRefusedError:
            self.logger.error('USRP server connection failed: {}:{}'.format(usrpConfig['driver_hostname'], port))

            if usrpConfig['usrp_hostname'] in self.hostnameList_inactive[jrad]:
               idx_usrp = self.hostnameList_inactive[jrad].index(usrpConfig['usrp_hostname'])
               self.antennaList_inactive[jrad][idx_usrp].append(usrpConfig['array_idx'])
            else:
               self.addressList_inactive[jrad].append(connectPar)
               self.antennaList_inactive[jrad].append([usrpConfig['array_idx']])
               self.hostnameList_inactive[jrad].append(usrpConfig['usrp_hostname'])
               self.driverHostnameList_inactive[jrad].append(usrpConfig['driver_hostname'])

      SomeActiveUSRPs=False
      for jrad in range(RHM.N_RADARs):
         self.logger.debug("len socks[{}]: {}".format(jrad,len(self.socks[jrad])))
         if len(self.socks[jrad]) != 0:
            SomeActiveUSRPs = True

      if not SomeActiveUSRPs:
         self.logger.error("No connection to USRPs. Exit usrp_server.")
         RHM.exit()

      self.logger.debug("SomeActiveUSRPs {} so returning".format(SomeActiveUSRPs))


   def remove_sock(self, jrad, sock_to_remove):
       iSock = self.socks[jrad].index(sock_to_remove)
       self.logger.error("Removing usrp {} ({}:{}).".format(self.hostnameList_active[jrad][iSock], self.addressList_active[jrad][iSock][0], self.addressList_active[jrad][iSock][1]))
       self.addressList_inactive[jrad].append(self.addressList_active[jrad][iSock])
       del self.addressList_active[jrad][iSock]

       lost_antennas = self.antennaList_active[jrad][iSock]
       self.antennaList_inactive[jrad].append(lost_antennas)
       for iSwing in range(nSwings):
          self.fill_shm_with_zeros(lost_antennas, iSwing, ['rx', 'tx'])
       del self.antennaList_active[jrad][iSock]

       self.hostnameList_inactive[jrad].append(self.hostnameList_active[jrad][iSock])
       del self.hostnameList_active[jrad][iSock]

       self.driverHostnameList_inactive[jrad].append(self.driverHostnameList_active[jrad][iSock])
       del self.driverHostnameList_active[jrad][iSock]

       del self.socks[jrad][iSock]

       if hasattr(self.RHM, 'clearFreqRawDataManager'):
          self.RHM.clearFreqRawDataManager.set_usrp_driver_connections(jrad, self.socks[jrad])


   def get_all_main_antenna_socks(self, jrad):
       main_ant_sock_list = []
       for sock, ant in zip(self.socks[jrad], self.antennaList_active[jrad]):
           if ant not in [16, 17, 18, 19]:
               main_ant_sock_list.append(sock)
       return main_ant_sock_list


   def eval_client_return(self, cmd, jrad, fcn=None):
       if fcn is None: # default receive function
         client_return = cmd.client_return()
       else:
         client_return = fcn()

       if CONNECTION_ERROR in client_return:
         offset = 0
         for iSock, singleReturn in enumerate(client_return):
            if singleReturn == CONNECTION_ERROR:

               if jrad is None:
                  socks = np.concatenate(self.socks).tolist()
                  badsock=socks[iSock-offset]
                  for jjrad in range(self.RHM.N_RADARs):
                     if badsock in self.socks[jjrad]:
                        jrad = jjrad
                        index = self.socks[jrad].index(badsock)
               else:
                  index=iSock-offset

               self.logger.error("Connection lost to usrp {} ({}:{}). Removing it from sock list.".format(self.hostnameList_active[jrad][index], self.addressList_active[jrad][index][0], self.addressList_active[jrad][index][1]))

               self.remove_sock(jrad,self.socks[jrad][index])
               offset += 1

       SomeActiveUSRPs=False
       for jrad in range(self.RHM.N_RADARs):
         if len(self.socks[jrad]) != 0:
            SomeActiveUSRPs = True

       if not SomeActiveUSRPs:
         self.logger.error("No connection to USRPs. Exit usrp_server.")
         self.RHM.exit()

       return client_return


   def fill_shm_with_zeros(self, antenna_list, swing, direction_list):
       side = 0
       # direction_list = ['rx', 'tx']
       nInts_shm = int(self.RHM.ini_shm_settings['rxshm_size']) / 2 # two bytes per int16
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
              for iBlock in range(nFullBlocks): # TODO speed up by writing more than one byte at a time?
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
       tmp_driverHostname_list = self.driverHostnameList_inactive
       self.addressList_inactive  = [[] for jrad in range(self.RHM.N_RADARs)]
       self.antennaList_inactive  = [[] for jrad in range(self.RHM.N_RADARs)]
       self.hostnameList_inactive = [[] for jrad in range(self.RHM.N_RADARs)]
       self.driverHostnameList_inactive = [[] for jrad in range(self.RHM.N_RADARs)]

       do_resync = False
       for jrad in range(self.RHM.N_RADARs):
         for iUSRP,usrp in enumerate(tmp_address_list[jrad]):

            if usrp in self.addressList_active[jrad]:
               self.logger.error(" Already connected to USRP {}:{}, something went wrong!".format(usrp[0],usrp[1]))
               #idx_usrp = self.hostnameList_active.index(usrp[0])
               #self.antennaList_active[idx_usrp].append(usrpConfig['array_idx'])

            try:
               usrpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
               usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
               usrpsock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
               time.sleep(0.002)
               usrpsock.connect(usrp)

               self.socks[jrad].append(usrpsock)
               self.addressList_active[jrad].append(usrp)
               self.antennaList_active[jrad].append(tmp_antenna_list[jrad][iUSRP])
               self.hostnameList_active[jrad].append(tmp_hostname_list[jrad][iUSRP])
               self.driverHostnameList_active[jrad].append(tmp_driverHostname_list[jrad][iUSRP])
               self.logger.info('reconnection to usrp {} successful'.format(tmp_hostname_list[jrad][iUSRP]))
               do_resync = True

            except ConnectionRefusedError:
               self.logger.warning('reconnection to usrp {} failed'.format(tmp_hostname_list[jrad][iUSRP]))
               self.addressList_inactive[jrad].append(usrp)
               self.antennaList_inactive[jrad].append(tmp_antenna_list[jrad][iUSRP])
               self.hostnameList_inactive[jrad].append(tmp_hostname_list[jrad][iUSRP])
               self.driverHostnameList_inactive[jrad].append(tmp_driverHostname_list[jrad][iUSRP])

       # sync to other usrps
       if do_resync:
         self.RHM._resync_usrps()
         self.RHM.rxfe_init()


class usrpMixingFreqManager():
    """ Manages usrp mixing frequency based on channels. Ensures that only one channel
        at a time can call add_new_freq_band(). """

    def __init__(self, cFreq, bandWidth, N_RADARs):
       self.current_mixing_freq = [cFreq for jrad in range(N_RADARs)] # in kHz (to be compatible with control program)
       self.usrp_bandwidth      = bandWidth - USRP_BANDWIDTH_RESTRICTION*2/1000   # in kHz (to be compatible with control program)
       self.semaphore = posix_ipc.Semaphore('usrp_mixing_freq', posix_ipc.O_CREAT)
       self.semaphore.release()
       self.channelRangeList    = [[] for jrad in range(N_RADARs)]
       self.channelUniqueList   = [[] for jrad in range(N_RADARs)]
       self.channelList         = [[] for jrad in range(N_RADARs)]


    def add_new_freq_band(self, channel):
       """ Checks if new channel is covered with current mixing frequency and bandwidth.
           Returns True/False if channel can/cannot be added. If changing the mixing frequency
           allows to add the new channel, the new mixing frequency will be output argument.
       """

       RHM = channel.parent_RadarHardwareManager
       jrad = channel.rnum

       # if channel is already in list then this must be a new scan,
       # so remove all channel and search range information
       if channel.cnum in self.channelList[jrad]:
          self.channelRangeList[jrad] = []
          self.channelUniqueList[jrad] = []
          self.channelList[jrad] = []

       newLower, newUpper = self.get_range_of_channel(channel)
       if newLower < RHM.hardwareLimit_freqRange[jrad][0] or newUpper > RHM.hardwareLimit_freqRange[jrad][1]:
          channel.logger.error("radar {} ch {}: channel bandwidth ({} - {} MHz) is not covered by radar hardware limits ({} - {} MHz)".format(channel.rnum, channel.cnum, newLower/1000, newUpper/1000, RHM.hardwareLimit_freqRange[jrad][0]/1000, RHM.hardwareLimit_freqRange[jrad][1]/1000))
          return False

       channel.logger.debug("radar {} ch {}: waiting for semaphore of usrpMixingFreqManager".format(channel.rnum, channel.cnum))
       self.semaphore.acquire()
       channel.logger.debug("radar {} ch {}: acquired semaphore of usrpMixingFreqManager".format(channel.rnum, channel.cnum))

       if newLower >= (self.current_mixing_freq[jrad] - self.usrp_bandwidth/2) and newUpper <= (self.current_mixing_freq[jrad] + self.usrp_bandwidth/2):
          channel.logger.debug("radar {} ch {}: channel range is within USRP bandwidth".format(channel.rnum, channel.cnum))
          result = True
       else:
          # determine range of all channels
          allCh_lower = newLower
          allCh_upper = newUpper

          for otherChRange in self.channelRangeList[jrad]:
             allCh_lower = min(allCh_lower, otherChRange[0])
             allCh_upper = max(allCh_upper, otherChRange[1])

          if (allCh_upper - allCh_lower) > self.usrp_bandwidth:
             channel.logger.error("radar {} ch {}: new channel can not be added. USRP bandwidth too small".format(channel.rnum, channel.cnum))
             result = False
          else:
             channel.logger.debug("radar {} ch {}: trying to adjust mixing frequency to support new channel".format(channel.rnum, channel.cnum))
             newMixingFreq = (allCh_upper - allCh_lower)/2 + allCh_lower
             # adjust mixing freq that everything is in overall bandwidth
             newMixingFreq = max(newMixingFreq, RHM.hardwareLimit_freqRange[jrad][0]+self.usrp_bandwidth/2)
             newMixingFreq = min(newMixingFreq, RHM.hardwareLimit_freqRange[jrad][1]-self.usrp_bandwidth/2)
             result = newMixingFreq

       # adjust mixing freq to avoid overlap with channel search ranges
       if result is not False:
          uniqueList = self.get_unique_ranges(channel)
          if result is True:
             newMixingFreq = self.current_mixing_freq[jrad]

          shift = -1
          attempt = 0
          invalid = [True] * len(uniqueList)
          while any(invalid):
             for idx, tmpCh in enumerate(uniqueList):
                while ( ((tmpCh[0] < newMixingFreq+50) and
                         (newMixingFreq-50 < tmpCh[1])) or
                        (tmpCh[0] < newMixingFreq - self.usrp_bandwidth/2) or
                        (tmpCh[1] > newMixingFreq + self.usrp_bandwidth/2) ):

                   # conflict with this range, so must check all others after adjusting
                   invalid[:] = [True] * len(uniqueList)

                   # adjust mixing frequency by 50 kHz
                   newMixingFreq += shift*50

                   # shift in the other direction if a hardware limit is reached
                   if (newMixingFreq < RHM.hardwareLimit_freqRange[jrad][0] or
                       newMixingFreq > RHM.hardwareLimit_freqRange[jrad][1]):
                      shift *= -1

                   # stop adjusting after too many attempts
                   attempt += 1
                   if attempt > 1000:
                      channel.logger.error("radar {} ch {}: could not adjust mixing frequency to avoid overlap with channel range {}".format(channel.rnum, channel.cnum, tmpCh))
                      self.semaphore.release()
                      channel.logger.debug("radar {} ch {}: released semaphore of usrpMixingFreqManager".format(channel.rnum, channel.cnum))
                      return False

                # this search range does not conflict with the mixing freq or bandwidth edges
                invalid[idx] = False

          self.channelUniqueList[jrad] = uniqueList
          self.channelRangeList[jrad].append([newLower, newUpper])
          self.channelList[jrad].append(channel.cnum)

          if newMixingFreq == self.current_mixing_freq[jrad]:
             result = True
          else:
             channel.logger.info("radar {} ch {}: calculated new usrp mixing frequency: {} kHz (old was {} kHz)".format(channel.rnum, channel.cnum, newMixingFreq, self.current_mixing_freq[jrad]))
             self.current_mixing_freq[jrad] = newMixingFreq
             result = newMixingFreq

       self.semaphore.release()
       channel.logger.debug("radar {} ch {}: released semaphore of usrpMixingFreqManager".format(channel.rnum, channel.cnum))
       return result


    def get_range_of_channel(self, channel):
       if channel.scanManager.fixFreq in [ None, -1, 0]:
          rangeList = channel.scanManager.clear_freq_range_list
          lower = rangeList[0][0]
          upper = rangeList[0][1]
          for periodRange in rangeList[1:]:
             lower = min(lower, periodRange[0])
             upper = max(upper, periodRange[1])
       else:
          lower = channel.scanManager.fixFreq
          upper = channel.scanManager.fixFreq
       return lower, upper


    def get_unique_ranges(self, channel):
       # get list of all clear search ranges in scan
       if channel.scanManager.fixFreq in [ None, -1, 0]:
          rangeList = channel.scanManager.clear_freq_range_list
       else:
          rangeList = [[channel.scanManager.fixFreq, channel.scanManager.fixFreq]]

       # get unique search ranges from any other channels
       if self.channelUniqueList[channel.rnum]:
          channelUniqueList = self.channelUniqueList[channel.rnum]
          for unique in channelUniqueList:
             rangeList.append(unique)

       if len(rangeList) == 1:
          return rangeList
       else:
          # get unique clear search ranges
          ctr = Counter(frozenset(x) for x in rangeList)

          # convert frozenset of unique search ranges to list
          # and check for any fixed frequencies
          tmpList = [list(x) for x in ctr]
          for x in range(len(tmpList)):
            if len(tmpList[x]) == 1:
              fixfrq = tmpList[x]
              tmpList[x] = []
              tmpList[x] = [fixfrq, fixfrq]

          # sort unique search ranges from lowest to highest
          uniqueList = sorted([[y,x] if y<x else [x,y] for x,y in tmpList])

          return uniqueList


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
    SAMPLES_NUM         = 5000
    ANTENNA_NUM         = 16
    STATIC_ANTENNA_NUM  = 20
    RESTRICT_NUM        = 20
    META_ELEM           = 3                                    # 3 = 4 - 1 (fcenter has unique obj)
    CLR_BAND_MAX        = 6

    SAMPLES_ELEM_NUM    = ANTENNA_NUM * SAMPLES_NUM * 2
    CLR_RANGE_ELEM_NUM  = 2
    RESTRICT_ELEM_NUM   = RESTRICT_NUM * 2
    META_ELEM_NUM       = META_ELEM + ANTENNA_NUM
    CLR_BAND_ELEM_NUM   = 1 * 3                         # 3     = start & stop freqs and noise
    SITE_ID_ELEM_NUM    = 1 * 3                         # 1 * 3 = one instance of a 3 letter identifier

    SAMPLES_SHM_SIZE        = (ANTENNA_NUM * SAMPLES_NUM * 2 * INT_SIZE)
    CLR_RANGE_SHM_SIZE      = (2 * INT_SIZE)
    FCENTER_SHM_SIZE        = (1 * INT_SIZE)
    BEAM_NUM_SHM_SIZE       = (1 * INT_SIZE)
    SAMPLE_SEP_SHM_SIZE     = (1 * INT_SIZE)
    RESTRICT_SHM_SIZE       = (RESTRICT_NUM * 2 * INT_SIZE)             # 2 = start and end freqs
    META_DATA_SHM_SIZE      = ((META_ELEM + ANTENNA_NUM) * DOUBLE_SIZE)
    ANTENNA_SHM_SIZE        = (1 * INT_SIZE)
    CLR_BAND_SHM_SIZE       = (1 * INT_SIZE * 3)
    SITE_ID_SHM_SIZE        = (3 * CHAR_SIZE)
    RADAR_ID_SHM_SIZE       = (1 * INT_SIZE)
    CHANNEL_ID_SHM_SIZE     = (1 * INT_SIZE)
    ACTIVE_CLIENTS_SHM_SIZE = (1 * INT_SIZE)
    MUTED_ANT_SHM_SIZE      = (STATIC_ANTENNA_NUM * INT_SIZE)           # List of Muted Antennas

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
    SITE_ID_SHM_NAME =          "/site_id"
    RADAR_ID_SHM_NAME =         "/radar_id"
    CHANNEL_ID_SHM_NAME =       "/channel_id"
    ACTIVE_CLIENTS_SHM_NAME =   "/active_clients"   # For Debugging
    MUTED_ANT_SHM_NAME =        "/muted_ant"

    # Semaphore Constants
    SAMPLE_PARAM_NUM =      2
    RESTRICT_PARAM_NUM =    2
    PARAM_NUM =             10

    SEM_F_CLIENT =      "/sf_client"               # For reserving client and server roles during data transfer
    SEM_F_SERVER =      "/sf_server"               # And for signalling specific data transfers
    SEM_F_SAMPLES =     "/sf_samples"
    SEM_F_INIT =        "/sf_init"
    SEM_F_CLRFREQ =     "/sf_clrfreq"
    SEM_F_PROCESSED =   "/sf_processed"            # For processed data transfer
    SEM_L_SAMPLES =     "/sl_samples"              # For Data locking b/w write/reads
    SEM_L_INIT =        "/sl_init"                 # init = initialization
    SEM_L_CLRFREQ =     "/sl_clrfreq"

    SEM_NUM =       9
    SL_NUM =        3

    # Service Variables
    semaphores = []
    shm_objects = []
    muted_antennas = []
    cur_antenna_num = ANTENNA_NUM
    old_meta_data = [[], 0, 0.0, 0.0]
    old_smsep = 0

    log = logging.getLogger('clearFrequency')


    def __init__(self):

        try:
            # Skip Initialization if SHMs exists
            if (len(ClearFrequencyService.semaphores) > 0 and len(ClearFrequencyService.shm_objects) > 0):
                print("[clearFrequencyService] Existing Shared Memory Objects and Semaphores found. Skipping Initialization...")
                return

            # Shared Memory Object and Semaphores
            ClearFrequencyService.sf_client      = self.create_semaphore(self.SEM_F_CLIENT)
            ClearFrequencyService.sf_server      = self.create_semaphore(self.SEM_F_SERVER)
            ClearFrequencyService.sf_samples     = self.create_semaphore(self.SEM_F_SAMPLES)
            ClearFrequencyService.sf_init        = self.create_semaphore(self.SEM_F_INIT)
            ClearFrequencyService.sf_clrfreq     = self.create_semaphore(self.SEM_F_CLRFREQ)
            ClearFrequencyService.sf_processed   = self.create_semaphore(self.SEM_F_PROCESSED)
            ClearFrequencyService.sl_samples     = self.create_semaphore(self.SEM_L_SAMPLES)
            ClearFrequencyService.sl_init        = self.create_semaphore(self.SEM_L_INIT)
            ClearFrequencyService.sl_clrfreq     = self.create_semaphore(self.SEM_L_CLRFREQ)
            ClearFrequencyService.semaphores = [
                ClearFrequencyService.sf_client,
                ClearFrequencyService.sf_server,
                ClearFrequencyService.sf_samples,
                ClearFrequencyService.sf_init,
                ClearFrequencyService.sf_clrfreq,
                ClearFrequencyService.sf_processed,
                ClearFrequencyService.sl_samples,
                ClearFrequencyService.sl_init,
                ClearFrequencyService.sl_clrfreq,
            ]
            ClearFrequencyService.shm_objects = [
                self.create_shm_obj(self.SAMPLES_SHM_NAME,          self.SAMPLES_SHM_SIZE       , self.SAMPLES_ELEM_NUM),
                self.create_shm_obj(self.FCENTER_SHM_NAME,          self.FCENTER_SHM_SIZE       , ),
                self.create_shm_obj(self.CLR_RANGE_SHM_NAME,        self.CLR_RANGE_SHM_SIZE     , self.CLR_RANGE_ELEM_NUM),
                self.create_shm_obj(self.BEAM_NUM_SHM_NAME,         self.BEAM_NUM_SHM_SIZE      , ),
                self.create_shm_obj(self.SAMPLE_SEP_SHM_NAME,       self.SAMPLE_SEP_SHM_SIZE    , ),
                self.create_shm_obj(self.RESTRICT_SHM_NAME,         self.RESTRICT_SHM_SIZE      , self.RESTRICT_ELEM_NUM),
                self.create_shm_obj(self.META_DATA_SHM_NAME,        self.META_DATA_SHM_SIZE     , self.META_ELEM_NUM),
                self.create_shm_obj(self.ANTENNA_SHM_NAME,          self.ANTENNA_SHM_SIZE       , ),
                self.create_shm_obj(self.CLRFREQ_SHM_NAME,          self.CLR_BAND_SHM_SIZE      , self.CLR_BAND_ELEM_NUM),
                self.create_shm_obj(self.SITE_ID_SHM_NAME,          self.SITE_ID_SHM_SIZE       , self.SITE_ID_ELEM_NUM),
                self.create_shm_obj(self.RADAR_ID_SHM_NAME,         self.RADAR_ID_SHM_SIZE      , ),
                self.create_shm_obj(self.CHANNEL_ID_SHM_NAME,       self.CHANNEL_ID_SHM_SIZE    , ),
                self.create_shm_obj(self.ACTIVE_CLIENTS_SHM_NAME,   self.ACTIVE_CLIENTS_SHM_SIZE, ),
                self.create_shm_obj(self.MUTED_ANT_SHM_NAME,        self.MUTED_ANT_SHM_SIZE     , self.STATIC_ANTENNA_NUM)
            ]

            for obj in ClearFrequencyService.shm_objects:
                obj['shm_fd'] = self.initialize_shared_memory(obj['name'])

            ClearFrequencyService.active_clients_fd = None
            self.initialize_active_clients_counter()
            print("[clearFrequencyService] Done Initializing...\n\n")
            self.log.debug("clearFrequencyService initialized")

        except ValueError:
            print("[ClearFrequencyService] Initialization Failed. Cleaning up SHM Objects and Semaphores...")
            ClearFrequencyService.soft_kill = True
            self.cleanup_shm()
        except KeyboardInterrupt:
            print("[CFS] Keyboard Interupt triggered during Initialization... Canceling and cleaning up...")
            ClearFrequencyService.soft_kill = True
            self.cleanup_shm()


    def create_shm_obj(self, name: str, size: int, elem_num= 1):
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


    def create_semaphore(self, name: str):
        return {
            'name': name,
            'sem':  self.initialize_semaphore(name)
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


    def initialize_semaphore(self, name):
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


    @classmethod
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
            print(f"[clearFrequencyService] Decremented Active Clients Counter: {active_clients}\n")
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
            # Debug: Record start time
            start_time = time.time()

            flattened_data = []
            if atype == 'complex':
                # Convert to np array
                array_data_np = np.array(array_data, dtype=np.complex64)

                # Flatten and interleave real and imaginary parts as integers
                interleaved_data = np.empty(array_data_np.size * 2, dtype=np.int32)
                interleaved_data[0::2] = array_data_np.real.astype(np.int32).ravel()
                interleaved_data[1::2] = array_data_np.imag.astype(np.int32).ravel()

                # Print set per 2500 elements (till 3 sets) in interleaved_data to verify
                for i in range(0, interleaved_data.size // 5000):
                    if (i < 3): print(f"[Frequency Client] interleaved_data: ", interleaved_data[i * 5000:(i + 1) * 5000], "...")

                # Write directly to shared memory
                obj['shm_ptr'].seek(0)
                obj['shm_ptr'].write(interleaved_data.tobytes())

                return
            elif atype == 'meta':
                for i in range (1, len(array_data)):
                    flattened_data.append(array_data[i])
                # Place antenna list last
                flattened_data += array_data[0]
            # elif atype == "sid":
            #     for letter in array_data:
            #         flattened_data.append(bytes(letter, 'ascii'))
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

            # Determine dtype for Packing
            print(f"flattened array type: {type(flattened_data)}")
            dtype = 'i'
            if atype == 'meta':
                dtype = 'd'
            # elif atype == "sid":
            #     dtype = b'c'
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
                obj['shm_ptr'].write(struct.pack(dtype * obj['elem_num'], *flattened_data))
            else:
                print("[Frequency Client] new_data len of: ", 1)
                print("[Frequency Client] Writing data:\n", flattened_data)

                obj['shm_ptr'].seek(0)
                obj['shm_ptr'].write(struct.pack(dtype * 1, flattened_data))

        # If element size is incorrect, Display error
        except AttributeError as e:
            print("[Frequency Client] ERROR: Element Size is incorrect. send_samples()'s parameters were likely not assigned properly. Please verify...")
            print(f"AttributeError: {e}")
            print(f"Object: {obj}, Attributes: {dir(obj)}")
            raise

        except ValueError as e:
            print("[Frequency Client] ERROR: Antenna list mismatch with sample set. send_samples()'s parameters were likely not assigned properly. Please verify...")

            # Print data difference between the sample set and expected size
            print("sample bytes:", len(interleaved_data.tobytes()))
            obj['shm_ptr'].seek(0, 2)  # Seek to end
            print("expected size:", obj['shm_ptr'].tell())
            obj['shm_ptr'].seek(0)

            print(f"AttributeError: {e}")
            print(f"Object: {obj}, Attributes: {dir(obj)}")
            raise

        finally:
            # Debug: Print time to write
            end_time = time.time()
            elapsed_time = end_time - start_time

            # print(f"[Frequency Client] Time to write {obj['elem_num']} elements: {elapsed_time:.6f} seconds")


    def read_m_data(self, obj):
        """Reads in data from the shared memory file descriptor.

        Args:
            obj (dict): Data object containing file descriptor, shared memory size, and number of elements expected.

        Returns:
            list: Contains 1D list of Shared Memory data.
        """
        obj['shm_ptr'].seek(0)
        read_data = struct.unpack('i' * obj['elem_num'], obj['shm_ptr'].read(obj['size']))

        # Filter out filler/dummy constants
        result = []
        for elem in read_data:
            if elem >= 0:
                result.append(elem)

        # Debug: Verify format of data object's raw data
        print("[clearFrequencyService] Data read from Shm: ", result[:5], "...")  # Print first 10 integers for brevity

        return result


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
                packed_data.append(int(((start_freq + end_freq) / 2) / 1000))
                noise_data.append(noise)
            return packed_data, noise_data


    def get_muted_antenna_list(self):
        return self.muted_antennas


    def premap_shm(self, meta_data=None):
        """Premaps all shared memory objects' pointers to their memory addresses.
        """
        # If no SHM mapping and meta_data exist, map all SHM objects
        if self.shm_objects[0]['shm_ptr'] == None and meta_data != None:

            ## Check for Premapped antenna num
            # Map shared memory object pointer for antenna num
            print(f"Mapping {self.shm_objects[7]['name']}")
            self.log.debug("[clearFrequencyService] Verifying Antenna and Sample size")
            self.shm_objects[7]['shm_ptr'] = mmap.mmap(self.shm_objects[7]['shm_fd'], self.shm_objects[7]['size'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)

            # Check if Antenna Num changed, update corresponding values before they're mapped
            shm_ant_num = self.read_m_data(self.shm_objects[7])[0]
            print("SHM Antenna_num:  ", shm_ant_num)
            print("Meta Antenna num: ", len(meta_data['antenna_list']))
            if shm_ant_num != self.cur_antenna_num or self.cur_antenna_num != len(meta_data['antenna_list']) or self.shm_objects[0]['elem_num'] != (len(meta_data['antenna_list']) * int(meta_data['number_of_samples']) * 2):
                print("Antenna_num has been changed, updating SHM values before further SHM mapping...")
                self.cur_antenna_num = len(meta_data['antenna_list'])

                # Update meta SHM values
                meta_obj = self.shm_objects[6]
                meta_obj['elem_num'] = len(meta_data['antenna_list']) + self.META_ELEM
                meta_obj['size'] = meta_obj['elem_num'] * self.DOUBLE_SIZE
                os.ftruncate(meta_obj['shm_fd'], meta_obj['size'])

                # Update samples SHM values
                samples_obj = self.shm_objects[0]
                samples_obj['elem_num'] = len(meta_data['antenna_list']) * int(meta_data['number_of_samples']) * 2
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


    def send_samples(self, raw_samples, radar_id = 0, fcenter=None, meta_data=None):
        """ Waits for client requests, then processes server data, writes client
            data, and requests server to process new data. When process is
            terminated, the try/finally block cleans up.

            Note: fcenter and meta_data can be None after being passed as arguments on the first send_samples() method call.
        """
        input_data = [
            raw_samples,
            fcenter,
        #     clr_range,
        ]

        # Filter out Muted Antennas from data
        for i in range(0, len(meta_data['antenna_list'])):
            tmp_arr = np.array(raw_samples[i])
            # print(f"ndim is {tmp_arr.ndim} for ant {i}")
            if tmp_arr.ndim != 1:
                del meta_data['antenna_list'][i]
                print("removing muted ant from antenna_list")
        meta_data['antenna_num'] = len(meta_data['antenna_list'])

        meta_data_list = [
                        meta_data['antenna_list'],
                        meta_data['number_of_samples'],
                        meta_data['x_spacing'],
                        meta_data['usrp_rf_rate'],
                    ]

        # Special: Halt all future ClearFreqService
        if self.soft_kill is True:
            return

        # Fail: If no antennas, skip to recover on next Clear Search Cycle
        if meta_data is None or len(meta_data['antenna_list']) == 0:
            print("[clearFrequencyService] ERROR: No antennas found. Skipping...")
            return

        # Get in Queue
        active_clients = self.increment_active_clients()
        print(f"[clearFrequencyService] Active clients count: {active_clients}")

        try:
            self.premap_shm(meta_data)

            # Await for a Client Request
            print("[clearFrequencyService] Awaiting Client Request...\n")
            self.sf_client['sem'].acquire()
            print("[clearFrequencyService] Acquired Client Request...")

            # Check & Send Initialization Data
            if meta_data is not None:
                print("[clearFrequencyService] Requesting Initialization Semaphore...")
                self.sl_init['sem'].acquire()
                print("[clearFrequencyService] Initialization Semaphore Acquired...")

                # Read Radar ID
                print(f"[Frequency Client] Data Write: {self.shm_objects[10]['name']}")
                self.write_data(self.shm_objects[10], radar_id)

                # # Read Channel ID
                # print(f"[Frequency Client] Data Write: {self.shm_objects[11]['name']}")
                # self.write_data(self.shm_objects[11], channel_id)

                # If meta_data has changed
                if self.old_meta_data != meta_data_list:
                    self.old_meta_data = meta_data_list
                    shm_ant_num = self.read_m_data(self.shm_objects[7])

                    # If antenna length or sample_num has changed, send, set, and sync with server
                    if self.cur_antenna_num != len(meta_data['antenna_list']) or self.shm_objects[0]['elem_num'] != (len(meta_data['antenna_list']) * int(meta_data['number_of_samples']) * 2):
                        print(f"[Frequency Client] Antenna_num changed. Reallocating memory")
                        self.cur_antenna_num = len(meta_data['antenna_list'])

                        # Send Antenna Num
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
                        samples_obj['elem_num'] = len(meta_data['antenna_list']) * int(meta_data['number_of_samples']) * 2
                        samples_obj['size'] = samples_obj['elem_num'] * self.INT_SIZE
                        os.ftruncate(samples_obj['shm_fd'], samples_obj['size'])
                        samples_obj['shm_ptr'] = mmap.mmap(samples_obj['shm_fd'], samples_obj['size'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)

                    # If server's antenna num is outdated, update it
                    elif shm_ant_num != self.cur_antenna_num:
                        # Send
                        print(f"[Frequency Client] Data Write Progress: {self.shm_objects[7]['name']}")
                        self.write_data(self.shm_objects[7], len(meta_data['antenna_list']))

                    print(f"[Frequency Client] Data Write Progress: {self.shm_objects[6]['name']}")

                    # Rearrange meta_data ordering
                    self.write_data(self.shm_objects[6], meta_data_list, 'meta')

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
                    print(f"[Frequency Client] Data Write Progress: {i}/{self.SAMPLE_PARAM_NUM} {self.shm_objects[i]['name']}")

                    # General: Write updated input data
                    if input_data[i] is not None:
                        self.write_data(self.shm_objects[i], input_data[i])

                # Write Radar ID
                self.write_data(self.shm_objects[10], radar_id)

                self.sl_samples['sem'].release()
                self.sf_samples['sem'].release()
                print("[Frequency Client] Done writing data to Shared Memory...")
                self.log.debug("[clearFrequencyService] Writing Sample data to SHM done")

                # Request Server
                print("[clearFrequencyService] Requesting Server Response...")
                self.log.debug("[clearFrequencyService] Requesting Server response")
                self.sf_server['sem'].release()

                # Wait for Processed Data
                self.sf_processed['sem'].acquire()
                self.sl_samples['sem'].acquire()

                self.muted_antennas = []
                self.muted_antennas = self.read_m_data(self.shm_objects[13])
                # print(f"[ClearFrequencyService] Muted Antennas: {self.muted_antennas}")

                self.sl_samples['sem'].release()

        except KeyboardInterrupt:
            print("[clearFrequencyService] Keyboard interrupt received. Exiting...")
        except posix_ipc.ExistentialError or ValueError or AttributeError:
                print("[clearFrequencyService] Shared memory has been delinked. Exiting...")
        finally:
            active_clients = self.decrement_active_clients()

        return


    def request_clr_freq(self, radar_id, channel_id, beam_num=None, sample_sep=None, clr_range=None, fcenter=None):
        """ Waits for client requests, then processes server data, writes client
            data, and requests server to process new data. When process is
            terminated, the try/finally block cleans up.\

            Note that sample_sep is not expected to change for each clr freq request.
        """

        input_data = [
            fcenter,
            clr_range,
            beam_num,
            sample_sep,
        ]

        # Special: Halt all future ClearFreqService
        if self.soft_kill is True:
            return

        # Get in Queue
        active_clients = self.increment_active_clients()
        print(f"[clearFrequencyService] Active clients count: {active_clients}")

        try:
            self.premap_shm()

            # Await for a Client Request
            print("[clearFrequencyService] Awaiting Client Request...\n")
            self.sf_client['sem'].acquire()
            print("[clearFrequencyService] Acquired Client Request...")

            # Write Input Data present
            print("[clearFrequencyService] Requesting ClrFreq Semaphore...")
            self.sl_clrfreq['sem'].acquire()
            print("[clearFrequencyService] ClrFreq Semaphore Acquired...")

            for i in range(self.SAMPLE_PARAM_NUM-1, self.SAMPLE_PARAM_NUM + 3):

                # If data present, write to SHM
                if input_data[i - 1] is not None:
                    print(f"[Frequency Client] Data Write: {self.shm_objects[i]['name']}")
                    self.write_data(self.shm_objects[i], input_data[i - 1])

            # Write Radar ID
            print(f"[Frequency Client] Data Write: {self.shm_objects[10]['name']}")
            self.write_data(self.shm_objects[10], radar_id)

            # Write Channel ID
            print(f"[Frequency Client] Data Write: {self.shm_objects[11]['name']}")
            self.write_data(self.shm_objects[11], channel_id)

            self.sl_clrfreq['sem'].release()
            print("[clearFrequencyService] ClrFreq Semaphore Released ...")

            # Send Clear Frequency and Server Request
            print("[clearFrequencyService] Requesting Clear Freq...")
            self.sf_clrfreq['sem'].release()
            print("[clearFrequencyService] Requesting Server Response...")
            self.sf_server['sem'].release()

            # Read-in Clear Freq data
            print("[clearFrequencyService] Awaiting Server Response...")
            self.sf_processed['sem'].acquire()
            print("[clearFrequencyService] Recieved Server Response. Reading Clear Freq data...")
            self.sl_clrfreq['sem'].acquire()
            new_noise_data = []
            new_clrfreq_data = self.read_m_data(self.shm_objects[8])
            new_clrfreq_data, new_noise_data = self.repack_data(new_clrfreq_data, True)
            for clr_freq_and_noise in zip(new_clrfreq_data, new_noise_data):
                print(f"[clearFrequencyService] Clear Freq Band: | {clr_freq_and_noise[0]} (kHz), {clr_freq_and_noise[1]} (N/A) |")
            clr_freq, noise = new_clrfreq_data[0], new_noise_data[0]
            self.log.debug("[clearFrequencyService] clr_freq recieved...")

            self.sl_clrfreq['sem'].release()

        except KeyboardInterrupt:
            print("[clearFrequencyService] Keyboard interrupt received. Exiting...")
        except posix_ipc.ExistentialError or ValueError or AttributeError:
                print("[clearFrequencyService] Shared memory has been delinked. Exiting...")
        finally:
            active_clients = self.decrement_active_clients()

        return clr_freq, noise


    @classmethod
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
    def __init__(self, antenna_spacing, usrpManager, N_RADARs):
        self.rawData     = [None for jrad in range(N_RADARs)]
        self.antennaList = [None for jrad in range(N_RADARs)]
        self.recordTime  = [None for jrad in range(N_RADARs)]

        self.outstanding_request = [False for jrad in range(N_RADARs)]     # Flag set by the RadarChannelHandlers
        self.repeat_request_for_2nd_period = False

        self.usrpManager = usrpManager # TODO change to take socks automatically from usrpManager
        self.usrp_socks = [None for jrad in range(N_RADARs)]
        self.center_freq = [None for jrad in range(N_RADARs)]
        self.sampling_rate = [None for jrad in range(N_RADARs)]
        self.number_of_samples = None
        self.CFS = ClearFrequencyService()

        self.metaData = [{} for jrad in range(N_RADARs)]

        self.freq_occupied_by_other_channels = []
        self.get_raw_data_semaphore = threading.BoundedSemaphore()
        self.select_clear_freq = threading.BoundedSemaphore()

        for j in range(N_RADARs):
           self.metaData[j]['x_spacing'] = antenna_spacing[j]

        self.logger = logging.getLogger('clearFrequency')
        self.logger.debug('clearFrequencyRawDataManager initialized')

        self.cycleTicker = 0


    def set_usrp_driver_connections(self, jrad, usrp_driver_socks):
        self.usrp_socks[jrad] = usrp_driver_socks


    def set_clrfreq_search_span(self, jrad, center_freq, clrfreq_sampling_rate, number_of_clrfreq_samples):
        self.center_freq[jrad] = center_freq

        self.sampling_rate = clrfreq_sampling_rate
        self.number_of_samples = number_of_clrfreq_samples

        self.logger.debug('clearFrequencyRawDataManager set_clrfreq_search_span: center {} rate {} number {}'.format(self.center_freq[jrad],self.sampling_rate,self.number_of_samples))

        self.metaData[jrad]['usrp_fcenter'] = self.center_freq[jrad]
        self.metaData[jrad]['number_of_samples'] = self.number_of_samples
        self.metaData[jrad]['usrp_rf_rate'] = self.sampling_rate


    def reset_occupied_freqs(self):
        self.freq_occupied_by_other_channels = []


    def update_auto_clear_freq_data(self, jrad, antenna_list, raw_data, meta_data_dict):
        """ Take the auto clear freq data collected at the end of tranmitting and update the clear freq class"""
        self.rawData[jrad] = raw_data
        self.recordTime[jrad] = meta_data_dict['record_time']
        self.antennaList[jrad] = antenna_list
        self.metaData[jrad]['antenna_list'] = antenna_list
        self.sampling_rate = meta_data_dict['sampling_rate']
        self.center_freq[jrad] = meta_data_dict['center_freq'] /1000
        self.CFS.send_samples(self.rawData[jrad], int(jrad), int(self.center_freq[jrad]), meta_data=self.metaData[jrad])
        self.logger.debug("Updated clear freq raw data with auto_clear_freq data")

        # Update Muted Antenna List
        self.usrpManager.RHM.mute_antenna_list[jrad] = self.CFS.get_muted_antenna_list()


    def record_new_data(self, jrad):
        assert self.usrp_socks[jrad] != None, "no usrp drivers assigned to clear frequency search data manager"
        assert self.center_freq[jrad] != None, "no center frequency assigned to clear frequency search manager"

        if self.recordTime[jrad] == None:
            data_age = np.inf
            rec_new_samples = True
        else:
            data_age = time.time() - self.recordTime[jrad]
            rec_new_samples = data_age > self.usrpManager.RHM.auto_max_age

        if rec_new_samples:
            self.logger.debug("clearFreqRawData: age of data is {:2.2f} s. Recording new data".format(data_age))
            self.logger.debug('start record_clrfreq_raw_samples on radar {}'.format(jrad))
            self.rawData[jrad], self.antennaList[jrad] = record_clrfreq_raw_samples(self.usrpManager.get_all_main_antenna_socks(jrad), self.number_of_samples, self.center_freq[jrad], self.sampling_rate, self.usrpManager.RHM.min_clrfreq_delay)
            self.logger.debug('end record_clrfreq_raw_samples')

            self.metaData[jrad]['antenna_list'] = self.antennaList[jrad]
            self.CFS.send_samples(self.rawData[jrad], int(jrad), int(self.center_freq[jrad]), meta_data=self.metaData[jrad])
            self.logger.debug("recorded clear samples for clear frequency search, antenna list: {}".format(self.antennaList[jrad]))

            # so, self.rawData is np.array(complex(nantennas, nsamples)
            self.recordTime[jrad] = time.time()

            # Update Muted Antenna List
            self.usrpManager.RHM.mute_antenna_list[jrad] = self.CFS.get_muted_antenna_list()

            self.logger.debug("clrfreq record time: {}".format(self.recordTime[jrad]))
        else:
            self.logger.debug("clearFreqRawData: age of data is {:2.2f} s. No need to record new data...".format(data_age))

        self.outstanding_request[jrad] = False


    def get_raw_data(self,jrad):
        #self.get_raw_data_semaphore.acquire()
        if self.rawData[jrad] is None:
           self.record_new_data(jrad)
        else:
           self.logger.debug("clearFreqDataManager: recordTime {}".format(self.recordTime[jrad]))
           self.logger.debug("clearFreqDataManager: length rawData {}".format(len(self.rawData[jrad])))
           self.logger.debug("clearFreqDataManager: provide raw data (age {}), setting raw_rec request".format(time.time() - self.recordTime[jrad]))
           self.outstanding_request[jrad] = True

        #self.get_raw_data_semaphore.release()
        return self.rawData[jrad], self.metaData[jrad], self.recordTime[jrad]


    def add_channel(self, freq, bandwidth):
        freq *= 1000
        self.freq_occupied_by_other_channels.append([freq - bandwidth*1.5, freq + bandwidth*1.5])


class swingManager():
    """ Class to handle which swing is active and processing """

    def __init__(self):
        self.activeSwing     = 0
        self.processingSwing = 1

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

    def __init__(self, channel):
        self.scan_beam_list        = []
        self.clear_freq_range_list = []
        self.fixFreq = None

        self.current_period = 0
        self.repeat_clrfreq_recording = False # 2nd period is triggered automatically before ROS finishes 1st. if CLR_FRQ was requested for 1st => also do record on 2nd

        self.channel = channel
        self.RHM = channel.parent_RadarHardwareManager
        self.clearFreqService = ClearFrequencyService()
        self.beamSep = self.RHM.array_beam_sep[channel.rnum]
        self.numBeams = self.RHM.array_nBeams[channel.rnum]

        self.current_clrFreq_result = None
        self.next_clrFreq_result    = None
        self.isPrePeriod = True # is very first trigger_next_period() call that just triggers first period but does not collect cuda data
        self.isPostLast = False # to handle last trigger_next_swing() call
        self.logger = logging.getLogger('scanManager')

        self.logger.debug("in scanManager __init__. Setting isInitSetParameter true")
        self.isInitSetParameter = True

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
        self.logger.info("Starting scan with period {}.".format(iPeriod))


    def wait_for_next_trigger(self):
        if self.syncBeams:
           time_to_wait = self.beam_times[self.current_period] - self.get_time_in_scan() - self.RHM.integration_time_manager.get_usrp_delay_time()
           if time_to_wait > 0:
              self.logger.debug("Waiting for {} s".format(time_to_wait))
              time.sleep(time_to_wait)
           else:
              self.logger.debug("No waiting. ({} + {}) s too late.".format(time_to_wait + self.RHM.integration_time_manager.get_usrp_delay_time(), self.RHM.integration_time_manager.get_usrp_delay_time()))


    def init_new_scan(self, freq_range_list, scan_beam_list, fixFreq, scan_times_list, scan_duration, integration_duration, start_period):

        # list of [fstart, fstop] lists in Hz, desired frequency range for each period
        self.clear_freq_range_list = freq_range_list

        # list of [bmnum, bmnum..]
        self.scan_beam_list = scan_beam_list
        self.camping = len(scan_beam_list) == 1

        # sync parameter
        if scan_times_list is None:
            self.syncBeams = None
        else:
            self.syncBeams = True
        self.beam_times = scan_times_list
        self.scan_duration = scan_duration
        self.integration_duration = integration_duration

        self.fixFreq = fixFreq

        # reset all other parameters
        self.current_period         = start_period
        self.current_clrFreq_result = None
        self.next_clrFreq_result    = None
        self.isPrePeriod            = True # is very first trigger_next_period() call that just triggers first period but does not collect cuda data
        self.isPostLast             = False # to handle last trigger_next_swing() call
        self.logger.debug("in init_new_scan. Setting isInitSetParameter true")
        self.isInitSetParameter     = True
        self.isFirstPeriod          = True


    def get_nSec_to_scan_boundary(self, time_now):
        """ To limit the time of integration periods """
        nSec_left = self.scan_duration - ((time.gmtime(time_now).tm_min*60 + time.gmtime(time_now).tm_sec + time_now % 1) % self.scan_duration )
        return nSec_left


    def period_finished(self):
      #  print("swing manager period finished...")
        if self.next_clrFreq_result is not None:
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
        print("current period: {: >2d}/{}, beam: {}".format(self.current_period, len(self.scan_beam_list), self.current_beam))


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


    def get_current_clearFreq_result(self, jrad):
        if self.current_clrFreq_result is None:
           if self.fixFreq > 0: # it looks like control program could use -1 and 0 to disable it
               self.current_clrFreq_result = [self.fixFreq, 0, 0]
               self.logger.debug("Using fixed frequency of {} kHz for current period".format(self.fixFreq))
           else:
               self.logger.debug("  calc current clr_freq (radar {} ch {}, period {})".format(self.channel.rnum, self.channel.cnum, self.current_period))
               self.current_clrFreq_result = self.evaluate_clear_freq(jrad, self.channel.cnum, self.current_period, self.current_beam)
        return self.current_clrFreq_result


    def get_next_clearFreq_result(self, jrad):
        if self.next_clrFreq_result is None:
           if self.fixFreq != -1 and self.fixFreq != 0: # it looks like control program could use -1 and 0 to disable it
               self.next_clrFreq_result = [self.fixFreq, 0, 0]
               self.logger.debug("Using fixed frequency of {} kHz for next period".format(self.fixFreq))
           else:
               self.logger.debug("  calc next clr_freq (radar {} ch {}, period {})".format(self.channel.rnum, self.channel.cnum, self.current_period+1))
               # print("  calc next clr_freq (period {})".format(self.current_period+1))
               if self.camping:
                   next_period = self.current_period
               else:
                   next_period = self.current_period + 1

               self.next_clrFreq_result = self.evaluate_clear_freq(jrad, self.channel.cnum, next_period ,self.next_beam)
        return self.next_clrFreq_result


    def evaluate_clear_freq(self, jrad, cnum, iPeriod, beamNo):
        # TODO make sure this function is only called once at a time
        RHM = self.RHM

        RHM.clearFreqRawDataManager.select_clear_freq.acquire()
        RHM.set_par_semaphore.acquire()

        rawData, metaData, recordTime = RHM.clearFreqRawDataManager.get_raw_data(jrad)

        beam_angle = calc_beam_azm_rad(self.numBeams, beamNo, self.beamSep)
        self.logger.debug("clear_freq_range: {} on radar {} beam {} angle {}".format(self.clear_freq_range_list[iPeriod], jrad, beamNo, beam_angle))

        self.logger.debug('start calc_clear_freq_on_raw_samples')

        clear_freq_range = []
        for freq in self.clear_freq_range_list[iPeriod]:
           clear_freq_range.append(int(freq))
        print(f"smsep:      {int(self.channel.raw_export_data['smsep'])}")
        print(f"fcenter:    {int(metaData['usrp_fcenter'])}")
        print(f"beamNo:     {int(beamNo)}")
        print(f"antenna sample sets: { len(rawData) }")
        print(f"samples: {rawData[:2][:10]}")
        if len(rawData) != len(metaData['antenna_list']):
            self.logger.error("Mismatch in number of ant samples and ant length. len(rawData)= {} antenna_list: {}".format(len(rawData),metaData['antenna_list']))
        self.logger.debug(f"antenna sample sets: {len(rawData)}   antennas: {metaData['antenna_list']}")
        clearFreq, noise = self.clearFreqService.request_clr_freq(int(jrad), int(cnum), int(beamNo), int(self.channel.raw_export_data['smsep']), clear_freq_range, int(metaData['usrp_fcenter']))

        #Failsafe for when the search fails WB 7/21/25
        if clearFreq == 0:
           default_freq = (self.clear_freq_range_list[iPeriod][0]+self.clear_freq_range_list[iPeriod][1])/2
           clearFreq = default_freq
           noise = 99999.99

        self.logger.debug('end calc_clear_freq_on_raw_samples')
        if 'baseband_samplerate' in RHM.commonChannelParameter:
           bandwidth = RHM.commonChannelParameter['baseband_samplerate']
        else:    # first call before channel details are known
           bandwidth = 3333

        RHM.clearFreqRawDataManager.add_channel(clearFreq, bandwidth)

        self.logger.debug("clear freq result for radar {} channel {}: selected {}, noise level {:2.1f}".format(self.channel.rnum, self.channel.cnum, clearFreq, noise))
        RHM.clearFreqRawDataManager.select_clear_freq.release()
        RHM.set_par_semaphore.release()

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
        self.client_sock.bind(('', port))
        # self.client_sock.bind(('localhost', port))

        self.logger = logging.getLogger('HwManager')
        self.logger.info('listening on port ' + str(port) + ' for control programs')

        self.exit_usrp_server = False

        self.set_par_semaphore = threading.BoundedSemaphore()
        self.usrp_setup_semaphore = threading.BoundedSemaphore()
        self.channel_spawn_semaphore = threading.BoundedSemaphore()
        self.clear_search_data_semaphore = threading.BoundedSemaphore()

        self.ini_file_init()
        self.usrp_init()
        self.logger.info('USRPs initialized, now going to RXFE')
        self.rxfe_init()
        #self.test_rxfe_control() # toggle all amp and att stages on and off for testing
        self.logger.info('RXFE initialized, now going to cuda')
        self.cuda_init()

        self.nRegisteredChannels = 0  # number of channels after compatibility check
        self.n_SetParameterHandlers_active = 0
        self.nControlPrograms = 0  # number of control programs, also include unregistered channels
        self.channel_manager_consecutive_number = 10 # serial number shown in logger of channel_manager

        self.clearFreqRawDataManager = clearFrequencyRawDataManager(self.array_x_spacing, self.usrpManager, self.N_RADARs)
        for jrad in range(self.N_RADARs):
           self.clearFreqRawDataManager.set_usrp_driver_connections(jrad, self.usrpManager.socks[jrad]) # TODO check if this also works after reconnection to a usrp (copy or reference?)

           self.clearFreqRawDataManager.set_clrfreq_search_span(jrad, self.mixingFreqManager.current_mixing_freq[jrad], self.usrp_rf_rx_rate, int(AVG_RATIO * self.usrp_rf_rx_rate / CLRFREQ_RES))

           self.send_usrp_setup_command(jrad)

        self.last_period     = [[] for jrad in range(self.N_RADARs)]
        self.active_channels = [[] for jrad in range(self.N_RADARs)]
        self.channels        = [[] for jrad in range(self.N_RADARs)]   # all channels that are really transmitting
        self.newChannelList  = []   # waiting list for channels to be added at the right time (between two trigger_next() calls)
        self.swingManager    = swingManager()

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
                self.logger.error("RadarChannelHandler: Socket error => Deleting channel...")
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
            controlLoop_logger.info('Starting RHM.radar_main_control_loop()')
            statusFile = statusUpdater(self)
            sleepTime = 0.01 # used if control loop waits for one channel

            while True:

                # statusFile.update()
                statusFile.update_advanced()

                if self.exit_usrp_server:
                   self.logger.info("ending main_control_loop")
                   break

                # set start time of integration period (will be overwriten if not triggered)
                self.starttime_period = time.time() # TODO change this to reference clock and scan times

                # check if there are any disconnected USRPs
                # (only if no radars are active, to avoid resync interrupting scan)
                if not radar_active.any():
                   for jrad in range(self.N_RADARs):
                      if len(self.usrpManager.addressList_inactive[jrad]):
                         self.usrpManager.restore_lost_connections()

                # CLEAR FREQ SEARCH: recoring when ever requested (independent of swing, state or channel)
                for jrad in range(self.N_RADARs):
                   if self.clearFreqRawDataManager.outstanding_request[jrad]:
                      controlLoop_logger.debug('start self.clearFreqRawDataManager.record_new_data({})'.format(jrad))
                      self.clearFreqRawDataManager.record_new_data(jrad)

                      # check if CLR_FREQ has to be repeated
                      if CS_INACTIVE in [ch.active_state for ch in self.channels[jrad]]:
                         controlLoop_logger.debug("Repeating CLR_FREQ for next integation period")
                         self.clearFreqRawDataManager.repeat_request_for_2nd_period = True

                      controlLoop_logger.debug('end self.clearFreqRawDataManager.record_new_data({})'.format(jrad))
                      for ch in self.channels[jrad]:
                         controlLoop_logger.debug('radar {} ch {}: active state: {}'.format(jrad, ch.cnum,ch.active_state))

                # FIRST CUDA_ADD FOR NEW CHANNELS
                if len(self.newChannelList) != 0:
                   for jrad in range(self.N_RADARs):
                      self.logger.debug("active_channel list: {} on radar {}".format([active_ch.cnum for active_ch in self.active_channels[jrad]],jrad))

                   for jrad in range(self.N_RADARs):
                      self.logger.debug("channel list: {} on radar {}".format([(ch.cnum, ch.rnum) for ch in self.channels[jrad]],jrad))

                   self.logger.debug("new channel list: {} on radar {}".format([ch.cnum for ch in self.newChannelList],[ch.rnum for ch in self.newChannelList]))

                   for jrad in range(self.N_RADARs):
                      for active_ch in self.active_channels[jrad]:
                         while (active_ch not in (self.channels[jrad] + self.newChannelList)):
                            self.logger.info("Waiting for radar {} ch {} to be added to newChannelList".format(jrad, active_ch.cnum))
                            time.sleep(0.01)

                   while(self.n_SetParameterHandlers_active):
                      self.logger.debug("Waiting for all {} SetParameterHandlers to finish before initializing new channels".format(self.n_SetParameterHandlers_active))
                      time.sleep(0.001)
                   controlLoop_logger.info('initializing channel')
                   self.initialize_channel()

                # TRIGGER if any channel is in CS_TRIGGER
                if CS_TRIGGER in [ch.active_state for rchans in self.channels for ch in rchans]:
                   # wait for all channels to be in TRIGGER state
                   executeTrigger = True
                   for ch in np.concatenate(self.channels).tolist():
                         if ch.active_state not in [CS_TRIGGER, CS_INACTIVE]:
                            controlLoop_logger.debug('remaining in TRIGGER because radar {} channel {} state is {} (active swing is {})'.format(ch.rnum, ch.cnum, ch.active_state, ch.swingManager.activeSwing))
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
               self.logger.info("ending control program sock loop")
               break

            self.channel_spawn_semaphore.acquire()

            usrp_server_logger.info('connection from control program, spawning channel handler thread') #NEED SEMAPHORE!!!!!!!!!!!!!!!
            ct = threading.Thread(target=spawn_channel, args=(client_conn,), daemon=False)
            client_threads.append(ct)
            ct.start()

            # remove threads that are not alive
            # TODO: the client_sock.accept is probably blocking, this will only execute after connection of a new control program. is that the intended behavior? -jtk
            client_threads = [iThread for iThread in client_threads if iThread.is_alive()]

            self.channel_spawn_semaphore.release()

        self.client_sock.close()


    # read in ini config files..
    def ini_file_init(self):
        # READ array_config.ini
        array_config = configparser.ConfigParser()
        array_config.read('../array_config.ini')

        self.N_RADARs = int(array_config['array_info']['nradars'])
        self.ini_dsp_info = array_config['dsp_filters']

        self.ThisRadar               = [[] for jrad in range(self.N_RADARs)]
        self.array_nBeams            = [[] for jrad in range(self.N_RADARs)]
        self.array_beam_sep          = [[] for jrad in range(self.N_RADARs)]
        self.array_x_spacing         = [[] for jrad in range(self.N_RADARs)]
        self.hardwareLimit_freqRange = [[] for jrad in range(self.N_RADARs)]

        self.ini_rxfe_settings       = [[] for jrad in range(self.N_RADARs)]
        self.scaling_factor_tx_total = [[] for jrad in range(self.N_RADARs)]
        self.scaling_factor_rx_bb    = [[] for jrad in range(self.N_RADARs)]
        self.scaling_factor_rx_if    = [[] for jrad in range(self.N_RADARs)]
        self.apply_normalization     = [[] for jrad in range(self.N_RADARs)]
        self.mute_antenna_list       = [[] for jrad in range(self.N_RADARs)]

        for jrad in range(self.N_RADARs):
            self.ThisRadar[jrad]       = array_config['array_info']['stid']
            self.array_nBeams[jrad]    =   int(array_config['array_info']['nbeams'])
            self.array_beam_sep[jrad]  = float(array_config['array_info']['beam_sep']) # degrees
            self.array_x_spacing[jrad] = float(array_config['array_info']['x_spacing']) # meters
            self.hardwareLimit_freqRange[jrad] = [float(array_config['hardware_limits']['minimum_tfreq'])/1000, float(array_config['hardware_limits']['maximum_tfreq'])/1000] # converted to kHz

            self.ini_rxfe_settings[jrad]       = array_config['rxfe']
            self.scaling_factor_tx_total[jrad] = float(array_config['gain_control']['scaling_factor_tx_total'])
            self.scaling_factor_rx_bb[jrad]    = float(array_config['gain_control']['scaling_factor_rx_bb'])
            self.scaling_factor_rx_if[jrad]    = float(array_config['gain_control']['scaling_factor_rx_if'])
            self.apply_normalization[jrad]     = array_config.getboolean('gain_control','use_var_normalization')
            mute_antenna_str = array_config['gain_control']['mute_antenna_idx']
            if len(mute_antenna_str):
                self.mute_antenna_list[jrad] = [int(x) for x in mute_antenna_str.split(",")]
            else:
                self.mute_antenna_list[jrad] = []

            if self.apply_normalization[jrad]:
                self.logger.info("Radar {}: Normalizing is active.".format(jrad))
            if len(self.mute_antenna_list[jrad]):
                self.logger.info("Radar {}: Mute antennas before beamforming: {}".format(jrad, self.mute_antenna_list[jrad]))

            if (self.N_RADARs == 2) and (jrad == 0):
                radar_2 = array_config['array_info']['stid_2']
                array_config = configparser.ConfigParser()
                array_config.read('../config/{}/array_config__{}.ini'.format(radar_2, radar_2))

        # READ driver_config.ini
        driver_config = configparser.ConfigParser()
        driver_config.read('../driver_config.ini')
        self.ini_log_settings     = driver_config['log_settings']
        self.ini_clr_settings     = driver_config['clr_settings']
        self.ini_shm_settings     = driver_config['shm_settings']
        self.ini_cuda_settings    = driver_config['cuda_settings']
        self.ini_network_settings = driver_config['network_settings']

        self.min_clrfreq_delay = float(self.ini_clr_settings['min_clrfreq_delay'])
        self.auto_max_age      =   int(self.ini_clr_settings['auto_max_age'])
        self.auto_pause_time   = float(self.ini_clr_settings['auto_pause_time'])

        # READ usrp_config.ini
        usrp_config = configparser.ConfigParser()
        usrp_config.read('../usrp_config.ini')
        usrp_configs = []
        self.antenna_idx_list_main = [[] for jrad in range(self.N_RADARs)]
        self.antenna_idx_list_back = [[] for jrad in range(self.N_RADARs)]
        for usrp in usrp_config.sections():
            usrp_configs.append(usrp_config[usrp])
            if usrp_config[usrp]['mainarray'].lower() in ['true', 1]:
               self.antenna_idx_list_main[int(usrp_config[usrp]['radar'])].append(int(usrp_config[usrp]['array_idx']))
            else:
               self.antenna_idx_list_back[int(usrp_config[usrp]['radar'])].append(int(usrp_config[usrp]['array_idx']))

        self.ini_usrp_configs = usrp_configs


    def usrp_init(self):
        self.usrpManager = usrpSockManager(self)
        self.usrp_rf_tx_rate   = int(self.ini_cuda_settings['FSampTX'])
        self.usrp_rf_rx_rate   = int(self.ini_cuda_settings['FSampRX'])
        self.mixingFreqManager = usrpMixingFreqManager(DEFAULT_USRP_MIXING_FREQ, self.usrp_rf_tx_rate/1000, self.N_RADARs)
        self._resync_usrps(first_sync = True)
        self.logger.debug("usrp_init() complete")


    def send_usrp_setup_command(self,jrad):
       self.logger.debug("Call to USRP_SETUP to set initial parameters")
       self.usrp_setup_semaphore.acquire()
       cmd = usrp_setup_command(self.usrpManager.socks[jrad], \
                                self.mixingFreqManager.current_mixing_freq[jrad]*1000, \
                                self.mixingFreqManager.current_mixing_freq[jrad]*1000, \
                                self.usrp_rf_tx_rate, self.usrp_rf_rx_rate, \
                                1, 0, 0, 0, 0, [0], 0)
       cmd.transmit()
       time.sleep(0.001)

       # rxrate,rxfreq,txrate,txfreq = cmd.receive_settings(self.usrpManager.socks[jrad])
       rxrate,rxfreq,txrate,txfreq = cmd.receive_settings()
       cmd.client_return()

       self.logger.debug("USRP_SETUP received  rxrate {} rxfreq {} txrate {} txfreq {}".format(rxrate,rxfreq,txrate,txfreq))

       try:
          # give non-local usrps some extra time to respond
          if not self.usrpManager.socks[jrad][0].getpeername()[0] == '127.0.0.1':
             time.sleep(0.002)
       except:
          self.logger.debug("No USRPs for radar {}".format(jrad))

       # self.mixingFreqManager.current_mixing_freq[jrad] = rxfreq/1000

       # self.usrpManager.eval_client_return(cmd, jrad)
       self.usrp_setup_semaphore.release()
       self.logger.debug("end USRP_SETUP")


    def send_cuda_setup_command(self):
      if self.commonChannelParameter == {}:
         self.logger.debug("Skipping call of cuda_setup because up/down samplingRates are unknown.")
      else:
         self.logger.debug("start CUDA_SETUP")
         for jrad in range(self.N_RADARs):
            cmd = cuda_setup_command(self.cudasocks[jrad], self.commonChannelParameter['upsample_rate'],self.commonChannelParameter['downsample_rates'][0],self.commonChannelParameter['downsample_rates'][1], self.mixingFreqManager.current_mixing_freq[jrad]*1000 )
            cmd.transmit()
            time.sleep(0.001)
            cmd.client_return()

         self.logger.debug("end CUDA_SETUP")


    def _resync_usrps(self, first_sync = False):
       usrps_synced = False
       iResync = 1

       SomeActiveUSRPs=False
       for jrad in range(self.N_RADARs):
          if len(self.usrpManager.socks[jrad]) != 0:
             SomeActiveUSRPs = True

       if not SomeActiveUSRPs:
          self.logger.error("No connection to USRPs. Exit usrp_server.")
          sys.exit(0)

       socks=np.concatenate(self.usrpManager.socks).tolist()

       self.usrp_setup_semaphore.acquire()

       while not usrps_synced:

          cmd = usrp_sync_time_command(socks)
          cmd.transmit()
          time.sleep(0.001)
          self.usrpManager.eval_client_return(cmd, None)

          # once USRPs are connected, synchronize clocks/timers
          cmd = usrp_get_time_command(socks)
          cmd.transmit()
          time.sleep(0.001)

          usrptimes = []
          for iUSRP, usrpsock in enumerate(socks):
             try:
                usrptimes.append(cmd.recv_time(usrpsock))
             except:
                for jrad in range(self.N_RADARs):
                   if usrpsock in self.usrpManager.socks[jrad]:
                      iSock = self.usrpManager.socks[jrad].index(usrpsock)
                      self.logger.error("Error in sync USRPs for {}. Removing it...".format(self.usrpManager.hostnameList_active[jrad][iSock]))
                      self.usrpManager.remove_sock(jrad,usrpsock)

          self.usrpManager.eval_client_return(cmd, None)

          # check if sync succeeded..
          if max(np.abs(np.array(usrptimes) - usrptimes[0])) < .5:
             usrps_synced = True
             self.logger.info('USRPs synchronized, approximate times: ' + str(usrptimes))
          else:
             # TODO: why does USRP synchronization fail?
             self.logger.info("USRP times: {}".format(usrptimes))
             self.logger.warning('_resync_USRP USRP syncronization failed, trying again ({}) ...'.format(iResync))
             iResync += 1
             time.sleep(0.1)

       if not first_sync:
          for jrad in range(self.N_RADARs):
             self.clearFreqRawDataManager.set_usrp_driver_connections(jrad, self.usrpManager.socks[jrad])

       self.usrp_setup_semaphore.release()


    #@timeit
    def rxfe_init(self):

        for jrad in range(self.N_RADARs):
           amp1 = self.ini_rxfe_settings[jrad].getboolean('enable_amp1')
           amp2 = self.ini_rxfe_settings[jrad].getboolean('enable_amp2')

           att = float(self.ini_rxfe_settings[jrad]['attenuation'])
           if att < 0:
              self.logger.warning('attenuation for rxfe in array.ini is defined positive, but given value is negative ({} dB). correcting that to {} dB...'.format(att, att*(-1)))
              att *= -1

           if att > 31.5:
              self.logger.warning('attenuation ({}) for rxfe in array.ini is > 31.5 dB. using maximum atenuation of 31.5 dB'.format(att))
              att = 31.5

           self.logger.info("Radar {}: Setting RXFE: Amp1={}, Amp2={}, Attenuation={} dB".format(jrad, amp1, amp2, att))
           cmd = usrp_rxfe_setup_command(self.usrpManager.socks[jrad], amp1, amp2, att*2) # *2 since LSB is 0.5 dB
           cmd.transmit()
           time.sleep(0.001)
           self.usrpManager.eval_client_return(cmd, jrad)


    def test_rxfe_control(self):
        """ Function that steps through all amp and att stages of the rxfe board """

        self.logger.warning("Starting RXFE test: walk through all bits:")

        testParSet = [[False, False, 0], [True, False, 0], [True, True, 0], [False, True, 0], [True, True, 31.5]] + [[False, False, 2**i/2] for i in range(6) ]

        nSets = len(testParSet)

        for iSet in range(nSets):
            amp1 = testParSet[iSet][0]
            amp2 = testParSet[iSet][1]
            att  = testParSet[iSet][2]

            # loop over jradar
            for jrad in range(self.N_RADARs):
               cmd = usrp_rxfe_setup_command(self.usrpManager.socks[jrad], amp1, amp2, att*2) # *2 since LSB is 0.5 dB
               cmd.transmit()
               time.sleep(0.001)
               cmd.client_return()
               self.logger.warning("Current settings: Amp1={}, Amp2={}, Attenuation={} dB".format(amp1, amp2, att))
               #npt = input('  Press Enter for next change... ')
               time.sleep(2)

        print("Finished testing RXFE!")


    def cuda_init(self):
        #time.sleep(.05)

        # connect cuda_driver servers

        cuda_driver_port = int(self.ini_network_settings['CUDADriverPort'])

        cuda_driver_hostnames = [
           [name.strip() for name in self.ini_network_settings['CUDADriverHostnames_R1'].split(',')],
           [name.strip() for name in self.ini_network_settings['CUDADriverHostnames_R2'].split(',')]
        ]

        cuda_driver_socks = []
        for jrad in range(self.N_RADARs):
           socks = []
           for c in cuda_driver_hostnames[jrad]:
              try:
                 self.logger.debug('radar {} connecting to cuda driver on {}:{}'.format(jrad,c, cuda_driver_port))
                 cudasock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                 cudasock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                 cudasock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
                 cudasock.connect((c, cuda_driver_port))

                 socks.append(cudasock)
              except ConnectionRefusedError:
                 self.logger.error("cuda server connection failed on {}".format(c))
                 sys.exit(1)

           cuda_driver_socks.append(socks)

        if len(cuda_driver_socks) == 0:
           self.logger.error("No cuda connection available. Exiting usrp_server")
           sys.exit(1)

        self.cudasocks = cuda_driver_socks


    def setup(self, chan):
        pass


    def initialize_channel(RHM):
        """ Adds first period of channel for new channel or after CS_INACTIVE. Also appends channel to RHM.channels if not already done."""
        wait_start_time = time.time()
        while (time.time() - wait_start_time < 0.1) and (RHM.nControlPrograms > np.sum([len(RHM.channels[jr]) for jr in range(RHM.N_RADARs)])+ len(RHM.newChannelList) ):
           RHM.logger.debug("initialize_channel: waiting 10 ms for other control program to SET_PARAMETER")
           time.sleep(0.010)

        RHM.set_par_semaphore.acquire()
        RHM.logger.debug("start initialize_channel")
        newChannelList = RHM.newChannelList.copy()  # make a copy in case another channel is added during this function call

        nChannelsNew = 0
        for ch2add in newChannelList:
           if ch2add not in np.concatenate(RHM.channels).tolist():
              nChannelsNew += 1

        RHM._calc_period_details(newChannels=newChannelList) # TODO only if this is first channel?
        for channel in newChannelList:

            RHM.apply_channel_scaling(channel.rnum, nChannelsWillBeAdded=nChannelsNew)

            # CUDA_ADD_CHANNEL in first period
            RHM.logger.debug('RADAR number: {}'.format(channel.rnum))
            seq = channel.get_current_sequence()
            RHM.logger.debug('SEQUENCE: {}'.format(seq))
            cmd = cuda_add_channel_command(RHM.cudasocks[channel.rnum], sequence=seq, swing = channel.swingManager.activeSwing)
            RHM.logger.debug('calling CUDA_ADD_CHANNEL at initialize_channel() (rnum {} cnum {}, swing {}, beam {})'.format(channel.rnum, channel.cnum, channel.swingManager.activeSwing, channel.scanManager.current_beam))
            cmd.transmit()
            time.sleep(0.001)
            cmd.client_return()
            if channel.active_state == CS_INACTIVE:
               RHM.logger.debug("initialize_channel() is setting radar {} ch {} swing {} from CS_INACTIVE to CS_READY".format(channel.rnum, channel.cnum, channel.swingManager.activeSwing))
               channel.active_state = CS_READY # channel not really ready until CUDA_GENERATE, but there will be no trigger in parallel to this function
            else:
               RHM.logger.debug("initialize_channel() radar {} ch {} state stays {}".format(channel.rnum, channel.cnum, channel.active_state))

            if channel not in RHM.channels[channel.rnum]:
               RHM.channels[channel.rnum].append(channel)
               RHM.logger.debug("Adding channel {} to RHM.channels on radar {}".format(channel.cnum,channel.rnum))
            RHM.newChannelList.remove(channel)

        # CUDA_GENERATE for first period
        RHM.logger.debug('start CUDA_GENERATE_PULSE swing {} (1st period)'.format(RHM.swingManager.activeSwing))

        for jrad in range(RHM.N_RADARs):
           if radar_active[jrad]:
              RHM.logger.debug('CUDA_GENERATE_PULSE jrad {} socket {}'.format(jrad,RHM.cudasocks[jrad]))
              cmd = cuda_generate_pulse_command(RHM.cudasocks[jrad], RHM.swingManager.activeSwing, RHM.mixingFreqManager.current_mixing_freq[jrad]*1000)
              cmd.transmit()
              time.sleep(0.001)
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

        if channelObject in self.active_channels[channelObject.rnum]:
           self.active_channels[channelObject.rnum].remove(channelObject)

        if channelObject in self.newChannelList:
           self.newChannelList.remove(channelObject)

        if channelObject in self.channels[channelObject.rnum]:
       # this is only called if something went wrong or crtl program quit => so don't care about channel states ?
       #    # don't delete channel in middle of trigger, pretrigger, or ....
       #     channelObject._waitForState([CS_READY, CS_INACTIVE])
            self.logger.info('unregister_channel_from_HardwareManager() removing channel {} from HardwareManager'.format(self.channels[channelObject.rnum].index(channelObject)))
            self.channels[channelObject.rnum].remove(channelObject)

            # remove channel from cuda
            self.logger.debug('send CUDA_REMOVE_CHANNEL')
            for iSwing in range(nSwings):
               try:
                   cmd = cuda_remove_channel_command(self.cudasocks[channelObject.rnum], sequence=channelObject.get_current_sequence(remove_channel=True), swing = iSwing)
                   cmd.transmit()
                   time.sleep(0.001)
                   cmd.client_return()
               except AttributeError:
                   # catch errors where channel.getSequence() fails because npulses_per_sequence is uninitialized
                   # TODO: discover why this happens..
                   self.logger.error('unregister_channel_from_HardwareManager() failed to remove channel from HardwareManager')

            self.logger.debug('RHM:unregister_channel_from_HardwareManager {} channels left'.format(len(self.channels[channelObject.rnum])))

            self.nRegisteredChannels -= 1
            if self.nRegisteredChannels <= 0:
                self.nRegisteredChannels = 0
                self.commonChannelParameter = {}
                radar_active[:] = False

        else:
            self.logger.warning('unregister_channel_from_HardwareManager() channel already deleted')


    def exit(self):
        self.logger.warning("Entering RadarHardwareManager.exit() for clean exit")
        self.disconnect_driver_and_clean_up()
     #   self.logger.debug("Closing client socket...")
     #   self.client_sock.close()
        self.logger.debug("Setting exit flag...")
        self.exit_usrp_server = True

        # connect to ros port overcome blocking
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        sock.connect((USRP_SERVER_HOST, RMSG_PORT))
        sock.close()
        # sys.exit(0)


    def disconnect_driver_and_clean_up(self):
        self.logger.warning("Entering RadarHardwareManager.disconnect_driver_and_clean_up()")
        # clean up and exit
        self.client_sock.close()

        if hasattr(self, 'usrpManager'):

           # loop over jradar
           for jrad in range(self.N_RADARs):
              if radar_active[jrad]:
                 cmd = usrp_exit_command(self.usrpManager.socks[jrad])
                 cmd.transmit()
                 for sock in self.usrpManager.socks[jrad]:
                    sock.close()

        if hasattr(self, 'cudasocks'):
           for jrad in range(self.N_RADARs):
              cmd = cuda_exit_command(self.cudasocks[jrad])
              cmd.transmit()

              for sock in self.cudasocks[jrad]:
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

        self.logger.debug("_calc_period_details: samplingRate_bb={}, number_of_samples={}".format(self.commonChannelParameter['baseband_samplerate'], self.commonChannelParameter['number_of_samples']))
        # calculate the pulse sequence period with padding
    #    nSamples_per_sequence = self.commonChannelParameter['number_of_samples'] + int(PULSE_SEQUENCE_PADDING_TIME * self.commonChannelParameter['baseband_samplerate'])
        nSamples_per_sequence = max(self.commonChannelParameter['number_of_samples']+2, int(np.ceil((self.commonChannelParameter['pulse_sequence_offsets_vector'][-1]+ self.commonChannelParameter['pulseLength']/1e6 + PULSE_SEQUENCE_PADDING_TIME) * self.commonChannelParameter['baseband_samplerate'])))
        pulse_sequence_period = nSamples_per_sequence / self.commonChannelParameter['baseband_samplerate']

        self.logger.debug("nSamples_per_sequence: {}, pulse_sequence_period: {}".format(nSamples_per_sequence, pulse_sequence_period))

        self.logger.debug("self.starttime_period: {}".format(self.starttime_period))
        self.logger.debug("self.commonChannelParameter['integration_period_duration']: {}".format(self.commonChannelParameter['integration_period_duration']))

        # to find out how much time is available in an integration period for pulse sequences, subtract out startup delay
       # transmitting_time_left = self.starttime_period + self.commonChannelParameter['integration_period_duration'] - time.time() - self.integration_time_manager.get_usrp_delay_time() - self.integration_time_manager.estimate_calc_time()

        time_now = time.time()
        nSec_to_end_of_period = self.commonChannelParameter['integration_period_duration'] - time_now + self.starttime_period
        # reduce time if there is a scan boundary
        for ch in np.concatenate(self.channels).tolist()+newChannels:
            nSec_to_end_of_period = min(nSec_to_end_of_period, ch.scanManager.get_nSec_to_scan_boundary(time_now))

        transmitting_time_left = nSec_to_end_of_period - self.integration_time_manager.estimate_calc_time() - self.integration_time_manager.get_usrp_delay_time()
        self.logger.debug("transmitting time left: {}".format(transmitting_time_left))

        if transmitting_time_left <= 0:
            transmitting_time_left = 0
            self.logger.warning("no time is left in integration period for sampling!")

        # calculate the number of pulse sequences that fit in the available time within an integration period
        nSequences_per_period = int(transmitting_time_left / pulse_sequence_period)
        nSequences_per_period_max = int((self.commonChannelParameter['integration_period_duration']
                                         - self.integration_time_manager.get_usrp_delay_time()
                                         - self.integration_time_manager.estimate_calc_time() ) / pulse_sequence_period)
        ### sampling_duration = pulse_sequence_period * nSequences_per_period   # just record full number of sequences

        # calculate the number of RF transmit and receive samples
        downsamplingRates = self.commonChannelParameter["downsample_rates"]
        if nSequences_per_period == 0:
           nSamples_per_sequence_if = nSequences_per_period
           num_requested_rx_samples = nSequences_per_period
        else:
           ntap_rf_if_factor = int(self.ini_dsp_info['ntap_rf_if_factor'])
           ntap_if_bb_factor = int(self.ini_dsp_info['ntap_if_bb_factor'])

           # assumes fixed nTaps for filter = 2*downsampling
           nSamples_per_sequence_if = int(downsamplingRates[1])
                                        * ((nSamples_per_sequence*nSequences_per_period) - 1 )
                                        + int(downsamplingRates[1]*ntap_if_bb_factor)
           num_requested_rx_samples = int(downsamplingRates[0])
                                        * (nSamples_per_sequence_if                      - 1 )
                                        + int(downsamplingRates[0]*ntap_rf_if_factor)

        self.logger.debug("RFIFRATE: {}, IFBBRATE: {}, nSamples_per_sequence_if: {}, nSamples_per_sequence: {}, nSequences_per_period: {}, NTapsRX_ifbb: {}, NTapsRX_rfif: {}".format( \
                downsamplingRates[0], downsamplingRates[1], nSamples_per_sequence_if, nSamples_per_sequence, nSequences_per_period, downsamplingRates[0]*2, downsamplingRates[1]*2))

        self.logger.info("Effective integration time: {:0.3f}s = {} sequences ({}s) swing {}".format(num_requested_rx_samples /self.usrp_rf_tx_rate, nSequences_per_period, self.commonChannelParameter['integration_period_duration'], self.swingManager.activeSwing))

        self.nsamples_per_sequence     = pulse_sequence_period * self.usrp_rf_tx_rate

        # then calculate sample indices at which pulse sequences start within a pulse sequence
        nPulses_per_sequence           = self.commonChannelParameter['npulses_per_sequence']
        pulse_sequence_offsets_samples = self.commonChannelParameter['pulse_sequence_offsets_vector'] * self.usrp_rf_tx_rate

        # then, calculate sample indices at which pulses start within an integration period (all possible for cuda, only trasmitted for usrp_driver)
        # TODO if the pulse offsets change in one scan, this has to be changed (calc next pulse period for cuda, current for usrp driver)
        all_possible_integration_period_pulse_sample_offsets = np.zeros(nPulses_per_sequence * nSequences_per_period_max, dtype=np.uint64)
        for iSequence in range(nSequences_per_period_max):
            for iPulse in range(nPulses_per_sequence):
                all_possible_integration_period_pulse_sample_offsets[iSequence * nPulses_per_sequence + iPulse] = np.round(iSequence * self.nsamples_per_sequence + pulse_sequence_offsets_samples[iPulse])
        integration_period_pulse_sample_offsets = all_possible_integration_period_pulse_sample_offsets[:nSequences_per_period*nPulses_per_sequence]

        self.all_possible_integration_period_pulse_sample_offsets = all_possible_integration_period_pulse_sample_offsets
        self.nPulses_per_integration_period = nPulses_per_sequence * nSequences_per_period

        if True:
           self.logger.debug(" > nSamples_per_sequence (bb) {}".format(nSamples_per_sequence))
           self.logger.debug(" > nSamples_per_sequence (if) {}".format(nSamples_per_sequence_if))
           self.logger.debug(" > num_requested_rx_samples {}".format(num_requested_rx_samples))
           self.logger.debug(" > nSequences_per_period {}".format(nSequences_per_period))

        # inform all channels with the number of pulses per integration period
        for ch in np.concatenate(self.channels).tolist()+newChannels:
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
        self.integration_time_manager.started_trigger_next() #checks time for last integration and sets start time of current integration

        #self.apply_channel_scaling() # currently does nothing

        self._calc_period_details()
        trigger_next_period = self.nSequences_per_period != 0 # don't triger if no time left

        nSamples_per_pulse = int(np.round((self.commonChannelParameter['pulseLength'] / 1e6 * self.usrp_rf_tx_rate) + 2 * (self.commonChannelParameter['tr_to_pulse_delay']/1e6 * self.usrp_rf_tx_rate)))
        for ch in np.concatenate(self.channels).tolist():
            self.logger.debug('rnum {} cnum {}: tfreq={}, rfreq={}, usrp_center={} rx_rate={}MHz'.format(ch.rnum, ch.cnum, ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq'], self.mixingFreqManager.current_mixing_freq[ch.rnum], self.usrp_rf_tx_rate/1e6))
            if not (ch.ctrlprm_struct.payload['tfreq'] == ch.ctrlprm_struct.payload['rfreq']):
                self.logger.warning('tfreq (={}) != rfreq (={}) !'.format(ch.ctrlprm_struct.payload['tfreq'], ch.ctrlprm_struct.payload['rfreq']))

        # START LOOP OVER RADARS

        # block to fill current swing

        # look for one active channel
        transmittingChannelAvailable = [False for j in range(self.N_RADARs)]
        for jrad in range(self.N_RADARs):
           self.logger.debug("in trigger_next_swing() checking if any transmitting channels available on radar {}".format(jrad))
           if not radar_active[jrad]:
              continue

           for tmpChannel in self.channels[jrad]:
              if (tmpChannel is not None) and (not tmpChannel.scanManager.isLastPeriod):
                 channel = tmpChannel
                 transmittingChannelAvailable[jrad] = True
                 break

           self.swingManager.nextSwingToTrigger = self.swingManager.processingSwing
           self.logger.debug("setting nextSwingToTrigger to swing {}".format(self.swingManager.nextSwingToTrigger))

           if transmittingChannelAvailable[jrad]:
              if trigger_next_period:
                 # USRP SETUP
                 self.logger.debug('triggering period no {}, swing {}'.format(channel.scanManager.current_period + 1 - channel.scanManager.isPrePeriod, self.swingManager.activeSwing))
                 self.logger.debug("start USRP_SETUP")
                 if self.auto_collect_clrfrq_after_rx:
                    nSamples_clear_freq = self.clearFreqRawDataManager.number_of_samples
                    nSamples_pause_before_autoclearfreq = int(self.auto_pause_time * self.usrp_rf_rx_rate)
                    collect_auto_clear_freq_samples = True
                    auto_clear_freq_meta_data = dict(sampling_rate=self.usrp_rf_rx_rate, center_freq=self.mixingFreqManager.current_mixing_freq[jrad]*1000)
                 else:
                    nSamples_clear_freq = 0
                    nSamples_pause_before_autoclearfreq = 0
                    collect_auto_clear_freq_samples = False
                    auto_clear_freq_meta_data = None

                 self.logger.debug('rnum {} usrp_setup pars: pulses:{}  total_samples: {} nsamples_pause: {} nsamples_clearfreq: {} nsamples_per_pulse: {}'.format(jrad, self.nPulses_per_integration_period, channel.nrf_rx_samples_per_integration_period, nSamples_pause_before_autoclearfreq, nSamples_clear_freq, nSamples_per_pulse))

                 self.usrp_setup_semaphore.acquire()

                 cmd = usrp_setup_command(self.usrpManager.socks[jrad], \
                                          self.mixingFreqManager.current_mixing_freq[jrad]*1000, \
                                          self.mixingFreqManager.current_mixing_freq[jrad]*1000, \
                                          self.usrp_rf_tx_rate, self.usrp_rf_rx_rate, \
                                          self.nPulses_per_integration_period, \
                                          channel.nrf_rx_samples_per_integration_period, \
                                          nSamples_pause_before_autoclearfreq, nSamples_clear_freq, \
                                          nSamples_per_pulse, \
                                          channel.integration_period_pulse_sample_offsets, \
                                          self.swingManager.activeSwing)
                 cmd.transmit()
                 time.sleep(0.001)

                 # rxrate,rxfreq,txrate,txfreq = cmd.receive_settings(self.usrpManager.socks[jrad][0])
                 rxrate,rxfreq,txrate,txfreq = cmd.receive_settings()
                 cmd.client_return()

                 self.logger.debug("USRP_SETUP received  rxrate {} rxfreq {} txrate {} txfreq {}".format(rxrate,rxfreq,txrate,txfreq))

                 # self.mixingFreqManager.current_mixing_freq[jrad] = rxfreq/1000

                 try:
                    # give non-local usrps some extra time to respond
                    if not self.usrpManager.socks[jrad][0].getpeername()[0] == '127.0.0.1':
                       time.sleep(0.002)
                 except:
                    self.logger.debug("No USRPs for radar {}".format(jrad))

                 # self.usrpManager.eval_client_return(cmd, jrad)
                 self.logger.debug("end USRP_SETUP")

                 self.usrp_setup_semaphore.release()

                 # wait if periods should be time synchronized
                 for tmpChannel in self.channels[jrad]:
                    if (tmpChannel is not None) and (not tmpChannel.scanManager.isLastPeriod):
                       tmpChannel.scanManager.wait_for_next_trigger()

        # USRP_TRIGGER - END OF SETUP NEW LOOP OVER RADARS
        self.logger.debug("start USRP_GET_TIME")

        cmd = usrp_get_time_command(self.usrpManager.socks[0][0]) # grab current usrp time from one usrp_driver
        cmd.transmit()
        time.sleep(0.001)

        # TODO: tag time using a better source? this will have a few hundred microseconds of uncertainty
        # maybe measure offset between usrp time and computer clock time somewhere, then calculate from there
        usrp_time = cmd.recv_time(self.usrpManager.socks[0][0])
        cmd.client_return()

        self.logger.debug("end USRP_GET_TIME")

        trigger_time = usrp_time + self.integration_time_manager.get_usrp_delay_time()

        usrp_integration_period_start_clock_time = time.time() + self.integration_time_manager.get_usrp_delay_time()

        for jrad in range(self.N_RADARs):

           if not radar_active[jrad]:
              continue

           for tmpChannel in self.channels[jrad]:
              if (tmpChannel is not None):
                 channel = tmpChannel
                 break

           nSamples_rx_requested_of_last_trigger = channel.nrf_rx_samples_per_integration_period

           if transmittingChannelAvailable[jrad] and trigger_next_period and self.auto_collect_clrfrq_after_rx:
              auto_clear_freq_meta_data['record_time'] = usrp_integration_period_start_clock_time + (nSamples_rx_requested_of_last_trigger / self.usrp_rf_rx_rate)

           if transmittingChannelAvailable[jrad]:
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
              for channel in self.channels[jrad]:
                 if (channel is not None) and (not channel.scanManager.isLastPeriod):
                    resultDict['nbb_rx_samples_per_sequence'] = channel.nbb_rx_samples_per_sequence
                    resultDict['pulse_lens']                  = channel.pulse_lens
                    channel.resultDict_list.insert(0,copy.deepcopy(resultDict))

              if trigger_next_period:
                 # broadcast the start of the next integration period to all usrp
                 self.logger.debug('start USRP_TRIGGER')

                 # trigger_time = usrp_time + self.integration_time_manager.get_usrp_delay_time()
                 cmd = usrp_trigger_pulse_command(self.usrpManager.socks[jrad], trigger_time, self.commonChannelParameter['tr_to_pulse_delay'], self.swingManager.activeSwing)
                 self.logger.debug('sending trigger pulse command for swing {}'.format(self.swingManager.activeSwing))
                 cmd.transmit()
                 self.logger.debug('radar {} current usrp time: {}, trigger time of: {}'.format(jrad, usrp_time, trigger_time))
              else:
                 self.logger.info("No time left, not triggering this swing.")

              # set state of channel to CS_PROCESSING
              for ch in self.channels[jrad]:
                 if ch.active_state == CS_TRIGGER:
                    ch.active_state = CS_PROCESSING
                    ch.logger.debug("Changing active channel state from CS_TRIGGER to CS_PROCESSING (rnum: {}, cnum: {}, swing {}, period {})".format(ch.rnum, ch.cnum, self.swingManager.activeSwing, ch.scanManager.current_period))
              if trigger_next_period:
                 self.logger.debug('waiting for trigger return')
                 returns = self.usrpManager.eval_client_return(cmd, jrad)

                 if TRIGGER_BUSY in returns:
                    self.logger.error('could not trigger, usrp driver is busy')
                    # pdb.set_trace()
                 self.logger.debug('end USRP_TRIGGER')
           else:
              self.logger.debug('No transmitting channels available on radar {}. Skipping USRP_TRIGGER'.format(jrad))

        # Block to process last swing

        for jrad in range(self.N_RADARs):
           if not radar_active[jrad]:
              continue

           allProcessingChannelStates = [ch.processing_state for ch in self.channels[jrad]]
           self.logger.debug("jrad {} allProcessingChannelStates: {}".format(jrad,allProcessingChannelStates))
           if self.processing_swing_invalid: # TODO check if this works (in control program or data files)
              self.logger.warning("Last swing has been invalid. Preparing 0 sequences to transmit")
              for iChannel, channel in enumerate(self.channels[jrad]):
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
                 chResExportList = [ ch.resultDict_list[-1] for ch in np.concatenate(self.channels).tolist() if ch.processing_state == CS_PROCESSING]
                 with open('tmpRawData.pkl', 'wb') as f:
                    pickle.dump([[], [], chResExportList, self.antenna_idx_list_main[0], self.antenna_idx_list_back[0]], f, pickle.HIGHEST_PROTOCOL)
                    pickle.dump([[], [], chResExportList, self.antenna_idx_list_main[1], self.antenna_idx_list_back[1]], f, pickle.HIGHEST_PROTOCOL)
                 os.rename("tmpRawData.pkl", "liveRawData.pkl")
                 os.remove("./bufferLiveData.flag")

              self.processing_swing_invalid = False
           else:
              if CS_PROCESSING in allProcessingChannelStates:
                 # CUDA_GET_DATA
                 self.logger.debug('start CUDA_GET_DATA radar {}'.format(jrad))
                 cmd = cuda_get_data_command(self.cudasocks[jrad], self.swingManager.processingSwing)
                 cmd.transmit()

                 nMainAntennas = len(self.antenna_idx_list_main[jrad])
                 nBackAntennas = len(self.antenna_idx_list_back[jrad])
                 main_samples = None

                 for cudasock in self.cudasocks[jrad]:
                    nAntennas = recv_dtype(cudasock, np.uint32)
                    for iChannel,channel in enumerate(self.channels[jrad]):

                       if channel.processing_state == CS_PROCESSING:
                          channel.logger.debug("Receiving {} antennas for channel {} on radar {}".format(nAntennas, channel.cnum, channel.rnum))
                          transmit_dtype(cudasock, channel.cnum, np.int32)
                          time.sleep(0.001)

                          for iAntenna in range(nAntennas):
                             antIdx = recv_dtype(cudasock, np.uint16)
                             nSamples_bb = int(recv_dtype(cudasock, np.uint32) / 2)
                             if main_samples is None:
                                main_samples = np.zeros((len(np.concatenate(self.channels).tolist()), nMainAntennas, nSamples_bb), dtype=np.complex64)
                                back_samples = np.zeros((len(np.concatenate(self.channels).tolist()), nBackAntennas, nSamples_bb), dtype=np.complex64)

                             samples = recv_dtype(cudasock, np.float32, nSamples_bb * 2)
                             samples = (samples[0::2] + 1j * samples[1::2]) * self.scaling_factor_rx_bb[jrad] # unpacked interleaved i/q

                             if antIdx in self.antenna_idx_list_main[jrad]:
                                iAntenna = self.antenna_idx_list_main[jrad].index(antIdx)
                                main_samples[iChannel][iAntenna] = samples[:]
                             elif antIdx in self.antenna_idx_list_back[jrad]:
                                iAntenna = self.antenna_idx_list_back[jrad].index(antIdx)
                                back_samples[iChannel][iAntenna] = samples[:]
                             else:
                                self.logger.error("Cuda transmitted antenna ({}) that is not in main array list ({}) and back array list ({}). (Maybe different antenna definitions in usrp_config.ini on both computers?)".format(antIdx, self.antenna_idx_list_main[jrad], self.antenna_idx_list_back[jrad]))
                                sys.exit(1)
                       else:
                          channel.logger.debug("Receiving NOTHING for channel {} because processing_state is {}".format(channel.cnum, channel.processing_state))

                    transmit_dtype(cudasock, -1, np.int32) # to end transfer process

                 cmd.client_return()
                 self.logger.debug('end CUDA_GET_DATA')

                 # BEAMFORMING
                 self.logger.debug('start rx beamforming')
                 antenna_scale_factors = self.calc_normalize_and_mute_factors(jrad, main_samples, back_samples)

                 beamformed_main_samples, beamformed_back_samples = self.calc_beamforming(jrad, main_samples, back_samples, antenna_scale_factors)
                 for iChannel, channel in enumerate(self.channels[jrad]):

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
                          # for iChannel, channel in enumerate(self.channels):
                          channel.bb_export = dict()
                          channel.bb_export["main_samples"] = main_samples[iChannel]
                          channel.bb_export["back_samples"] = back_samples[iChannel]
                          channel.bb_export["nMainAntennas"] = nMainAntennas
                          channel.bb_export["nBackAntennas"] = nBackAntennas
                          channel.bb_export['number_of_samples'] = channel.resultDict_list[-1]['number_of_samples']
                          channel.bb_export['number_of_sequences'] = channel.resultDict_list[-1]['npulses_per_sequence']
                          channel.bb_export["antenna_list"] = self.antenna_idx_list_main[jrad] + self.antenna_idx_list_back[jrad]
                          channel.bb_export['nSamples'] = nSamples_bb
                          channel.bb_export['nSequences_per_period'] = channel.resultDict_list[-1]['nSequences_per_period']
                          channel.bb_export['sequence_start_time_secs'] = channel.resultDict_list[-1]['sequence_start_time_secs']
                          channel.bb_export['sequence_start_time_usecs'] = channel.resultDict_list[-1]['sequence_start_time_usecs']
                          channel.bb_export['nbb_rx_samples_per_sequence'] = channel.resultDict_list[-1]['nbb_rx_samples_per_sequence']

                          channel.write_bb_data()

                 self.logger.debug('end rx beamforming')

                 # save BB samples if usrp live view is active
                 if os.path.isfile("./bufferLiveData.flag"):
                    self.logger.info("Buffering raw data to disk.")
                    chResExportList = [ ch.resultDict_list[-1] for ch in np.concatenate(self.channels).tolist() if ch.processing_state == CS_PROCESSING]
                    with open('tmpRawData.pkl', 'wb') as f:
                       pickle.dump([main_samples, back_samples,chResExportList, self.antenna_idx_list_main[jrad], self.antenna_idx_list_back[jrad]], f, pickle.HIGHEST_PROTOCOL)
                       os.rename("tmpRawData.pkl", "liveRawData.pkl")
                       os.remove("./bufferLiveData.flag")

                 # save IF raw data
                 for channel in self.channels[jrad]:
                    if channel.processing_state == CS_PROCESSING and os.path.isfile("/collect.if.{:c}".format(96+channel.cnum)):
                       channel.logger.warning("Channel {} saving raw IF samples.".format(channel.cnum))
                       channel.get_if_data()
                       channel.write_if_data()

              else:
                 self.logger.debug('No processing channels available for radar {}. Skipping CUDA_GET_DATA and rx beamforming'.format(jrad))

        # PERIOD FINISHED
        self.next_period_RHM()
        self.clearFreqRawDataManager.reset_occupied_freqs()

        # update (next) states
        for iChannel, channel in enumerate(np.concatenate(self.channels).tolist()):
           self.logger.debug("Checking radar {} channel {} to see if it is processing: state: {}".format(channel.rnum,channel.cnum,channel.processing_state))
           if channel.processing_state is CS_PROCESSING:

              # determine next state after samples are returned to control program
              if channel.scanManager.isPostLast: # or channel.scanManager.isForelastPeriod:
                 channel.next_processing_state = CS_INACTIVE
                 channel.active_state          = CS_INACTIVE
                 channel.next_active_state     = CS_INACTIVE
                 channel.logger.debug('last period finished, setting radar {} cnum {} active and next processing state to CS_INACTIVE'.format(channel.rnum,channel.cnum))
              elif channel.scanManager.isLastPeriod:
                 channel.next_processing_state = CS_LAST_SWING
                 channel.logger.debug('Setting radar {} channel {} processing state to CS_LAST_SWING'.format(channel.rnum,channel.cnum))
              else:
                 channel.next_processing_state = CS_READY
              channel.logger.debug("Switching next processing state (swing {}) of radar {} cnum {} to {}".format(self.swingManager.processingSwing, channel.rnum, channel.cnum, channel.next_processing_state))

              channel.logger.debug("Switching processing state (swing {}) state of radar {} cnum {} from CS_PROCESSING to CS_SAMPLES_READY".format(self.swingManager.processingSwing, channel.rnum, channel.cnum))
              channel.processing_state = CS_SAMPLES_READY

        # CUDA_ADD & CUDA_GENERATE for processingSwing
        for channel in np.concatenate(self.channels).tolist():
           self.logger.debug("Checking radar {} channel {} to see if it should be added".format(channel.rnum,channel.cnum))
           if channel.scanManager.isLastPeriod: # or channel.scanManager.isForelastPeriod:
               self.last_period[channel.rnum] = True
               self.logger.debug("start CUDA_REMOVE_CHANNEL")
               cmd = cuda_remove_channel_command(self.cudasocks[channel.rnum], sequence=channel.get_current_sequence(remove_channel=True), swing = self.swingManager.processingSwing)
               self.logger.debug('send CUDA_REMOVE_CHANNEL (rnum {} cnum {}, swing {})'.format(channel.rnum, channel.cnum, self.swingManager.processingSwing))
               cmd.transmit()
               cmd.client_return()
               self.logger.debug("end CUDA_REMOVE_CHANNEL")
               if channel.scanManager.isPostLast:

                  if channel in self.channels[channel.rnum]:
                     self.channels[channel.rnum].remove(channel)
                     self.nRegisteredChannels -= 1
                     if self.nRegisteredChannels <= 0:
                        self.nRegisteredChannels = 0
                        self.commonChannelParameter = {}
                        radar_active[:] = False
                  if channel in self.active_channels[channel.rnum]:
                     self.active_channels[channel.rnum].remove(channel)

           else:
               self.last_period[channel.rnum] = False
               if channel.active:
                  self.logger.debug("start CUDA_ADD_CHANNEL")
                  seq = channel.get_next_sequence()
                  self.logger.debug("socket info {}".format(self.cudasocks[channel.rnum]))
                  cmd = cuda_add_channel_command(self.cudasocks[channel.rnum], sequence=seq, swing = self.swingManager.processingSwing)
                  self.logger.debug('send CUDA_ADD_CHANNEL (rnum {}, cnum {}, swing {}, beam {})'.format(channel.rnum, channel.cnum, self.swingManager.processingSwing, channel.scanManager.next_beam))
                  cmd.transmit()
                  cmd.client_return()
                  self.logger.debug("end CUDA_ADD_CHANNEL")

                  if channel.processing_state == CS_INACTIVE: # first use of swing 1
                      channel.processing_state = CS_READY
                      channel.logger.debug("Switching processing state (swing {}) state of cnum {} to CS_READY (first use of swing)".format(self.swingManager.processingSwing, channel.cnum))
               else:
                  self.logger.debug("radar {} ch {}: channel not active => not calling CUDA_ADD".format(channel.rnum, channel.cnum))
                  self.logger.error("When is this happening and is this okay???")

        # CUDA_GENERATE for next period
        self.logger.debug('start CUDA_GENERATE_PULSE')
        for jrad in range(self.N_RADARs):
           if self.last_period[jrad]:
              self.logger.debug('skipping CUDA_GENERATE_PULSE jrad {} (is last period)'.format(jrad))
           elif radar_active[jrad]:
              self.logger.debug('start CUDA_GENERATE_PULSE jrad {}  socket {}'.format(jrad,self.cudasocks[jrad]))
              cmd = cuda_generate_pulse_command(self.cudasocks[jrad], self.swingManager.processingSwing,
                                                self.mixingFreqManager.current_mixing_freq[jrad]*1000)
              cmd.transmit()
              cmd.client_return()
        self.logger.debug('end CUDA_GENERATE_PULSE')

        all_usrps_report_failure = True
        for jrad in range(self.N_RADARs):
           if not radar_active[jrad]:
              continue

           if transmittingChannelAvailable[jrad] and trigger_next_period:
              # USRP_READY_DATA for activeSwing
              self.logger.debug('start USRP_READY_DATA')

              self.logger.debug("socks: {}".format(self.usrpManager.socks[jrad]))

              cmd = usrp_ready_data_command(self.usrpManager.socks[jrad], self.swingManager.activeSwing)
              cmd.transmit()

              # give non-local usrps some extra time to respond
              if self.usrpManager.socks[jrad][0].getpeername()[0] == '127.0.0.1':
                 time.sleep(0.002)
              else:
                 time.sleep(0.004)

              # check status of usrp drivers
              self.logger.debug('start receiving all USRP status for radar {}'.format(jrad))
              payloadList = self.usrpManager.eval_client_return(cmd, jrad, fcn=cmd.receive_all_metadata)
              self.logger.debug('end receiving all USRP status')

              antenna_list_offset = 0
              for iUSRP, ready_return in enumerate(payloadList):
                 if ready_return == CONNECTION_ERROR:
                    self.usrpManager.fault_status[iUSRP] = True
                    self.logger.error('connection to USRP broke in GET_DATA')
                    antenna_list_offset += 1
                 else:
                    rx_status = ready_return['status']
                    if rx_status < 0:
                       rx_error_codes = dict(ERROR_CODE_NONE = 0x0, ERROR_CODE_TIMEOUT = 0x1,
                                             ERROR_CODE_LATE_COMMAND = 0x2, ERROR_CODE_BROKEN_CHAIN = 0x4,
                                             ERROR_CODE_OVERFLOW = 0x8, ERROR_CODE_ALIGNMENT = 0xc,
                                             ERROR_CODE_BAD_PACKET = 0xf, WRONG_NUMBER_OF_SAMPLES = 100)

                       error_code = - rx_status
                       print_name = 'unknown'
                       if error_code % 1000 in rx_error_codes.values():
                          for err_name, err_value in rx_error_codes.items():
                             if err_value == (error_code % 1000):
                                print_name = "UHD::" + err_name
                                break
                       if error_code == 10:
                          print_name = "RX_WORKER_STREAM_TIME_ERROR"
                       # out of sequence flag adds (-) 1000 to error code
                       if error_code >= 1000:
                          print_name += " and out_of_sequence=1"
                       self.logger.error("Error: {} (code {}) occurred in rx_worker for radar {} antennas {}.".format(print_name, rx_status, jrad, self.usrpManager.antennaList_active[jrad][iUSRP-antenna_list_offset]))

                    else:
                       all_usrps_report_failure = False

                    self.usrpManager.fault_status[iUSRP] = ready_return["fault"]

                    self.logger.debug('GET_DATA rx status {}'.format(rx_status))
                    if rx_status != 2:
                       self.logger.error('USRP driver status {} in GET_DATA'.format(rx_status))
                       #status = USRP_DRIVER_ERROR # TODO: understand what is an error here..

              self.usrpManager.watchdog(all_usrps_report_failure)

              self.logger.debug('start waiting for USRP_DATA return')
              self.usrpManager.eval_client_return(cmd, jrad)
              self.logger.debug('end waiting for USRP_DATA return')

              self.logger.debug('end USRP_READY_DATA')

        # SWITCH SWINGS
        self.swingManager.switch_swings()
        self.logger.debug('switching swings to: active={}, processing={}'.format(self.swingManager.activeSwing, self.swingManager.processingSwing))

        # START CUDA_PROCESS
        for jrad in range(self.N_RADARs):
           if transmittingChannelAvailable[jrad]:
              if trigger_next_period:
                 # CUDA_PROCESS for processingSwing
                 self.logger.debug('start CUDA_PROCESS')
                 self.logger.debug("cuda process radar {} active {}".format(jrad,radar_active[jrad]))
                 if radar_active[jrad]:
                    self.logger.debug("cuda process radar {} socks {}".format(jrad,self.cudasocks[jrad]))
                    cmd = cuda_process_command(self.cudasocks[jrad], swing=self.swingManager.processingSwing, nSamples=nSamples_rx_requested_of_last_trigger)
                    cmd.transmit()
                    time.sleep(0.001)
                    cmd.client_return()

                 self.logger.debug('end CUDA_PROCESS')

              # repeat CLR_FREQ record for 2nd period (if executed for 1st)
              if self.clearFreqRawDataManager.repeat_request_for_2nd_period:
                 self.logger.debug("Setting outstanding_request for CLR_FREQ for 2nd period.")
                 self.clearFreqRawDataManager.repeat_request_for_2nd_period = False
                 self.clearFreqRawDataManager.outstanding_request[jrad] = True

              # automatic trigger of second period (without ROS:SET_READY)
              for channel in self.channels[jrad]:
                 if channel.scanManager.isFirstPeriod:
                    channel.logger.debug('setting active state (rnum {} cnum {}, swing {}) to CS_TRIGGER to start second period'.format(channel.rnum, channel.cnum, self.swingManager.activeSwing))
                    channel.active_state = CS_TRIGGER
                    channel.triggered_swing_list.insert(0, self.swingManager.nextSwingToTrigger)

                    channel.scanManager.isFirstPeriod = False

           # GET AUTO CLEAR FREQ DATA
           if transmittingChannelAvailable[jrad] and trigger_next_period and self.auto_collect_clrfrq_after_rx:
              #self.clear_search_data_semaphore.acquire()
              nSamples_clear_freq = self.clearFreqRawDataManager.number_of_samples
              self.logger.debug("Getting auto clear freq data for radar {}. nSamples_clear_freq: {}".format(jrad,nSamples_clear_freq))
              cmd = usrp_get_auto_clear_freq_command(self.usrpManager.socks[jrad],int(nSamples_clear_freq))
              cmd.transmit()

              # give non-local usrps some extra time to respond
              if self.usrpManager.socks[jrad][0].getpeername()[0] == '127.0.0.1':
                 time.sleep(0.001)
              else:
                 time.sleep(0.0025)

              antenna_list, clr_samples = cmd.recv_all()
              cmd.client_return()

              self.clear_search_data_semaphore.acquire()
              try:
                 self.logger.debug("Have auto clear freq data for radar {}. antenna_list {} len clr_samples {}".format(jrad,antenna_list,len(clr_samples[0])))
                 auto_clear_freq_meta_data['record_time'] = time.time()
                 self.clearFreqRawDataManager.update_auto_clear_freq_data(jrad, antenna_list, clr_samples, auto_clear_freq_meta_data)
              except:
                 self.logger.error("radar {}: auto clear freq error".format(jrad))
              self.clear_search_data_semaphore.release()

        if not trigger_next_period:
           self.logger.info("This swing has not been triggered, setting processing_swing_invalid.")
           self.processing_swing_invalid = True # set for next call of trigger_next_period

        self.trigger_next_function_running = False


    def next_period_RHM(self):
        ch_do_not_increase_period = []
        for ch in np.concatenate(self.channels).tolist():
            if ch is not None:
                  ch_do_not_increase_period.append(ch.scanManager.isPrePeriod or ch.scanManager.isLastPeriod)
                  ch.scanManager.period_finished()


    def apply_channel_scaling(self, rnum, nChannelsWillBeAdded=0):
        # self.gain_control_divide_by_nChannels(self, rnum, nChannelsWillBeAdded)
        self.no_channel_gain_control(rnum)


    def no_channel_gain_control(self, rnum, nChannelsWillBeAdded=0):
        self.logger.debug("No channel scaling. Global factor for all channels is: totalScalingFactor = {}".format(self.scaling_factor_tx_total[rnum]))
        for ch in np.concatenate(self.channels).tolist() + self.newChannelList:
            ch.channelScalingFactor = self.scaling_factor_tx_total[rnum]


    def gain_control_divide_by_nChannels(self, rnum, nChannelsWillBeAdded=0):
        nChannels = len(np.concatenate(self.channels).tolist()) + nChannelsWillBeAdded
        self.logger.debug("Setting channel scaling factor to: totalScalingFactor / nChannels = {}/ {}".format(self.scaling_factor_tx_total[rnum], nChannels))
        for ch in np.concatenate(self.channels).tolist() + self.newChannelList:
            ch.channelScalingFactor = self.scaling_factor_tx_total[rnum]
#            ch.channelScalingFactor = 1 / nChannels * self.scaling_factor_tx_total


    # normalize
    def calc_normalize_and_mute_factors(RHM, jrad, main_samples, back_samples):
        antenna_scale_factors = np.ones(max(RHM.antenna_idx_list_main[jrad] + RHM.antenna_idx_list_back[jrad])+1)
        for ant_to_mute in RHM.mute_antenna_list[jrad]:
            # If antenna present and on mute list, mute it
            if (max(RHM.antenna_idx_list_main[jrad] + RHM.antenna_idx_list_back[jrad]) + 1) > ant_to_mute:
                antenna_scale_factors[ant_to_mute] = 0
        antenna_scale_factors = [antenna_scale_factors for i in range(len(np.concatenate(RHM.channels).tolist()))]

        if RHM.apply_normalization[jrad]:
            RHM.logger.info("start normalizing rx samples")
            debugPlot = False
            nChannels, nAntennas_main, nSamples = main_samples.shape
            nAntennas_back = back_samples.shape[1]
            bb_samplingRate = RHM.commonChannelParameter['baseband_samplerate']
            offset = int(np.round(900e-6*bb_samplingRate))
            # channel = RHM.channels[0]
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

            for iChannel,channel in enumerate(RHM.channels[jrad]):
                var_list = []
                for iAntenna in range(nAntennas_main):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[jrad][iAntenna]]:
                        curr_variance = np.var(np.real(main_samples[iChannel][iAntenna][rx_idx]))
                    else: # don't calculate if antenna is muted
                        curr_variance = 1

                    var_list.append(curr_variance)
                    if debugPlot and iAntenna < 8:
                        plt.subplot(8,2,iAntenna*2+1)
                        plt.plot(20*np.log10(np.abs(main_samples[iChannel][iAntenna])/2**0))
                        plt.title("var = {:2.3f}".format(curr_variance))

                # back array
                for iAntenna in range(nAntennas_back):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[jrad][iAntenna]]:
                        curr_variance = np.var(np.real(back_samples[iChannel][iAntenna][rx_idx]))
                    else: # don't calculate if antenna is muted
                        curr_variance = 1
                    var_list.append(curr_variance)

                max_var= max(var_list)
                var_threshold = max_var * 10 ** (-30/10)
                RHM.logger.info("max var = {:2.3f} = {:2.3f} **2, var_threshold = {} (-30 dB)".format(max_var, np.sqrt(max_var), var_threshold))
                for iAntenna in range(nAntennas_main):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[jrad][iAntenna]]:
                        if var_list[iAntenna] > var_threshold:
                            scale_factor = np.sqrt(max_var/var_list[iAntenna]) * antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[jrad][iAntenna]]

                            antenna_scale_factors[iChannel][RHM.antenna_idx_list_main[jrad][iAntenna]] = scale_factor
                            RHM.logger.info("scaling antenna {} with factor {:}".format(RHM.antenna_idx_list_main[jrad][iAntenna], scale_factor))
                        else:
                            RHM.logger.info("not scaling antenna {} because of small variance: {} (< threshold)".format(RHM.antenna_idx_list_main[jrad][iAntenna], var_list[iAntenna]))
                            scale_factor = 1
                    else:
                        RHM.logger.info("muting antenna {} (defined in array_config.ini)".format(RHM.antenna_idx_list_main[jrad][iAntenna]))

                    if debugPlot and iAntenna < 8:
                        plt.subplot(8,2,iAntenna*2+2)
                        plt.plot(20*np.log10(np.abs(main_samples[iChannel][iAntenna])/2**0))
                        plt.title("factor: {:2.3f}".format(scale_factor))
                if debugPlot:
                   plt.show()

                # back array
                for iAntenna in range(nAntennas_back):
                    if antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[jrad][iAntenna]]:
                        if var_list[iAntenna+nAntennas_main] > var_threshold:
                            scale_factor = np.sqrt(max_var/var_list[iAntenna+nAntennas_main]) * antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[jrad][iAntenna]]

                            antenna_scale_factors[iChannel][RHM.antenna_idx_list_back[jrad][iAntenna]] = scale_factor
                            RHM.logger.info("scaling antenna {} with factor {:}".format(RHM.antenna_idx_list_back[jrad][iAntenna], scale_factor))
                        else:
                            RHM.logger.info("not scaling antenna {} because of small variance: {} (< threshold)".format(RHM.antenna_idx_list_back[jrad][iAntenna], var_list[iAntenna+nAntennas_main]))
                            scale_factor = 1
                    else:
                        RHM.logger.info("muting antenna {} (defined in array_config.ini)".format(RHM.antenna_idx_list_back[jrad][iAntenna], var_list[iAntenna+nAntennas_main]))

#                print("list var: {}".format(np.sqrt(var_list)))
            RHM.logger.info("end normalizing rx samples")

        return antenna_scale_factors


    # BEAMFORMING
    def calc_beamforming(RHM, jrad, main_samples, back_samples, antenna_scale_factors):
        nSamples = main_samples.shape[2]
        beamformed_main_samples = np.zeros((len(np.concatenate(RHM.channels).tolist()), nSamples), dtype=np.uint32)
        beamformed_back_samples = np.zeros((len(np.concatenate(RHM.channels).tolist()), nSamples), dtype=np.uint32)
        debugPlot = False

        maxInt16value = np.iinfo(np.int16).max # +32767
        minInt16value = np.iinfo(np.int16).min # -32768

        for iChannel, channel in enumerate(RHM.channels[jrad]):
            if channel.processing_state is CS_PROCESSING:
                cur_beam=channel.ctrlprm_struct.payload['rbeam']
                cur_freq=channel.ctrlprm_struct.payload['rfreq']

                # calculate beam azimuth from transmit beam number
                bmazm = calc_beam_azm_rad(RHM.array_nBeams[channel.rnum], cur_beam, RHM.array_beam_sep[channel.rnum])
                channel.logger.debug("rx beamforming: radar {} ch {}, beam {}".format(channel.rnum, channel.cnum, cur_beam))

                # calculate antenna-to-antenna phase shift for steering at a frequency
                pshift = calc_phase_increment(bmazm, cur_freq * 1000., RHM.array_x_spacing[channel.rnum])
                channel.logger.debug("rx beamforming: radar {} ch {}, frequency {}".format(channel.rnum, channel.cnum, cur_freq))

                # MAIN ARRAY
                first_pol_ant_idx = [ant_idx for ant_idx in RHM.antenna_idx_list_main[jrad] if ant_idx < 20]
                first_pol_matrix_idx = [RHM.antenna_idx_list_main[jrad].index(ant_idx) for ant_idx in first_pol_ant_idx]

                # calculate a complex number representing the phase shift for each antenna
                phasing_matrix = np.matrix([rad_to_rect(ant_idx * pshift)*antenna_scale_factors[iChannel][ant_idx] for ant_idx in first_pol_ant_idx])
                print("main")
                print(phasing_matrix)

                # # create a matrix of the antenna scale factors (debug only)
                # s_matrix = np.matrix([antenna_scale_factors[iChannel][ant_idx] for ant_idx in first_pol_ant_idx])
                # print(s_matrix)

                complex_float_samples = phasing_matrix * np.matrix(main_samples[iChannel])[first_pol_matrix_idx,:]
                real_mat = np.real(complex_float_samples)
                imag_mat = np.imag(complex_float_samples)
                abs_max_value = max(abs(real_mat).max(), abs(imag_mat).max())
                RHM.logger.info("Main array abs max_value is {:.2f} (int16_max= {}, max_value / int16_max = {})".format(abs_max_value, maxInt16value, abs_max_value / maxInt16value))

                real_mx = np.max(np.abs(real_mat))
                imag_mx = np.max(np.abs(imag_mat))
                if (real_mx > maxInt16value) or (imag_mx > maxInt16value):
                   RHM.logger.warning("Overflow error while casting main array beamformed rx samples to complex int16s.")
                   OverflowError("calc_beamforming: overflow error in casting main array data to complex int")
                   scale_value = maxInt16value/np.max([real_mx,imag_mx])
                   real_mat = scale_value*real_mat
                   imag_mat = scale_value*imag_mat

                # # check for clipping
                # if (real_mat > maxInt16value).any() or (real_mat < minInt16value).any() or (imag_mat > maxInt16value).any() or (imag_mat < minInt16value).any():
                #    RHM.logger.warning("Overflow error while casting main array beamformed rx samples to complex int16s.")
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
                # calculate a complex number representing the phase shift for each antenna
                phasing_matrix = np.matrix([rad_to_rect((ant_idx-10) * pshift)* antenna_scale_factors[iChannel][ant_idx] for ant_idx in RHM.antenna_idx_list_back[jrad]])
                print("back")
                print(phasing_matrix)

                # # create a matrix of the antenna scale factors (debug only)
                # s_matrix = np.matrix([antenna_scale_factors[iChannel][ant_idx] for ant_idx in RHM.antenna_idx_list_back[jrad]])
                # print(s_matrix)

                complex_float_samples = phasing_matrix * np.matrix(back_samples[iChannel])
                real_mat = np.real(complex_float_samples)
                imag_mat = np.imag(complex_float_samples)
                abs_max_value = max(abs(real_mat).max(), abs(imag_mat).max())
                RHM.logger.info("Back array abs max_value is {:.2f} (int16_max= {}, max_value / int16_max = {})".format(abs_max_value, maxInt16value, abs_max_value / maxInt16value))

                real_mx = np.max(np.abs(real_mat))
                imag_mx = np.max(np.abs(imag_mat))
                if (real_mx > maxInt16value) or (imag_mx > maxInt16value):
                   RHM.logger.warning("Overflow error while casting back array beamformed rx samples to complex int16s.")
                   OverflowError("calc_beamforming: overflow error in casting back array data to complex int")
                   scale_value = maxInt16value/np.max([real_mx,imag_mx])
                   real_mat = scale_value*real_mat
                   imag_mat = scale_value*imag_mat

                # # check for clipping
                # if (real_mat > maxInt16value).any() or (real_mat < minInt16value).any() or (imag_mat > maxInt16value).any() or (imag_mat < minInt16value).any():
                #    RHM.logger.warning("Overflow error while casting back array beamformed rx samples to complex int16s.")
                #    OverflowError("calc_beamforming: overflow error in casting data to complex int")
                #    real_mat = np.clip(real_mat, minInt16value, maxInt16value)
                #    imag_mat = np.clip(imag_mat, minInt16value, maxInt16value)

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
        self.logger = logging.getLogger("ChManager #{}".format(parent_RadarHardwareManager.channel_manager_consecutive_number))
        self.state      = [CS_INACTIVE, CS_INACTIVE]
        self.next_state = [CS_INACTIVE, CS_INACTIVE]

        self.ctrlprm_struct = ctrlprm_struct(self.conn)
        self.seqprm_struct  = seqprm_struct(self.conn)
        self.clrfreq_struct = clrfreqprm_struct(self.conn)
        self.rprm_struct    = rprm_struct(self.conn)
        self.dataprm_struct = dataprm_struct(self.conn)

        self.channelScalingFactor = 0
        self.rnum = 'unknown'
        self.cnum = 'unknown'
        self.resultDict_list = []

        self.swingManager = parent_RadarHardwareManager.swingManager # reference to global swingManager of RadarHardwareManager
        self.triggered_swing_list = []

        self.received_first_SETPAR = False # to handle first GET_PAR before SET_PAR


# QUICK ACCESS TO CURRENT/NEXT ACTIVE/PROCESSING STATE
    @property
    def active_state(self):
        return self.state[self.swingManager.activeSwing]


    @active_state.setter
    def active_state(self, value):
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
    def next_active_state(self, value):
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

            self.logger.debug('radar {} ch {}: waiting for command'.format(self.rnum,self.cnum))
            rmsg.receive(self.conn)
            command = chr(rmsg.payload['type'] & 0xFF) # for some reason, libtst is sending out 4 byte commands with junk..
            try:
                self.logger.debug('radar {} ch {}: received command (ROS=>USRP_Server): {}, {}'.format(self.rnum, self.cnum, command, RMSG_COMMAND_NAMES[command]))
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
                self.logger.error('rnum {} ch {}: Error while command {} ({}). Removing this channel'.format(self.rnum, self.cnum, RMSG_COMMAND_NAMES[command], command))
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
        self.logger.debug('Deleting radar {} channel {}'.format(self.rnum, self.cnum))
        RHM = self.parent_RadarHardwareManager
        RHM.unregister_channel_from_HardwareManager(self)
        cnum = self.cnum
        del self # TODO close thread ?!?
        RHM.logger.info('Deleted radar {} channel {}.'.format(self.rnum, cnum))


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
               self.logger.debug("radar {} ch {}:_waitForState {}. state is {} (swing {})".format(self.rnum, self.cnum, state, self.state[swing], swing))
            time.sleep(RADAR_STATE_TIME)
            if time.time() - wait_start > CHANNEL_STATE_TIMEOUT:
                self.logger.error('CHANNEL STATE TIMEOUT for radar {} channel {}'.format(self.rnum, self.cnum))
                self.close()
                break


    def update_ctrlprm_class(self, period):
        if len(self.scanManager.scan_beam_list) == 0:
           self.logger.warning("scan_beam_list is empty. unable to update ctrlprm_class!")
           return

        if period == "current":
           beam = self.scanManager.current_beam
           freq = int(self.scanManager.get_current_clearFreq_result(self.rnum)[0])
        elif period == "next":
           beam = self.scanManager.next_beam
           freq = int(self.scanManager.get_next_clearFreq_result(self.rnum)[0])
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

        seq = sequence(self.npulses_per_sequence, self.tr_to_pulse_delay, self.parent_RadarHardwareManager.all_possible_integration_period_pulse_sample_offsets, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor, self.ctrlprm_struct.payload )

       # seq = sequence(self.npulses_per_sequence, self.tr_to_pulse_delay, self.pulse_sequence_offsets_vector, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor, self.ctrlprm_struct.payload )
        return seq


    def get_next_sequence(self):
        self.update_ctrlprm_class('next')
        seq = copy.deepcopy( sequence(self.npulses_per_sequence,self.tr_to_pulse_delay, self.parent_RadarHardwareManager.all_possible_integration_period_pulse_sample_offsets, self.pulse_lens, self.phase_masks, self.pulse_masks, self.channelScalingFactor, self.ctrlprm_struct.payload) )
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
        clrFreqResult = self.scanManager.get_current_clearFreq_result(self.rnum)

        transmit_dtype(self.conn, clrFreqResult[0], np.int32)
        transmit_dtype(self.conn, clrFreqResult[1], np.float32)

        self.logger.info('radar {} ch {}: clr frequency search raw data age: {:2.1f} s'.format(self.rnum, self.cnum, time.time() - clrFreqResult[2]))
        return RMSG_SUCCESS


    #@timeit
    def RequestClearFreqSearchHandler(self, rmsg):
        self.clrfreq_struct.receive(self.conn)
        if self.scanManager.fixFreq <= 0:
            # set request flat from RadarHardwareManager:clearFreqRawDatamanager
            self.parent_RadarHardwareManager.clearFreqRawDataManager.outstanding_request[self.rnum] = True
            self.logger.debug("RequestClearFreqSearchHandler: setting request CLR_FREQ flag in clearFreqRawDataManager (caused by radar {} ch {})".format(self.rnum,self.cnum))
        else:
            self.logger.debug("RequestClearFreqSearchHandler: ignoring because of fixfreq (radar {} ch {})".format(self.rnum, self.cnum))

        return RMSG_SUCCESS


    def UnsetReadyFlagHandler(self, rmsg):
        return RMSG_SUCCESS


    def SetReadyFlagHandler(self, rmsg):
        # ROS calls it ready, we call it trigger
        self.logger.debug("radar {} ch {}: SetReadyFlagHandler: waiting for nextSwingToTrigger (swing {}) to become CS_READY or CS_LAST_SWING".format(self.rnum, self.cnum, self.swingManager.nextSwingToTrigger))
        self._waitForState(self.swingManager.nextSwingToTrigger, [CS_READY, CS_LAST_SWING])
 #       transmit_dtype(self.conn, self.nSequences_per_period, np.uint32) # TODO mgu transmit here nSeq ?
        self.logger.debug("radar {} ch {}: SetReadyFlagHandler: setting nextSwingToTrigger state (swing {}) to CS_TRIGGER".format(self.rnum, self.cnum, self.swingManager.nextSwingToTrigger))
        self.state[self.swingManager.nextSwingToTrigger] = CS_TRIGGER
        self.triggered_swing_list.insert(0, self.swingManager.nextSwingToTrigger)
        # send trigger command
        return RMSG_SUCCESS


    #@timeit
    def RegisterSeqHandler(self, rmsg):

        # function to get the indexes of rising edges going from zero to a nonzero value in array ar
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

        self.logger.debug('Entering RegisterSeqHandler for channel {}'.format(self.cnum))

        # site libraries appear to not initialize the status, so a nonzero status here is normal.

        self.seqprm_struct.receive(self.conn)
        self.seq_rep  = recv_dtype(self.conn, np.uint8, self.seqprm_struct.payload['len'])
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

        self.logger.debug("pulse0 length: {} us, tr_pulse_delay: {} us, tx_time: {} us".format(self.pulse_lens[0], tr_to_pulse_delay, self.pulse_lens[0] + 2 * self.tr_to_pulse_delay))
        if npulses_per_sequence == 0:
            raise ValueError('number of pulses per sequence must be greater than zero!')
        if nbb_samples == 0:
            raise ValueError('number of samples in sequence must be nonzero!')

        return RMSG_SUCCESS


    def get_if_data(channel):
      RHM = channel.parent_RadarHardwareManager
      # CUDA_GET_IF_DATA
      channel.logger.debug('start CUDA_GET_IF_DATA')
      jrad=channel.rnum
      cmd = cuda_get_if_data_command(RHM.cudasocks[channel.nrad], RHM.swingManager.processingSwing)
      cmd.transmit()
      time.sleep(0.001)

      if_samples = None
      all_antenna_list = RHM.antenna_idx_list_main[jrad] + RHM.antenna_idx_list_back[jrad]
      for cudasock in RHM.cudasocks[channel.rnum]:
          nAntennas = recv_dtype(cudasock, np.uint32)

          if channel.processing_state == CS_PROCESSING:
              transmit_dtype(cudasock, channel.cnum, np.int32)

              for iAntenna in range(nAntennas):
                  antIdx = recv_dtype(cudasock, np.uint16)
                  nSamples_if = int(recv_dtype(cudasock, np.uint32) )
                  channel.logger.debug("Receiving {} if samples.".format(nSamples_if))
                  if if_samples is None:
                     if_samples = np.zeros((nAntennas, nSamples_if), dtype=np.float32)

                  samples = recv_dtype(cudasock, np.float32, nSamples_if )
#                  samples = samples[0::2] + 1j * samples[1::2] # TODO change to match export format. i/q int32 ????

                  # TODO add back array
                  iAntenna = all_antenna_list.index(antIdx)
                  if_samples[iAntenna] = samples[:]

          transmit_dtype(cudasock, -1, np.int32) # to end transfer process

          cmd.client_return()
          channel.raw_export_data['data'] = if_samples * RHM.scaling_factor_rx_if[channel.rnum]
          channel.raw_export_data['nAntennas'] = nAntennas
          channel.raw_export_data['nSamples'] = nSamples_if
          channel.logger.debug('end CUDA_GET_IF_DATA')


    def write_bb_data(channel):
        channel.logger.debug('start saving BB samples')
        time_now = datetime.datetime.now(datetime.UTC)
        version = 3
        hardwareManager = channel.parent_RadarHardwareManager

        savePath = "/data/image_samples/bb_data"
        if not os.path.isdir(savePath):
            os.mkdir(savePath)

        fileName = '{:04d}{:02d}{:02d}.{:02d}{:02d}.{}.{:c}.iraw'.format(time_now.year, time_now.month, time_now.day, time_now.hour, time_now.minute, channel.ststr, 96+channel.cnum)

        exportList = []
        exportList = []
        exportList.append( version )
        exportList.append( channel.stid )
        exportList.append( channel.cnum )
        exportList.append( time_now.year )
        exportList.append( time_now.month )
        exportList.append( time_now.day )
        exportList.append( time_now.hour )
        exportList.append( time_now.minute )
        exportList.append( time_now.second )
        exportList.append( time_now.microsecond )
        exportList.append( channel.raw_export_data['nrang'])
        exportList.append( channel.raw_export_data['mpinc'])
        exportList.append( channel.raw_export_data['smsep'])
        exportList.append( channel.raw_export_data['lagfr'])
        exportList.append( hardwareManager.commonChannelParameter['pulseLength'])  # in micro sec
        exportList.append( channel.raw_export_data['beam'])
        exportList.append( channel.raw_export_data['rfreq']) # in kHz
        exportList.append( channel.raw_export_data['mppul'])
        exportList += channel.raw_export_data['ppat']
        exportList.append( channel.raw_export_data['nbaud'])
        exportList += channel.raw_export_data['pcode']
#       exportList.append(channel.bb_export['nSamples'])
        exportList.append(channel.bb_export['number_of_samples'])
        exportList.append(channel.bb_export['nSequences_per_period'])
        exportList.append(channel.bb_export['nMainAntennas'] + channel.bb_export['nBackAntennas'])
        exportList += channel.bb_export["antenna_list"]

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
        exportList.append( time_now.microsecond )
        exportList.append( channel.raw_export_data['nrang'])
        exportList.append( channel.raw_export_data['mpinc'])
        exportList.append( channel.raw_export_data['smsep'])
        exportList.append( channel.raw_export_data['lagfr'])
        exportList.append( hardwareManager.commonChannelParameter['pulseLength'])  # in micro sec
        exportList.append( channel.raw_export_data['beam'])
        exportList.append( channel.raw_export_data['rfreq']) # in kHz
        exportList.append( channel.raw_export_data['mppul'])
        exportList += channel.raw_export_data['ppat']
        exportList.append( channel.raw_export_data['nbaud'])
        exportList += channel.raw_export_data['pcode']
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
        # TODO divide compatibility check in sequence and ctrlprm check?
        # TODO add compatibility check in parameter prediction function
        self.received_first_SETPAR = True
        RHM = self.parent_RadarHardwareManager

        self.logger.debug("radar {} ch {}: waiting for Parameter semaphore...".format(self.rnum, self.cnum))
        RHM.set_par_semaphore.acquire()
        self.logger.debug("radar {} ch {}: acquired semaphore, setting parameter".format(self.rnum, self.cnum))

        RHM.n_SetParameterHandlers_active += 1

        if self.scanManager.isPrePeriod:
            current_swing = self.swingManager.nextSwingToTrigger
        else:
            # swing with this parameters has already been triggered, just compare parameter
            current_swing = 1 - self.swingManager.nextSwingToTrigger

        if self.scanManager.isInitSetParameter:
           self.scanManager.isInitSetParameter = False
           self.ctrlprm_struct.receive(self.conn)

           self.logger.debug("radar {} ch {}: Received from ROS for swing {} (init SetPar is only stored): tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.rnum, self.cnum, current_swing, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))

           RHM.n_SetParameterHandlers_active -= 1
           RHM.set_par_semaphore.release()
           self.logger.debug("radar {} ch {}: released set_par_semaphore".format(self.rnum, self.cnum))
           return RMSG_SUCCESS

#        self.logger.debug("radar {} ch {}: Received from ROS SetParameter for swing {} : tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.rnum, self.cnum, current_swing, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))
        self.logger.debug("swing state {}".format(self.state[current_swing]))
        # wait if RHM.trigger_next_swing() is slower...
        self._waitForState(current_swing, [CS_INACTIVE, CS_PROCESSING, CS_LAST_SWING])

        # period not yet triggered
        if self.state[current_swing] == CS_INACTIVE:# or self.active_state == CS_READY: # not needed with change of site.c

           if self.state[current_swing] == CS_READY:
              self.logger.debug("Channel already initialized, but not triggered, Reinitializing it...")
              self.state[current_swing] = CS_INACTIVE

           self.ctrlprm_struct.receive(self.conn)
           self.logger.debug("radar {} ch {}: Received from ROS: tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.rnum, self.cnum, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))

           if not self.CheckChannelCompatibility(): # TODO for two swings and reset after transmit?
              self.logger.debug("CheckChannelCompatability FAIL")
              RHM.n_SetParameterHandlers_active -= 1
              RHM.set_par_semaphore.release()
              self.logger.debug("radar {} ch {}: released set_par_semaphore".format(self.rnum, self.cnum))
              return RMSG_FAILURE

           if self not in self.parent_RadarHardwareManager.newChannelList:
              self.parent_RadarHardwareManager.newChannelList.append(self)
              self.logger.debug("Adding radar {} ch {} to newChannelList ({})".format(self.rnum, self.cnum, self.parent_RadarHardwareManager.newChannelList))
           else:
               self.logger.debug("radar {} ch {}: already in newChannelList ({})".format(self.rnum, self.cnum, self.parent__RadarHardwareManager.newChannelList))

        # in middle of scan, period already triggerd. only compare with prediction
        elif self.state[current_swing] == CS_PROCESSING or self.state[current_swing] == CS_LAST_SWING:

           self.update_ctrlprm_class("current")
           ctrlprm_old = copy.deepcopy(self.ctrlprm_struct.payload)

           # compare received with predicted parameter
           self.ctrlprm_struct.receive(self.conn)
           self.logger.debug("radar {} ch {}: Received from ROS SetParameter for swing {} : tbeam={}, rbeam={}, tfreq={}, rfreq={}".format(self.rnum, self.cnum, current_swing, self.ctrlprm_struct.payload['tbeam'], self.ctrlprm_struct.payload['rbeam'], self.ctrlprm_struct.payload['tfreq'], self.ctrlprm_struct.payload['rfreq']))
           for key in ctrlprm_old.keys():
              if np.any(ctrlprm_old[key] != self.ctrlprm_struct.payload[key]):
                  # control program always sends 2 SET_PAR. 1st one with freq 12MHz
                  if (key == "tfreq" or key == "rfreq") and self.ctrlprm_struct.payload[key] == 12000:
                      continue
                  self.logger.debug("radar {} ch {}: received new ctrl_prm {} ({}) old ctrl_prm ({})".format(self.rnum, self.cnum, key, self.ctrlprm_struct.payload[key], ctrlprm_old[key]))
                  self.logger.error("radar {} ch {}: received ctrlprm_struct for {} ({}) is not equal with prediction ({})".format(self.rnum, self.cnum, key,self.ctrlprm_struct.payload[key], ctrlprm_old[key]))

           # RHM.set_par_semaphore.release()
           # self.logger.debug("radar {} Ch {} released semaphore".format(self.rnum, self.cnum))

        else:
           self.logger.error("ROS:SetParameter: Active state is {} (current_swing={}, activeSwing={} ). Dont know what to do...".format(self.state[current_swing], self.swingManager.activeSwing, self.active_state))
           self.logger.error("ROS:SetParameter: Exit usrp_server...")
           RHM.n_SetParameterHandlers_active -= 1

           RHM.set_par_semaphore.release()
           self.logger.debug("radar {} ch {}: released set_par_semaphore".format(self.rnum, self.cnum))
           return RMSG_FAILURE
           self.parent_RadarHardwareManager.exit() #How does this ever get executed? (WB)

        if (self.rnum < 0 or self.cnum < 0):
           self.logger.error("SET_PARAMETER: Invalid radar or channel number: rnum={}, cnum={}".format(self.rnum, self.cnum))
           RHM.n_SetParameterHandlers_active -= 1

           RHM.set_par_semaphore.release()
           self.logger.debug("radar {} ch {}: released set_par_semaphore".format(self.rnum, self.cnum))
           return RMSG_FAILURE

        RHM.n_SetParameterHandlers_active -= 1
        self.logger.debug("radar {} ch {}: Done SetParametersHandler".format(self.rnum,self.cnum))

        RHM.set_par_semaphore.release()
        self.logger.debug("radar {} ch {}: released set_par_semaphore".format(self.rnum, self.cnum))
        return RMSG_SUCCESS


    def CheckChannelCompatibility(self):
        self.logger.debug('checking channel compatibility for radar {} channel {}'.format(self.rnum, self.cnum))
        hardwareManager = self.parent_RadarHardwareManager
        commonParList_ctrl = ['number_of_samples', 'baseband_samplerate']
        commonParList_seq  = ['npulses_per_sequence', 'pulse_sequence_offsets_vector', 'tr_to_pulse_delay', 'integration_period_duration', 'tx_time']

        if all([self.pulse_lens[0]==self.pulse_lens[i] for i in range(1,len(self.pulse_lens))]):
            pulseLength = self.pulse_lens[0]
        else:
            self.logger.error("Pulse lengths in one sequence have to be the equal!") # TODO raise error?
            pdb.set_trace()
            return False

        if hardwareManager.nRegisteredChannels == 1 and (len(np.concatenate(hardwareManager.channels).tolist()) == 0 or np.concatenate(hardwareManager.channels).tolist()[0] == self):
           self.logger.info("Compatibility check: This channel is already registered at HardwareManager and is the only one. Renewing registration.")
           # hardwareManager.nRegisteredChannels = 0

        if (hardwareManager.nRegisteredChannels <= 0) or (len(hardwareManager.commonChannelParameter) == 0):  # this is the first channel
            hardwareManager.commonChannelParameter = {key: getattr(self, key) for key in commonParList_seq}
            hardwareManager.commonChannelParameter.update({key: self.ctrlprm_struct.payload[key] for key in commonParList_ctrl})
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
                                   [15, 20],  # 5M => 16.67k
                                   [10, 20],  # 5M => 25k
                                   [10, 75],  # 2.5M => 3.333k
                                   [30, 100], # 10M => 3.333k
                                   [20, 50],  # 10M => 10k
                                   [20, 30],  # 10M => 16.67k
                                   [20, 20],  # 10M => 25k
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

            try:
                parCompatibleList_seq = [hardwareManager.commonChannelParameter[parameter] == getattr(self, parameter) for parameter in commonParList_seq]
            except KeyError:
                self.logger.error('Error comparing sequence parameters: radar {} ch {}'.format(self.rnum, self.cnum))
                return False

            parCompatibleList_ctrl = [hardwareManager.commonChannelParameter[parameter] == self.ctrlprm_struct.payload[parameter] for parameter in commonParList_ctrl]

            idxOffsetVec = commonParList_seq.index('pulse_sequence_offsets_vector')  # convert vector of bool to scalar
            parCompatibleList_seq[idxOffsetVec] = parCompatibleList_seq[idxOffsetVec].all()

         #   pdb.set_trace()
            if (not all(parCompatibleList_seq)) or (not all(parCompatibleList_ctrl)) or (pulseLength != hardwareManager.commonChannelParameter['pulseLength']):
                self.logger.error('Unable to add new channel. Parameters not compatible with active channels.')
                for iPar,isCompatible in enumerate(parCompatibleList_seq):
                     if not all(isCompatible):
                        self.logger.error(" Not compatible sequence parameter: {}   old channel(s): {}, new channel: {}".format(commonParameterList_seq[iPar], hardwareManager.commonChannelParameter[commonParameterList_seq[iPar]], getattr(self, commonParameterList_seq[iPar])))
                for iPar,isCompatible in enumerate(parCompatibleList_ctrl):
                     if not isCompatible:
                        self.logger.error(" Not compatible ctrlprm: {}   old channel(s): {}, new channel: {}".format(commonParameterList_ctrl[iPar], hardwareManager.commonChannelParameter[commonParameterList_ctrl[iPar]], self.ctrlprm_struct.payload[commonParameterList_ctrl[iPar]]))
                if pulseLength != hardwareManager.commonChannelParameter['pulseLength']:
                    self.logger.error(" Pulse length of new channel ({}) is not compatible to old channel(s) ({})".format(pulseLength, hardwareManager.commonChannelParameter['pulseLength']))
                return False
            else:
                if self not in np.concatenate(hardwareManager.channels).tolist():
                   hardwareManager.nRegisteredChannels += 1
                return True

        # TODO change usrp_xx_cfreq somewhere if possible
        assert np.abs((ch.ctrlprm_struct.payload['tfreq'] * 1e3) - self.mixingFreqManager.current_mixing_freq[self.rnum]*1e6) < (self.usrp_rf_tx_rate / 2), 'transmit frequency outside range supported by sampling rate and center frequency'


    # send ctrlprm struct
    #@timeit
    def GetParametersHandler(self, rmsg):
        # TODO: return bad status if negative radar or channel
#        if self.received_first_SETPAR:
        self.update_ctrlprm_class("current")
        self.ctrlprm_struct.transmit()
        self.logger.debug("radar {} ch {}: sending current ctrlprm_struct (tfreq={}, rfreq={},tbeam={},rbeam={})".format(self.rnum, self.cnum, self.ctrlprm_struct.get_data('tfreq'), self.ctrlprm_struct.get_data('rfreq'), self.ctrlprm_struct.get_data('tbeam'), self.ctrlprm_struct.get_data('rbeam')))
        return RMSG_SUCCESS


    #@timeit
    def GetDataHandler(self, rmsg):
        self.logger.debug('start channelHandler:GetDataHandler radar {} ch: {}'.format(self.rnum, self.cnum))
        self.update_ctrlprm_class("current")
        self.dataprm_struct.set_data('samples', self.ctrlprm_struct.payload['number_of_samples'])

        self.dataprm_struct.transmit() # only 'samples' of dataprm is ever changed TODO check other parameter such as event_secs....
        self.logger.debug('radar {} ch {}: sending dprm struct'.format(self.rnum, self.cnum))

        if self.rnum < 0 or self.cnum < 0:
            pdb.set_trace()
            return RMSG_FAILURE

        # TODO investigate possible race conditions

        finishedSwing = self.triggered_swing_list.pop()
        self.logger.debug('radar {} ch {}: channelHandler:GetDataHandler waiting for channel to idle before GET_DATA (finished swing is {})'.format(self.rnum, self.cnum, finishedSwing))
        self.logger.debug("start waiting for CS_SAMPLES_READY")
        self._waitForState(finishedSwing, CS_SAMPLES_READY)
        self.logger.debug("end waiting for CS_SAMPLES_READY")

        self.logger.debug('radar {} ch {}: channelHandler:GetDataHandler returning samples'.format(self.rnum, self.cnum))
#        transmit_dtype(self.conn, self.parent_RadarHardwareManager.resultData_nSequences_per_period, np.uint32)
        self.send_results_to_control_program()

        self.logger.debug('radar {} ch {}: channelHandler:GetDataHandler finished returning samples. setting state to {} (swing {})'.format(self.rnum, self.cnum, self.next_state[finishedSwing], finishedSwing))
        self.state[finishedSwing] = self.next_state[finishedSwing]
        self.logger.debug('end channelHandler:GetDataHandler radar {} ch: {}'.format(self.rnum, self.cnum))

        return RMSG_SUCCESS


    def send_results_to_control_program(self):
        # interact with site library's SiteIntegrate loop
        # send metadata for integration period
        # currently assuming pulse sequences are uniform within an integration period

        for listno,resultDict in enumerate(self.resultDict_list):
           self.logger.debug("radar {} channel {} list number {}  number of sequences in period: {}".format(self.rnum, self.cnum, listno,resultDict['nSequences_per_period']))

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
            self.logger.warning('Following USRPs report Fault: {} (usrp index)'.format([k for k in range(txstatus_agc.size) if txstatus_agc[k] != 0]))

        transmit_dtype(self.conn, num_transmitters, np.int32)
        transmit_dtype(self.conn, txstatus_agc,     np.int32) # length num_transmitters
        transmit_dtype(self.conn, txstatus_lowpwr,  np.int32) # length num_transmitters

        # transmit trigger time
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
            # self.logger.debug('GET_DATA sending samples for seq {} time: {}'.format(iSequence, resultDict['sequence_start_time_secs'][iSequence]+resultDict['sequence_start_time_usecs'][iSequence]/1.e6))

            #self.logger.debug('GET_DATA sending sequence start time')
            transmit_dtype(self.conn, resultDict['sequence_start_time_secs'][iSequence],  np.uint32)
            transmit_dtype(self.conn, resultDict['sequence_start_time_usecs'][iSequence], np.uint32)

            # calculate the baseband sample index for the start and end of a pulse sequence
            # within a block of beamformed samples over the entire integration period
            # assuming that the first sample is aligned with the center of the first transmit pulse
            # and all sequences within the integration period have the same length

            pulse_sequence_start_index = iSequence * resultDict['nbb_rx_samples_per_sequence']
            pulse_sequence_end_index = pulse_sequence_start_index + resultDict['number_of_samples']
            # self.logger.debug("Number of samples if {} (no deepcopy version is {}), main beamformed shape: {}".format(resultDict['number_of_samples'], rd_shallow['number_of_samples'], len(resultDict['main_beamformed'])/resultDict['number_of_samples']))
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

        self.logger.debug('GET_DATA: finished returning samples')


    #@timeit
    def SetRadarChanHandler(self, rmsg):
        self.stid = recv_dtype(self.conn, np.int32)
        data_length = recv_dtype(self.conn, np.int32)
        temp = recv_dtype(self.conn, str, nitems=data_length)
        self.ststr = temp.decode("utf-8")[-4:-1]
        self.rnum = recv_dtype(self.conn, np.int32)
        self.cnum = recv_dtype(self.conn, np.int32)

        if [self.rnum,self.cnum] in [[[ch.rnum,ch.cnum]] for ch in np.concatenate(self.parent_RadarHardwareManager.channels).tolist() if ch is not None and ch is not self]:
           self.logger.error("New channel (cnum {}) can not be added on radar {} beause channel with this cnum already active.".format(self.cnum,self.rnum))
           return RMSG_FAILURE

        self.ctrlprm_struct.set_data('channel', self.cnum)
        self.ctrlprm_struct.set_data('radar', self.rnum)

        self.scanManager = scanManager(self)

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
        # send information about tx_scaling_factor_total as txpow
        tx_factor = self.parent_RadarHardwareManager.scaling_factor_tx_total[self.rnum]*100
        transmit_dtype(self.conn, tx_factor, np.int32)

        # send information about amplification and attenuation settings as atten
        amp1 = self.parent_RadarHardwareManager.ini_rxfe_settings[self.rnum].getboolean('enable_amp1')
        amp2 = self.parent_RadarHardwareManager.ini_rxfe_settings[self.rnum].getboolean('enable_amp2')
        atten = float(self.parent_RadarHardwareManager.ini_rxfe_settings[self.rnum]['attenuation'])
        if amp1 == True:
          atten -= 15.0
        if amp2 == True:
          atten -= 15.0
        transmit_dtype(self.conn, atten, np.int32)
        return RMSG_SUCCESS


    def SetActiveHandler(self, rmsg):
        # called by site library at the start of a scan
        self.active = True
        if self not in self.parent_RadarHardwareManager.active_channels[self.rnum]:
           self.parent_RadarHardwareManager.active_channels[self.rnum].append(self)
           self.logger.debug("Added radar {} ch {} to RHM.active_channels list".format(self.rnum,self.cnum))

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

        if scan_num_beams == 1: # make sure these variables are lists even for one beam per scan
           clrfreq_start_list = [clrfreq_start_list]
           clrfreq_bandwidth_list = [clrfreq_bandwidth_list]
           scan_beam_list = [scan_beam_list]

        self.logger.debug('SetActiveHandler number of beams per scan: {}'.format(scan_num_beams))
        self.logger.debug('SetActiveHandler fixFreq: {}'.format(fixFreq))
        self.logger.debug('SetActiveHandler clear frequency search start frequencies: [{}]'.format(' '.join(map(str,clrfreq_start_list))))
        self.logger.debug('SetActiveHandler clear frequency search bandwidths (kHz): [{}]'.format(' '.join(map(str,clrfreq_bandwidth_list))))
        self.logger.debug('SetActiveHandler scan beam list: {}'.format(scan_beam_list))
        self.logger.debug('SetActiveHandler scan_duration: {}'.format(scan_time))
        self.logger.debug('SetActiveHandler integration_duration: {}'.format(integration_time))

        if syncBeams == 1:
           scan_times_list = recv_dtype(self.conn, np.int32, nitems = scan_num_beams) / 1000
           self.logger.debug('SetActiveHandler scan_times_list: {}'.format(scan_times_list))
        else:
           scan_times_list = None
           self.logger.debug('SetActiveHandler: no time sync of beams')

        if self.scanManager.camping:
            # if camping==True control progam uses SetActive for every new beam but no paramater should change
            # in this case skip reset of scanManager and swingManager to be able to use two swings parallel
            assert(fixFreq == self.scanManager.fixFreq)
            assert(scan_beam_list == self.scanManager.scan_beam_list)
            self.logger.info("SetActiveHandler: Skipping reset of scanManager and swingManager because nothing changed with camping beam.")
            radar_active[self.rnum] = True
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

        radar_active[self.rnum] = True

        addFreqResult = self.parent_RadarHardwareManager.mixingFreqManager.add_new_freq_band(self)

        if addFreqResult == True:
            #self.swingManager.reset()
            #self.logger.debug("Resetting swing manager (active={}, processing={})".format(self.swingManager.activeSwing, self.swingManager.processingSwing))
            return RMSG_SUCCESS
        elif addFreqResult == False:
            self.logger.error("Freq range of new channel (rnum {} cnum {}) is not in USRP bandwidth. (freq_range_list[0][0] = {})".format(self.rnum, self.cnum, freq_range_list[0][0]))
            self.scanManager.clear_freq_range_list = None
            self.scan_beam_list = None
            self.fixFreq = None
            return RMSG_FAILURE
        else: # new mixing freq
            self.parent_RadarHardwareManager.send_cuda_setup_command()
            self.parent_RadarHardwareManager.clearFreqRawDataManager.center_freq[self.rnum] = self.parent_RadarHardwareManager.mixingFreqManager.current_mixing_freq[self.rnum]
            self.parent_RadarHardwareManager.clearFreqRawDataManager.metaData[self.rnum]['usrp_fcenter'] = self.parent_RadarHardwareManager.mixingFreqManager.current_mixing_freq[self.rnum]
            #self.swingManager.reset()
            #self.logger.debug("Resetting swing manager (active={}, processing={})".format(self.swingManager.activeSwing, self.swingManager.processingSwing))
            return RMSG_SUCCESS


    def SetInactiveHandler(channelObject, rmsg):
        RHM = channelObject.parent_RadarHardwareManager
        RHM.logger.info('ROS:SET_INACTIVE received for radar {} channel {}'.format(channelObject.rnum,channelObject.cnum))
        RHM.logger.info('radar {} ch {}: RHM active channels objects: {}'.format(channelObject.rnum,channelObject.cnum,RHM.active_channels[channelObject.rnum]))

        if channelObject in RHM.active_channels[channelObject.rnum]:
            RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE trying to remove channel {} from RHM.active_channels'.format(channelObject.rnum,channelObject.cnum,RHM.channels[channelObject.rnum].index(channelObject)))
            try:
               RHM.active_channels[channelObject.rnum].remove(channelObject)
               RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE removing from RHM.active_channels success'.format(channelObject.rnum,channelObject.cnum))
            except:
               RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE failed to remove from RHM.active_channels'.format(channelObject.rnum,channelObject.cnum))
        else:
            RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE no channels to remove from RHM.active_channels'.format(channelObject.rnum,channelObject.cnum))

        RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE trying to remove channels from HardwareManager'.format(channelObject.rnum,channelObject.cnum))
        if channelObject in np.concatenate(RHM.channels).tolist():
            RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE removing channel {} from HardwareManager'.format(channelObject.rnum,channelObject.cnum,RHM.channels[channelObject.rnum].index(channelObject)))
            try:
               RHM.channels[channelObject.rnum].remove(channelObject)
               RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE removing from HardwareManager success'.format(channelObject.rnum,channelObject.cnum))
            except:
               RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE failed to remove from HardwareManager'.format(channelObject.rnum,channelObject.cnum))

            RHM.nRegisteredChannels -= 1
            if RHM.nRegisteredChannels <= 0:
                RHM.logger.debug("radar {} ch {}: No channels left, removing commonChannelParameter".format(channelObject.rnum,channelObject.cnum))
                RHM.commonChannelParameter = {}
                RHM.nRegisteredChannels = 0
                radar_active[:] = False
        else:
            RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE no channels to remove from HardwareManager'.format(channelObject.rnum,channelObject.cnum))

        channelObject.active = False
        RHM.logger.debug('radar {} ch {}: ROS:SET_INACTIVE sending RMSG_SUCCESS'.format(channelObject.rnum,channelObject.cnum))
        # TODO: return failure status if the radar or channel number is invalid?
        return RMSG_SUCCESS


def main():
    # maybe switch to multiprocessing with manager process

    now = datetime.datetime.now(datetime.UTC)
    now_string = now.strftime("__%Y%m%d_%H%M%S")
    logging_usrp.initLogging('server' + now_string + '.log')
    logging.info('Starting main() of usrp_server')

    radar = RadarHardwareManager(RMSG_PORT)
    radar.run()


if __name__ == '__main__':
    main()
