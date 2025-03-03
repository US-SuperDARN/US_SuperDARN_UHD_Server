# library containing constants and functions for passing messages over sockets between c/python and python/python
import numpy as np
import uuid
import collections
import pdb
import logging
import time
from socket_utils import *
from termcolor import cprint

checkSwing  = True # check if transmitted swing is declared


UHD_SETUP = ord('s')
UHD_RXFE_SET = ord('r')
UHD_READY_DATA = ord('d')
UHD_TRIGGER_PULSE = ord('t')
UHD_CLRFREQ = ord('c')
UHD_AUTOCLRFREQ = ord('a')
UHD_GETTIME = ord('m')
UHD_SYNC = ord('S')
UHD_EXIT = ord('e')

CUDA_PROCESS = ord('p')
CUDA_GET_DATA = ord('g')
CUDA_GET_IF_DATA = ord('o')
CUDA_EXIT = ord('e')
CUDA_ADD_CHANNEL = ord('q')
CUDA_REMOVE_CHANNEL = ord('r')
CUDA_GENERATE_PULSE = ord('l')
CUDA_SETUP = ord('s')

CONNECTION_ERROR = "CONNECTION_ERROR"

# NOT USED:
CUDA_PULSE_INIT = ord('i')
NO_COMMAND = ord('n')
UHD_SETUP_READY = ord('y')
UHD_TRIGGER_BUSY = ord('b')
UHD_TRIGGER_PROCESS = ord('p')
USRP_DRIVER_ERROR = -1
TRIGGER_BUSY = 'b'




# parent class for driver socket messages
class driver_command(object):
    # class to help manage sending data 
    class socket_data(object):
        def __init__(self, data, dtype, name, nitems = 1):
            self.name = name
            self.nitems = nitems
            
            self.dtype = dtype
            self.data = dtype(data)
        
        def transmit(self, sock):
     #       print('\t driverMSG transmitting: {}, value: {}, type {}'.format(self.name, self.data, self.dtype))
            return transmit_dtype(sock, self.data, self.dtype)

        def receive(self, sock):
            self.data = recv_dtype(sock, self.dtype, self.nitems)
            #print('\treceiving: {}, value: {}, type {}'.format(self.name, self.data, self.dtype))
            return self.data

    def __init__(self, clients, command):
        if not hasattr(clients, '__iter__'):
            clients = [clients]

        self.clients = clients
        self.dataqueue = [] # ordered list of variables to transmit/receive
        self.payload = {} # dictionary to store received values
        self.command = np.uint8(command)
        self.logger = logging.getLogger('socket_data')
    
    def queue(self, data, dtype, name = '', nitems = 1):
        # pdb.set_trace()
        self.dataqueue.append(self.socket_data(data, dtype, name, nitems = nitems))
   
    
    # command to set variables by name in the dataqueue
    def set_data(self, name, data):
        for var in self.dataqueue:
            if var.name == name:
                var.data = data
                break

    def get_data(self, name):
        for var in self.dataqueue:
            if var.name == name:
                return var.data
        return None

    def transmit(self):
        for clientsock in self.clients:
            try:
               if self.command != NO_COMMAND:
                   transmit_dtype(clientsock, np.uint8(self.command))

               for item in self.dataqueue:
            #       pdb.set_trace()
                   if checkSwing and  item.name == "swing" and item.data == np.uint32(-1):
                      raise ValueError("swing has been not defined!")
                   item.transmit(clientsock)

                  # cprint('transmitting {}: {}'.format(item.name, item.data), 'yellow')
            except:
#               self.logger.error("Error transmitting command {} to cient {}:{} (from {};{})".format(self.command, clientsock.getpeername()[0], clientsock.getpeername()[1], clientsock.getsockname()[0], clientsock.getsockname()[1] ))
#               self.logger.error("Error transmitting command {}  (from {};{})".format(self.command,  clientsock.getsockname()[0], clientsock.getsockname()[1] ))
               self.logger.error("Error transmitting command {} ".format(self.command ))

    # ask all clients for a return value, compare against command
    # normally, client will indicate success by sending the command byte back to the server 
    def client_return(self, dtype = np.uint8, check_return = True):
        returns = []
        for client in self.clients:
            try:
               r = recv_dtype(client, dtype)
               if check_return:
                   assert r == self.command, 'return {} does not match expected command of {}'.format(r, self.command)
               
               returns.append(r)
            except:
               self.logger.error("Error receiving client_return for command '{}'({}) from  client {}:{}".format(chr(self.command), self.command, client.getsockname()[0], client.getsockname()[1] ))
              # pdb.set_trace()
               returns.append(CONNECTION_ERROR)

        #cprint('command return success', 'yellow')
        return returns

    def receive(self, sock):
        for item in self.dataqueue:
            data = item.receive(sock)
            self.payload[item.name] = data
             

class server_ctrlprm(driver_command):
    def __init__(self, servers = None, ctrlprm_dict = None):
        if not (servers == None) and not isinstance(servers, collections.Iterable):
                servers = [servers]

        driver_command.__init__(self, servers, NO_COMMAND)
        if ctrlprm_dict == None:
            # stuff in default values to be over-written during rx
            ctrlprm_dict = {}
            ctrlprm_dict['radar'] = 0
            ctrlprm_dict['channel'] = 0
            ctrlprm_dict['local'] = 0
            ctrlprm_dict['priority'] = 0
            ctrlprm_dict['current_pulseseq_idx'] = 0
            ctrlprm_dict['tbeam'] = 0
            ctrlprm_dict['tbeamcode'] = 0
            ctrlprm_dict['tbeamazm'] = 0
            ctrlprm_dict['tbeamwidth'] = 0
            ctrlprm_dict['tfreq'] = 0
            ctrlprm_dict['trise'] = 0
            ctrlprm_dict['number_of_samples'] = 0
            ctrlprm_dict['buffer_index'] = 0
            ctrlprm_dict['baseband_samplerate'] = 0
            ctrlprm_dict['filter_bandwidth'] = 0
            ctrlprm_dict['match_filter'] = 0
            ctrlprm_dict['rfreq'] = 0
            ctrlprm_dict['rbeam'] = 0
            ctrlprm_dict['rbeamcode'] = 0
            ctrlprm_dict['rbeamazm'] = 0
            ctrlprm_dict['rbeamwidth'] = 0
            ctrlprm_dict['status'] = 0

        self.queue(ctrlprm_dict['radar'], np.int32, 'radar')
        self.queue(ctrlprm_dict['channel'], np.int32, 'channel')
        self.queue(ctrlprm_dict['local'], np.int32, 'local')
        self.queue(ctrlprm_dict['priority'], np.int32, 'priority')
        self.queue(ctrlprm_dict['current_pulseseq_idx'], np.int32, 'pulseseq_idx')
        self.queue(ctrlprm_dict['tbeam'], np.int32, 'tbeam')
        self.queue(ctrlprm_dict['tbeamcode'], np.uint32, 'tbeamcode')

        self.queue(ctrlprm_dict['tbeamazm'], np.float32, 'teambeamazm')
        self.queue(ctrlprm_dict['tbeamwidth'], np.float32, 'tbeamwidth')

        self.queue(ctrlprm_dict['tfreq'], np.int32, 'tfreq')
        self.queue(ctrlprm_dict['trise'], np.int32, 'trise')

        self.queue(ctrlprm_dict['number_of_samples'], np.int32, 'number_of_samples')
        self.queue(ctrlprm_dict['buffer_index'], np.int32, 'buffer_index')

        self.queue(ctrlprm_dict['baseband_samplerate'], np.float32, 'baseband_samplerate')
        self.queue(ctrlprm_dict['filter_bandwidth'], np.int32, 'filter_bandwidth')

        self.queue(ctrlprm_dict['match_filter'], np.int32, 'match_filter')
        self.queue(ctrlprm_dict['rfreq'], np.int32, 'rfreq')
        self.queue(ctrlprm_dict['rbeam'], np.int32, 'rbeam')
        self.queue(ctrlprm_dict['rbeamcode'], np.uint32, 'rbeamcode')
        self.queue(ctrlprm_dict['rbeamazm'], np.float32, 'rbeamazm')
        self.queue(ctrlprm_dict['rbeamwidth'], np.float32, 'rbeamwidth')

        self.queue(ctrlprm_dict['status'], np.int32, 'status')

        # TODO: set these properly 
        name_arr = np.uint8(np.zeros(80))
        self.queue(name_arr, np.uint8, name='name_arr', nitems=80)

        description_arr = np.uint8(np.zeros(120))
        self.queue(description_arr, np.uint8, name='desc_arr', nitems=120)

class cuda_get_if_data_command(driver_command):
    def __init__(self, cudas, swing = -1):
        driver_command.__init__(self, cudas, CUDA_GET_IF_DATA)
        self.queue(swing, np.uint32, 'swing')
    
class cuda_get_data_command(driver_command):
    def __init__(self, cudas, swing = -1):
        driver_command.__init__(self, cudas, CUDA_GET_DATA)
        self.queue(swing, np.uint32, 'swing')
    
class cuda_exit_command(driver_command):
    def __init__(self, cudas):
        driver_command.__init__(self, cudas, CUDA_EXIT)

class cuda_pulse_init_command(driver_command):
    def __init__(self, cudas, swing = -1):
        driver_command.__init__(self, cudas, CUDA_PULSE_INIT)
        self.queue(swing, np.uint32, 'swing')

class cuda_process_command(driver_command):
    def __init__(self, cudas, swing = -1, nSamples = -1):
        driver_command.__init__(self, cudas, CUDA_PROCESS)
        self.queue(swing, np.uint32, 'swing')
        self.queue(nSamples, np.uint64, 'nSamples')

class cuda_setup_command(driver_command):
    def __init__(self, cudas, upsampleRate = 0, downsampleRate_rf2if=0, downsampleRate_if2bb=0, usrp_mixing_freq=0):
        driver_command.__init__(self, cudas, CUDA_SETUP)
        self.queue(upsampleRate, np.uint32, 'upsampleRate') 
        self.queue(downsampleRate_rf2if, np.uint32, 'downsampleRate_rf2if')
        self.queue(downsampleRate_if2bb, np.uint32, 'downsampleRate_if2bb')
        self.queue(usrp_mixing_freq, np.uint32, 'usrp_mixing_freq')
  

# add a pulse sequence/channel 
class cuda_add_channel_command(driver_command):
    def __init__(self, cudas, sequence = None, swing = -1):
        driver_command.__init__(self, cudas, CUDA_ADD_CHANNEL)
        self.queue(swing, np.uint32, 'swing')
        self.sequence = sequence
 
    def receive(self, sock):
        super().receive(sock)
        self.sequence = pickle_recv(sock)

    def transmit(self):
        if self.sequence == None:
            print('cannot send undefined sequence to channel')
            pdb.set_trace()
        super().transmit()
        for sock in self.clients:
            pickle_send(sock, self.sequence)

# clear one channels from gpu
# TODO: just transmit channel number to save time?
class cuda_remove_channel_command(driver_command):
    def __init__(self, cudas, sequence = None, swing = -1):
        driver_command.__init__(self, cudas, CUDA_REMOVE_CHANNEL)
        self.queue(swing, np.uint32, 'swing')
        self.sequence = sequence 

    def receive(self, sock):
        super().receive(sock)
        self.sequence = pickle_recv(sock)

    def transmit(self):
        if self.sequence == None:
            print('cannot send undefined sequence to channel')
            pdb.set_trace()
        super().transmit()
        for sock in self.clients:
            pickle_send(sock, self.sequence)

# generate rf samples for a pulse sequence
class cuda_generate_pulse_command(driver_command):
    def __init__(self, cudas, swing=-1, mixing_freq=-1 ):
        driver_command.__init__(self, cudas, CUDA_GENERATE_PULSE)
        self.queue(swing, np.uint32, 'swing')
        self.queue(mixing_freq, np.float32, 'mixing_freq')


# re-initialize the usrp driver for a new pulse sequence
class usrp_setup_command(driver_command):
    def __init__(self, usrps, txfreq, rxfreq, txrate, rxrate, npulses, num_requested_rx_samples, num_pause_samples, num_auto_clear_freq, num_requested_tx_samples, pulse_offsets_vector, swing):
        driver_command.__init__(self, usrps, UHD_SETUP)
        
        self.queue(swing , np.int16,   'swing' )
        self.queue(txfreq, np.float64, 'txfreq')
        self.queue(rxfreq, np.float64, 'rxfreq')
        self.queue(txrate, np.float64, 'txrate')
        self.queue(rxrate, np.float64, 'rxrate')
        self.queue(npulses, np.uint32, 'npulses')
        self.queue(num_requested_rx_samples, np.uint64, 'num_requested_rx_samples')
        self.queue(num_pause_samples, np.uint64, 'num_pause_samples')
        self.queue(num_auto_clear_freq, np.uint64, 'num_auto_clear_freq')
        self.queue(num_requested_tx_samples, np.uint64, 'num_requested_tx_samples')
        self.queue(pulse_offsets_vector, np.uint64, 'pulse_offsets_vector')

# set rxfe (amplifier and attenuator) settings 
class usrp_rxfe_setup_command(driver_command):
    def __init__(self, usrps, amp0 = 0, amp1 = 0, att = 0):
        driver_command.__init__(self, usrps, UHD_RXFE_SET)
        self.queue(amp0, np.uint8, 'amp0')
        self.queue(amp1, np.uint8, 'amp1')
        self.queue(att, np.uint8, 'att')

# trigger the start of an integration period
class usrp_trigger_pulse_command(driver_command):
    def __init__(self, usrps, trigger_time, tr_to_pulse_delay, swing):
        driver_command.__init__(self, usrps, UHD_TRIGGER_PULSE)
        self.queue(swing , np.int16,   'swing' )
        self.queue(np.uint32(np.int_(trigger_time)), np.uint32, 'uhd_time_int')
        self.queue(np.float64(np.mod(trigger_time,1)), np.float64, 'uhd_time_frac')
        self.queue(np.float64(tr_to_pulse_delay), np.float64, 'tr_to_pulse_delay')






# command usrp drivers to ready rx sample data into shared memory
class usrp_ready_data_command(driver_command):
    def __init__(self, usrps, swing):
        driver_command.__init__(self, usrps, UHD_READY_DATA)
        self.queue(swing , np.int16,   'swing' )
        
    def receive_all_metadata(self):
       payloadList = []
       for sock in self.clients:
           try:
              payload = {}
              payload['status']   = recv_dtype(sock, np.int32)
              payload['antenna']  = recv_dtype(sock, np.int32)
              payload['nsamples'] = recv_dtype(sock, np.int32)
              payload['fault']    = recv_dtype(sock, np.bool_)
              payloadList.append(payload)
           except:
              payloadList.append(CONNECTION_ERROR)
              self.logger.error("Connection error.")
       return payloadList

    def recv_metadata(self, sock):
        payload = {}
        payload['status']   = recv_dtype(sock, np.int32)
        payload['antenna']  = recv_dtype(sock, np.int32)
        payload['nsamples'] = recv_dtype(sock, np.int32)
        payload['fault']    = recv_dtype(sock, np.bool_)
        return payload

    def recv_samples(self, sock, sock_samples = False):
        super().receive(sock)
        
        if sock_samples:
            self.samples = recv_dtype(sock, np.uint16, nitems = 2 * self.payload['nsamples'])

# command to query usrp time
class usrp_get_time_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_GETTIME)
    
    def recv_time(self, sock):
        self.full_sec = recv_dtype(sock, np.uint32)
        self.frac_sec = recv_dtype(sock, np.float64)
        return self.full_sec + self.frac_sec

# command get auto clear freq samples
class usrp_get_auto_clear_freq_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_AUTOCLRFREQ)
    
    def recv_samples_from_one_usrp(self, sock):
        antenna_no = recv_dtype(sock, np.int32)
        if antenna_no == -1:
            sample_buf = []
        else:
            nSamples = recv_dtype(sock, np.uint32)
            print("receive ant {}: {} samples".format(antenna_no, nSamples))
            time.sleep(0.001)
            sample_buf = recv_dtype(sock, np.int16, nitems = int(2 * nSamples))
            sample_buf = sample_buf[0::2] + 1j * sample_buf[1::2]
        return antenna_no, sample_buf

    def recv_all(self):
        antenna_list = []
        all_samples = []
        for sock in self.clients:
            tmp_ant, tmp_samples = self.recv_samples_from_one_usrp(sock)
            if tmp_ant != -1:
                antenna_list.append(tmp_ant)
                all_samples.append(tmp_samples)
        return antenna_list, all_samples
 
 
# command to query usrp time
class usrp_sync_time_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_SYNC)
    
   
# command to prompt usrp drivers to run clear frequency sequence
class usrp_clrfreq_command(driver_command):
    def __init__(self, usrps, num_clrfreq_samples, clrfreq_uhd_time, clrfreq_freq, clrfreq_rate):
        driver_command.__init__(self, usrps, UHD_CLRFREQ)
        self.queue(num_clrfreq_samples, np.int32, 'num_clrfreq_samples')
        self.queue(np.int32(np.int32(clrfreq_uhd_time)), np.int32, 'clrfreq_uhd_time_int')
        self.queue(np.float64(np.mod(clrfreq_uhd_time,1)), np.float64, 'clrfreq_uhd_time_frac')
        self.queue(clrfreq_freq, np.float64, 'clrfreq_freq')
        self.queue(clrfreq_rate, np.float64, 'clrfreq_rate')

# prompt the usrp driver to cleanly shut down, useful for testing
class usrp_exit_command(driver_command):
    def __init__(self, usrps):
        driver_command.__init__(self, usrps, UHD_EXIT)
        # TODO: what do I need to send here?


# class with pulse sequence information
class sequence(object):
    def __init__(self, npulses, tr_to_pulse_delay, pulse_offsets_vector, pulse_lens, phase_masks, pulse_masks, channelScalingFactor, ctrlprm):
        self.ctrlprm = ctrlprm
        self.npulses = npulses
        self.pulse_offsets_vector = pulse_offsets_vector
        self.pulse_lens = pulse_lens # length of pulses, in seconds
        self.phase_masks = phase_masks # phase masks are complex number to multiply phase by, so, 1 + j0 is no rotation
        self.pulse_masks = pulse_masks
        self.tr_to_pulse_delay = tr_to_pulse_delay
        self.ready = True # TODO: what is ready flag for?
        self.tx_time = self.pulse_lens[0] + 2 * self.tr_to_pulse_delay
        self.channelScalingFactor = channelScalingFactor # correct frequency dependency and superpositioning of channels
         
        # validate input sequence
        if self.ctrlprm['rfreq'] and self.ctrlprm['rfreq'] != self.ctrlprm['tfreq']:
            print('rfreq != tfreq!?, this may not actually be a problem due to clear frequency searches..?')
        #    raise ValueError('rfreq != tfreq, this behavior is not yet supported')
        
        if self.ctrlprm['number_of_samples'] <= 0:
            raise ValueError('number of samples must be greater than zero!')

        if self.npulses == 0:
            raise ValueError('number of pulses must be greater than zero!')

    
        self.sequence_id = uuid.uuid1()

def create_testsequence_uafscan():
    import pickle
    fh = open('uafscan_sequence.pickle', 'rb')
    seq = pickle.load(fh)
    fh.close
    
    return seq
