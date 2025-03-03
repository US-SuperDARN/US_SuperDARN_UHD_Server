import numpy as np
import collections.abc as collections

from drivermsg_library import driver_command, NO_COMMAND

SET_RADAR_CHAN = 'R'
SET_INACTIVE = 'a'
SET_ACTIVE = 'A'
QUERY_INI_SETTINGS = 'i'
GET_SITE_SETTINGS = 's'
UPDATE_SITE_SETTINGS = 'S'

GET_PARAMETERS = 'c'
SET_PARAMETERS = 'C'
SET_PARAMETERS_I = 'D'

PING = '='
OKAY = '^'
NOOP = '~'
QUIT = '.'
ExitServer = ','

REGISTER_SEQ = '+'
REMOVE_SEQ = '-'

REQUEST_ASSIGNED_FREQ = '>'
REQUEST_CLEAR_FREQ_SEARCH = '<'
LINK_RADAR_CHAN = 'L'

SET_READY_FLAG = '1'
UNSET_READY_FLAG = '!'
SET_PROCESSING_FLAG = '2'
UNSET_PROCESSING_FLAG = '@'

GET_DATA = 'd'

WAIT_FOR_DATA = 'w'

RMSG_SUCCESS = True
MSG_FAILURE = False

RMSG_COMMAND_NAMES = {'SET_RADAR_CHAN' : 'R', 'SET_INACTIVE' : 'a', 'SET_ACTIVE' : 'A', 'QUERY_INI_SETTINGS' : 'i', \
        'GET_SITE_SETTINGS' : 's', 'UPDATE_SITE_SETTINGS' : 'S', 'GET_PARAMETERS' : 'c', 'SET_PARAMETERS' : 'C', 'SET_PARAMETERS_I' : 'D', \
        'PING' : '=', 'OKAY' : '^', 'NOOP' : '~', 'QUIT' : '.', 'ExitServer' : ExitServer, 'REGISTER_SEQ' : '+', 'REMOVE_SEQ' : '-', \
        'REQUEST_ASSIGNED_FREQ' : '>', 'REQUEST_CLEAR_FREQ_SEARCH' : '<', 'LINK_RADAR_CHAN' : 'L', 'SET_READY_FLAG' : '1',\
        'UNSET_READY_FLAG' : '!', 'SET_PROCESSING_FLAG' : '2', 'UNSET_PROCESSING_FLAG' : '@', 'GET_DATA' : 'd'}
RMSG_COMMAND_NAMES = {v: k for k, v in RMSG_COMMAND_NAMES.items()} 



class rosmsg_command(driver_command):
    def __init__(self, client):
        driver_command.__init__(self, [client], NO_COMMAND)
        
        rosmsg_dict = {}

        rosmsg_dict['status'] = 0
        rosmsg_dict['type'] = 0

        self.queue(rosmsg_dict['status'], np.int32, 'status')
        self.queue(rosmsg_dict['type'], np.uint32, 'type') # TODO: why is this a uint32_t...


class rprm_struct(driver_command):
    def __init__(self, socket):
        driver_command.__init__(self, [socket], NO_COMMAND)
        
        prm_dict = {}

        prm_dict['site'] = 0
        prm_dict['radar'] = 0
        prm_dict['channel'] = 0

        self.queue(prm_dict['site'], np.int32, 'site')
        self.queue(prm_dict['radar'], np.int32, 'radar')
        self.queue(prm_dict['channel'], np.int32, 'channel')

class clrfreqprm_struct(driver_command):
    def __init__(self, socket):
        driver_command.__init__(self, [socket], NO_COMMAND)

        prm_dict = {}

        prm_dict['start'] = 0               # start frequency of search, units of kHz
        prm_dict['end'] = 0                 # end frequency of search, units of kHz
        prm_dict['filter_bandwidth'] = 0    # bandwidth of search, units of kHz
        prm_dict['pwr_threshold'] = 0       # seems to be unused? 
        prm_dict['nave'] = 0                # number of averages (capped at 10 by gc316 driver..?)

        self.queue(prm_dict['start'], np.int32, 'start')
        self.queue(prm_dict['end'], np.int32, 'end')
        self.queue(prm_dict['filter_bandwidth'], np.float32, 'filter_bandwidth')
        self.queue(prm_dict['pwr_threshold'], np.float32, 'pwr_threshold')
        self.queue(prm_dict['nave'], np.int32, 'nave')

class seqprm_struct(driver_command):
    def __init__(self, socket):
        driver_command.__init__(self, [socket], NO_COMMAND)

        prm_dict = {}

        prm_dict['index'] = 0
        prm_dict['len'] = 0
        prm_dict['step'] = 0
        prm_dict['samples'] = 0
        prm_dict['smdelay'] = 0

        self.queue(prm_dict['index'], np.uint32, 'index')
        self.queue(prm_dict['len'], np.uint32, 'len')
        self.queue(prm_dict['step'], np.uint32, 'step')
        self.queue(prm_dict['samples'], np.uint32, 'samples')
        self.queue(prm_dict['smdelay'], np.uint32, 'smdelay')

class dataprm_struct(driver_command):
    def __init__(self, socket):
        driver_command.__init__(self, [socket], NO_COMMAND)

        prm_dict = {}
        
        # todo: populate event_secs and event_nsecs from GPS time (or USRP time?)
        prm_dict['event_secs'] = 0      # uint32, start of pulse sequence, seconds, filled by site library if zero
        prm_dict['event_nsecs'] = 0     # uint32, start of pulse sequence, nanoseconds (used only for seqlog?)
        prm_dict['event_capture'] = 0   # flag used for seqlog 
        prm_dict['samples'] = 0         # number of baseband rx samples
        prm_dict['shm_memory'] = 0      # ? 
        prm_dict['status'] = 0          # status, 0 is good
        prm_dict['frame_header'] = 0    # ? 
        prm_dict['bufnum'] = 0          # ?

        self.queue(prm_dict['event_secs'], np.uint32, 'event_secs')
        self.queue(prm_dict['event_nsecs'], np.uint32, 'event_nsecs')
        self.queue(prm_dict['event_capture'], np.uint32, 'event_capture')
        self.queue(prm_dict['samples'], np.int32, 'samples')
        self.queue(prm_dict['shm_memory'], np.int32, 'shm_memory')
        self.queue(prm_dict['status'], np.int32, 'status')
        self.queue(prm_dict['frame_header'], np.int32, 'frame_header')
        self.queue(prm_dict['bufnum'], np.int32, 'bufnum')

class ctrlprm_struct(driver_command):
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
        self.queue(name_arr, np.uint8, nitems = 80)

        description_arr = np.uint8(np.zeros(120))
        self.queue(description_arr, np.uint8, nitems = 120)
