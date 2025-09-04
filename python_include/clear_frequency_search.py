# tools for clear frequency search, used by usrp_server.py
# timeline of clear frequency search:
# control program sends RequestClearFreqSearch command to a channel
#   channel enters CLR_FREQ state, waits for hardware manager to start a clear frequency search
#   
# control program sends RequestAssignedFreq command, channel responds with tfreq in kHz and noise
#   channel uses self.tfreq and snsmitelf.noise 

# this replaces the following legacy qnx code:
# ros/server/reciever_handler.c receiver_assign_frequency
# ros/server/main.c:273, reading restrict file
# based on gc316_tcp_driver/main.c, fetching samples and signal processing
import numpy as np
import time
from drivermsg_library import *
from rosmsg import *
from radar_config_constants import *


DEBUG = 1
def dbPrint(msg):
   if DEBUG:
     print("clear_frequency_search.py : " + msg)


def record_clrfreq_raw_samples(usrp_sockets, num_clrfreq_samples, center_freq, clrfreq_rate_requested):
    dbPrint("enter record_clrfreq_raw_samples")
    num_clrfreq_samples = int(num_clrfreq_samples)
    output_samples_list     = []
    output_antenna_idx_list = []
    clrfreq_rate_actual = 0

    # gather current UHD time
    dbPrint("send usrp_get_time")
    gettime_cmd = usrp_get_time_command(usrp_sockets)
    gettime_cmd.transmit()
    
    usrp_times = []
    for sock in usrp_sockets:
        try:
            usrp_times.append( gettime_cmd.recv_time(sock))
        except:
            pass

    usrptime = usrp_times[0]

    gettime_cmd.client_return()

    # schedule clear frequency search in MIN_CLRFREQ_DELAY seconds
    dbPrint(" send clrfreq_command")

    clrfreq_time = usrptime + MIN_CLRFREQ_DELAY

    dbPrint("current UHD time: {}, scheduling clrfreq for: {}".format(usrptime, clrfreq_time))

    clrfreq_cmd = usrp_clrfreq_command(usrp_sockets, num_clrfreq_samples, clrfreq_time, center_freq, clrfreq_rate_requested)
    clrfreq_cmd.transmit()
    time.sleep(0.001)
    
    # if usrp_sockets[0].getpeername()[0] != '127.0.0.1':
    #    time.sleep(0.01)
    
    dbPrint("CLRFREQ command sent, waiting for raw samples")
    # grab raw samples
    for usrpsock in usrp_sockets:
        try:
           nSides=recv_dtype(usrpsock, np.int32)

           dbPrint("CLRFREQ nSides {}".format(nSides))
             
           for j in range(nSides):
              antenna_no_tmp = recv_dtype(usrpsock, np.int32)
              clrfreq_rate_actual = recv_dtype(usrpsock, np.float64)
              assert clrfreq_rate_actual == clrfreq_rate_requested
              
              #dbPrint("antenna {} clrfreq rate is: {} (requested: {})".format(output_antenna_idx_list[-1], clrfreq_rate_actual, clrfreq_rate_requested))
              dbPrint("antenna {} waiting for {} samples".format(antenna_no_tmp, int(num_clrfreq_samples)))
              
              time.sleep(0.002)
              if usrpsock.getpeername()[0] != '127.0.0.1':
                 time.sleep(0.002)
                 
              sample_buf = recv_dtype(usrpsock, np.int16, nitems = int(2 * num_clrfreq_samples))
              
              dbPrint("CLRFREQ number samples {}".format(len(sample_buf)))
              
              output_samples_list.append(sample_buf[0::2] + 1j * sample_buf[1::2])
              output_antenna_idx_list.append(antenna_no_tmp)
              
        except:
           dbPrint("CLRFREQ response from {} failed.".format(usrpsock))
           
           
    try:
        clrfreq_cmd.client_return()
        dbPrint("record sample command completed")
    except:
        dbPrint("CLRFREQ, communication with at least one USRP failed")

    return output_samples_list, output_antenna_idx_list


if __name__ == '__main__':
    pass

    

