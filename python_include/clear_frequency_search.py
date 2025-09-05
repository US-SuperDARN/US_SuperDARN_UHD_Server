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
import time
from drivermsg_library import *
from rosmsg import *


DEBUG = 1
def dbPrint(msg):
   if DEBUG:
     print("clear_frequency_search.py : " + msg)


def record_clrfreq_raw_samples(usrp_sockets, num_clrfreq_samples, center_freq, clrfreq_rate_requested, min_clrfreq_delay):
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

    clrfreq_time = usrptime + min_clrfreq_delay

    dbPrint("current UHD time: {}, scheduling clrfreq for: {}".format(usrptime, clrfreq_time))

    clrfreq_cmd = usrp_clrfreq_command(usrp_sockets, num_clrfreq_samples, clrfreq_time, center_freq, clrfreq_rate_requested)
    clrfreq_cmd.transmit()

    time.sleep(0.001)

    output_antenna_idx_list, output_samples_list = clrfreq_cmd.recv_all(num_clrfreq_samples)

    try:
        clrfreq_cmd.client_return()
        dbPrint("record sample command completed")
    except:
        dbPrint("CLRFREQ, communication with at least one USRP failed")

    return output_samples_list, output_antenna_idx_list


if __name__ == '__main__':
    pass

    

