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
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal
from drivermsg_library import *
from rosmsg import *
from phasing_utils import calc_beam_azm_rad, calc_phase_increment, rad_to_rect, beamform_uhd_samples
from radar_config_constants import *

RESTRICTED_POWER = 1e12 # arbitrary high power for restricted frequency
RESTRICT_FILE = '/home/radar/repos/SuperDARN_MSI_ROS/linux/home/radar/ros.3.6/tables/superdarn/site/site.ksr/restrict.dat.inst'

SAVE_CLEAR_FREQUENCY_SEARCH = False 
CLEAR_FREQUENCY_DUMP_DIR = '/data/logs/clearfreq_logs/'


DEBUG = 1
def dbPrint(msg):
   if DEBUG:
     print("clear_frequency_search.py : " + msg)


def read_restrict_file(restrict_file):
    restricted_frequencies = []
    with open(restrict_file, 'r') as f:
        for line in f:
            if line[0] == '#' or line[0] == 'd' or len(line) < 8:
                continue
            line = line.split(' ')
            restrict_start = int(line[0]) * 1e3 # convert kHz units in restrict to Hz
            restrict_end = int(line[1]) * 1e3 # convert kHz units in restrict to Hz
            restricted_frequencies.append([restrict_start, restrict_end])

    return restricted_frequencies; 

def calc_clear_freq_on_raw_samples(raw_samples, sample_meta_data, restricted_frequencies, clear_freq_range, beam_angle, smsep):
    # unpack meta data
    antennas = np.array(sample_meta_data['antenna_list'])
    num_samples = int(sample_meta_data['number_of_samples'])

    # calculate phasing vector 
    phase_increment = calc_phase_increment(beam_angle, np.mean(clear_freq_range) * 1000, sample_meta_data['x_spacing'])
    phasing_vector =np.array([rad_to_rect(ant * phase_increment) for ant in antennas])

    # mute back array antennas
    phasing_vector[np.logical_and(antennas > 15, antennas < 20)] = 0

    # apply beamforming 
    #beamformed_samples = beamform_uhd_samples(raw_samples, phasing_matrix, num_samples, antennas, False)
    beamformed_samples = np.array(phasing_vector * np.matrix(raw_samples))[0]

    # apply spectral estimation 
    spectrum_power = fft_clrfreq_samples(beamformed_samples)

   
    # calculate spectrum range of rf samples given sampling rate and center frequency
    freq_vector = np.fft.fftshift(np.fft.fftfreq(num_samples, 1/sample_meta_data['usrp_rf_rate'])) + sample_meta_data['usrp_fcenter']*1000
 
    # mask restricted frequencies
    if restricted_frequencies:
        spectrum_power = mask_spectrum_power_with_restricted_freqs(spectrum_power, freq_vector, restricted_frequencies)
   
    # search for a clear frequency within the given frequency range
    clear_bw = 2e6/smsep
    tfreq, noise = find_clrfreq_from_spectrum(spectrum_power, freq_vector, clear_freq_range[0] * 1e3, clear_freq_range[1] * 1e3, clear_bw = clear_bw)
    
    if SAVE_CLEAR_FREQUENCY_SEARCH:
        import pickle
        import time
        clr_time = time.time()
        pickle.dump({'time':clr_time, 'raw_samples': raw_samples, 'sample_data':sample_meta_data, 'clrfreq':tfreq, 'noise':noise, 'freq_vector':freq_vector,  'power':spectrum_power, 'clear_freq_range':clear_freq_range, 'phasing_vector':phasing_vector, 'beam_angle':beam_angle}, open(CLEAR_FREQUENCY_DUMP_DIR + 'clrfreq_dump.'  + str(clr_time) + '.pickle', 'wb'))

    return np.round(tfreq), noise

def mask_spectrum_power_with_restricted_freqs(spectrum_power, spectrum_freqs, restricted_frequencies):
    import pdb
    for freq in restricted_frequencies:
        restricted_mask = np.logical_and(spectrum_freqs > freq[0], spectrum_freqs < freq[1])
        spectrum_power[restricted_mask] = RESTRICTED_POWER

    return spectrum_power

def find_clrfreq_from_spectrum(spectrum_power, spectrum_freqs, fstart, fstop, clear_bw = 40e3):
    dbPrint("enter find_clrfreq_from_spectrum")
    # apply filter to convolve spectrum with filter response
    # TODO: filter response is currently assumed to be boxcar..
    # return lowest power frequency
    channel_filter = np.ones(int(clear_bw / CLRFREQ_RES))
    channel_power = scipy.signal.correlate(spectrum_power, channel_filter, mode='same')
         
    # mask channel power spectrum to between fstart and fstop
    usable_mask = (spectrum_freqs > fstart) * (spectrum_freqs < fstop)
    channel_power = channel_power[usable_mask]
    spectrum_freqs = spectrum_freqs[usable_mask]
    
    # find lowest power channel
    clrfreq_idx = np.argmin(channel_power) 
    
    clrfreq = spectrum_freqs[clrfreq_idx] / 1e3
    noise = channel_power[clrfreq_idx]
    
    return clrfreq, noise

def fft_clrfreq_samples(samples):
    # return fft of width usable_bandwidth, kHz resolution
    power_spectrum = np.fft.fftshift(np.abs(np.fft.fft(samples, norm = 'ortho')) ** 2)
    return power_spectrum 

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

    dbPrint("CLRFREQ command sent, waiting for raw samples")
    # grab raw samples
    for usrpsock in usrp_sockets:
        try:
            antenna_no_tmp = recv_dtype(usrpsock, np.int32)
            clrfreq_rate_actual = recv_dtype(usrpsock, np.float64)
            assert clrfreq_rate_actual == clrfreq_rate_requested

            #dbPrint("antenna {} clrfreq rate is: {} (requested: {})".format(output_antenna_idx_list[-1], clrfreq_rate_actual, clrfreq_rate_requested))
            dbPrint("antenna {} waiting for {} samples".format(antenna_no_tmp, int(num_clrfreq_samples)))
           
            sample_buf = recv_dtype(usrpsock, np.int16, nitems = int(2 * num_clrfreq_samples))

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

    

