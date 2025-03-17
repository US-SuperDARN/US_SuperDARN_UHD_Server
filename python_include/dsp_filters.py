# all filters for RX downsampling and smooting TX pulses
import numpy as np
import scipy.ndimage
from scipy import signal


# filter for TX pulses
def gaussian_pulse(samples, trise, rate):
# samples - input samples of rate rate
# trise - rise time, in seconds
# rate - sampling rate, in Hz
    # generate filter coefficients
    gauss_sigma = rate * (trise)

    filt_real = scipy.ndimage.filters.gaussian_filter1d(np.real(samples), gauss_sigma)
    filt_imag = scipy.ndimage.filters.gaussian_filter1d(np.imag(samples), gauss_sigma)

    return filt_real + 1j * filt_imag

# filter also includes mixing frequencies
def kaiser_filter_s0(nTaps, channelFreqVec, samplingRate, normalize = True):
    gain = 3.73 # was 3.5 before
    beta = 5.25
    #beta = 10
    filterData = np.zeros((len(channelFreqVec), nTaps,2), dtype=np.float32)
    m = nTaps - 1
    b = scipy.special.i0(beta)

    for iChannel, channelFreq in enumerate(channelFreqVec):
        if channelFreq != None:
          # dbPrint('filter generation Channel {}: Freq : {} kHz'.format(iChannel, channelFreq/1e3))
           for iTap in range(nTaps):
               phi = 2 * np.pi * channelFreq * iTap / samplingRate # phase of downmixing frequency
               k = scipy.special.i0((2 * beta / m) * np.sqrt(iTap * (m - iTap)))
               filterData[iChannel,iTap,0] = gain * (k / b)# * np.cos(phi)
               filterData[iChannel,iTap,1] = gain * (k / b)# * np.sin(phi) # still real filter, imag part result from multiplication with oscillator
        else:
           dbPrint("filter generation: channel {}: skipping because undefined".format(iChannel))

    if normalize:
        filterData /= nTaps * 2

    return filterData

#TODO: test this filter
def rect_filter_s0(nTaps, channelFreqVec, samplingRate):
#simple filter: real = 1, image = 0
    filterData = np.zeros((len(channelFreqVec), nTaps,2), dtype=np.float32)
    for iChannel, channelFreq in enumerate(channelFreqVec):
        if channelFreq != None:
           dbPrint('filter generation Channel {}: Freq : {} kHz'.format(iChannel, channelFreq/1e3))
           for iTap in range(nTaps):
               phi = 2 * np.pi * channelFreq * iTap / samplingRate # phase of downmixing frequency
               filterData[iChannel,iTap,0] = np.cos(phi)
               filterData[iChannel,iTap,1] = np.sin(phi) # still real filter, imag part result from multiplication with oscillator
        else:
           dbPrint("filter generation: channel {}: skipping because undefined".format(iChannel))
    return filterData


def kaiser_filter_r0(nTaps, channelFreqVec, normalize = True, beta=5, gain=3.73):
    # gain = 3.73 # was 3.5 before
    filterData = np.zeros((len(channelFreqVec), nTaps,2), dtype=np.float32)
    m = nTaps - 1
    b = scipy.special.i0(beta)

    for iChannel, channelFreq in enumerate(channelFreqVec):
        if channelFreq != None:
          # dbPrint('filter generation Channel {}: Freq : {} kHz'.format(iChannel, channelFreq/1e3))
            window=signal.windows.kaiser(nTaps,beta=beta)
            for iTap in range(nTaps):
               filterData[iChannel,iTap,0] = gain * window[iTap]
        else:
           dbPrint("filter generation: channel {}: skipping because undefined".format(iChannel))

    if normalize:
        filterData /= nTaps * 2

    return filterData




def raisedCosine_filter(nTaps, nChannels, normalize = True):

    alpha = 0.22
    filterData = np.zeros((nChannels, nTaps, 2), dtype=np.float32)
    for iTap in range(nTaps-1):
        t = 2 * iTap - (nTaps-1)
        if t == 0:
           filterData[:,iTap,0] = 1
        elif np.absolute(t) ==  (nTaps-1)/(2*alpha):
           filterData[:,iTap,0] = np.sin(np.pi/(2*alpha)) / (np.pi/(2*alpha)) * np.pi/4
        else:
           filterData[:,iTap,0] = np.sin(np.pi*t/(nTaps-1)) / (np.pi*t/(nTaps-1)) * np.cos(alpha*np.pi*t / (nTaps-1)) / (1-2*(alpha*t/(nTaps-1))**2)
        
    if normalize:
        filterData /= np.sum(filterData[0]) 

    return filterData

def dbPrint(msg):
    print(' {}: {} '.format(__file__, msg) )

# functions to test filters
if __name__ == '__main__':
    from pylab import *
    from scipy import signal
    import pdb

    ntaps_rfif = 40
    ntaps_ifbb = 150
    rfif_rate = 20
    fsamp_rf = 5e6
    fsamp_if = fsamp_rf / rfif_rate

    kaiser_filter = kaiser_filter_s0(ntaps_rfif, [1e6], fsamp_rf)
    raised_cosine = raisedCosine_filter(ntaps_ifbb, 1)
    
    kaiser_filter = (kaiser_filter[0].T[0] + 1j * kaiser_filter[0].T[1])
    raised_cosine = (raised_cosine[0].T[0] + 1j * raised_cosine[0].T[1])
    
    kw, kh = signal.freqz(kaiser_filter) 
    cw, ch = signal.freqz(raised_cosine) 

    plt.subplot(2,1,1)
    plt.plot(kw, 20 * np.log10(abs(kh)))
    plt.title('kaiser filter')
    plt.ylabel('Amplitude [dB]')
    plt.xlabel('Frequency [rad/sample]')

    plt.subplot(2,1,2)
    plt.title('raised cosine filter')
    plt.plot(cw, 20 * np.log10(abs(ch)))

    plt.ylabel('Amplitude [dB]')
    plt.xlabel('Frequency [rad/sample]')
    plt.show()

    pdb.set_trace()






