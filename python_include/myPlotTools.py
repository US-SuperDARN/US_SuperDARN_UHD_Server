import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter
import pdb

def plot_time(timeData, samplingRate, iqInterleaved=False, show=True, ax=None, dB=False   ):

   if ax == None:
      ax = plt.gca()

   formatter = FuncFormatter(niceUnitPrefix_formatter)
   ax.xaxis.set_major_formatter(formatter )  # nice SI unit prefixes

   if iqInterleaved:  # would be smart you use real and imag directly
      timeData = timeData[0::2] + 1j* timeData[1::2]

   tVec = [ t / samplingRate for t in range(timeData.size) ]
     
   if dB:
     plotData_real = 20*np.log10(np.absolute(np.real(timeData)))
     plotData_imag = 20*np.log10(np.absolute(np.imag(timeData)))
     yLabelString  = 'amplitude in dB'
   else:
     plotData_real = np.real(timeData)
     plotData_imag = np.imag(timeData)
     yLabelString  = 'amplitude'

   plt.plot(tVec, plotData_real , label='real', marker=".")
   plt.plot(tVec, plotData_imag , label='imag', marker=".")

   plt.grid(True)
   ax.set_xlim([tVec[0], tVec[-1]])
   plt.xlabel('time in s')
   plt.legend(loc=0)
   plt.ylabel(yLabelString)	
   if show:
      plt.show()


def plot_freq(timeData, samplingRate, iqInterleaved=False, show=True, ax=None, dB=True   ):

   if ax == None:
      ax = plt.gca()

   ax.xaxis.set_major_formatter( FuncFormatter(niceUnitPrefix_formatter)) # nice SI unit prefixes

   if iqInterleaved:
      timeData = timeData[0::2] + 1j* timeData[1::2]

   tVec = [ t / samplingRate for t in range(timeData.size) ]
   
   spk = np.fft.fft(timeData) # *2 / timeData.size
   freqVec = np.fft.fftfreq(timeData.size, 1/samplingRate )
   sortIdx = np.argsort(freqVec)
   
   if dB:
     plotData_real = 20*np.log10(np.absolute(np.real(spk[sortIdx])))
     plotData_imag = 20*np.log10(np.absolute(np.imag(spk[sortIdx])))
     yLabelString  = 'magintude in dB'
   else:
     plotData_real = np.real(spk[sortIdx])
     plotData_imag = np.imag(spk[sortIdx])
     yLabelString  = 'magnitude'

   plt.plot(freqVec[sortIdx], plotData_real , label='real', marker=".")
   plt.plot(freqVec[sortIdx], plotData_imag , label='imag', marker=".")

   plt.grid(True)
   ax.set_xlim([freqVec[sortIdx[1]], freqVec[sortIdx[-1]]])
   plt.xlabel('frequency in Hz')
   plt.legend(loc=0)
   plt.ylabel(yLabelString)	
   if show:
      plt.show()

def plot_time_freq(timeData, samplingRate, iqInterleaved=False, show=True,  time_dB=False, freq_dB=True   ):
  
  # plt.figure()
   ax = plt.subplot(121)
   plot_time(timeData, samplingRate, iqInterleaved=iqInterleaved, show=False, dB=time_dB )

   ax = plt.subplot(122)

   plot_freq(timeData, samplingRate, iqInterleaved=iqInterleaved, show=False, dB=freq_dB )
   if show:
      plt.show()



def niceUnitPrefix_formatter(value, pos):
    if value == 0:
        return '0'
    
    unitPrefixes = 'afpnµm kMGTPE'
    prefixIdx = np.floor(np.log10(np.absolute(value)) / 3)
    prefix = unitPrefixes[int(prefixIdx)+6]
    multiplyFactor = 10 ** (-prefixIdx*3)
    
    if np.mod(value*multiplyFactor, 1) == 0:
        strTemplate = '{:.0f}{}'
    else:
        strTemplate = '{}{}'
    outputStr = strTemplate.format(np.round(value*multiplyFactor*1.0e12)/1.0e12, prefix )
    return outputStr
    
