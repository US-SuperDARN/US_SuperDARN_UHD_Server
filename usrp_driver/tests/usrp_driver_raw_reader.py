#!/usr/bin/python3
# reads in raw sample dump from usrp_driver
import sys

filename = "diag/raw_samples_tx_ant_0.cint16"

cfreq = 13e6
rfrate = 10e6
sys.path.insert(0, '../python_include')

from myPlotTools import *
from pylab import *


with open(filename, 'r') as raw_file:
    subplot(211)
    samples = np.fromfile(raw_file, dtype=np.int16)
    samples = samples[0::2] + 1j * samples[1::2]

#    plot_freq(samples, rfrate)
    plot(samples)

    subplot(212)
    nSamplesPP = 8192
    nPulses = int(len(samples) /nSamplesPP)
    print(nPulses)
    for iPulse in range(nPulses):
        plot(samples[iPulse*nSamplesPP:(iPulse+1)*nSamplesPP ])

    show()
    import pdb
    pdb.set_trace()

