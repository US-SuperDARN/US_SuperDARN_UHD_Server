#!/usr/bin/python3
# 
# function to plot the raw samples from ursp_server in real time 
# usage:
# 

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib import gridspec
import pathlib
import pdb

buffer_flag_file = "../usrp_server/bufferLiveData.flag"
data_file = '../usrp_server/liveRawData.pkl'
sys.path.insert(0, '../python_include')
from drivermsg_library import *


def read_and_plot():
   with open(data_file, 'rb') as f:
      main_data, back_data, parDictList, ant_idx_main, ant_idx_back= pickle.load(f)

   os.remove(data_file)
   with open(buffer_flag_file, "w+") as f:
      pass

   # put dataqueue in dict
   for parDict in parDictList:
      for item in parDict['ctrlprm_dataqueue']:
         parDict[item.name] = item.data

      #print(parDict)
      for key in parDict.keys():
         if key != 'ctrlprm_dataqueue':
            print(" {} = {}".format(key, parDict[key]))
      print("main_beamformed: {}".format(len(parDict['main_beamformed'])))

   antennaPlotData = np.array(main_data)

   nChannels = len(antennaPlotData)
   nAntennas = len(antennaPlotData[0] )
   nSamples =  len(antennaPlotData[0][0])
   nSequences = parDict['nSequences_per_period']
   nSamples_sequence = parDict['number_of_samples']
   nSamples_sequence_padded = parDict["nbb_rx_samples_per_sequence"]
   # assert nSamples == nSamples_sequence * nSequences
   print("nChannels {}, nAntennas {}, nSamples {}, nSequences {}, nSamples/seq {}, nSamples_padded/seq {}".format(nChannels, nAntennas, nSamples, nSequences, nSamples_sequence, nSamples_sequence_padded))

   average_active = False

   for iChannel in range(nChannels):
#       pdb.set_trace()
      beamformedPlotData = np.int16(parDictList[iChannel]['main_beamformed'] >> 16) + 1j* np.int16(parDictList[iChannel]['main_beamformed'] & 0xffff)
      if average_active:
         channelPlotData = np.reshape(antennaPlotData[iChannel], [nAntennas, nSequences, nSamples_sequence_padded ])
         channelPlotData = np.mean(channelPlotData, axis=1)
         beamformedPlotData = np.reshape(beamformedPlotData, [nSequences, nSamples_sequence_padded ])
         beamformedPlotData = np.mean(beamformedPlotData, axis=0)
         xAxis_scaling = 1
         xAxis_label = "BB Samples"
      else:
         channelPlotData = antennaPlotData[iChannel]
         xAxis_scaling = nSamples_sequence_padded
         xAxis_label = "Sequences"

      for iAnt in range(nAntennas):
         plt.subplot(int(np.ceil(nAntennas/2)), 2, 1+iAnt)
         if iChannel == 0:
            plt.cla()
         plt.plot([i /xAxis_scaling for i in range(len(channelPlotData[iAnt]))], np.real(channelPlotData[iAnt]))
         plt.plot([i /xAxis_scaling for i in range(len(channelPlotData[iAnt]))], np.imag(channelPlotData[iAnt]))
         plt.grid(True)

         # start of sequences
         seqStartTimes = np.array(parDict['sequence_start_time_secs']) + np.array(parDict["sequence_start_time_usecs"]) / 1e6
         seqStartTimes = seqStartTimes - seqStartTimes[0]
         seqValueVec = [100000 for i in range(len(seqStartTimes))]
         #plt.plot( seqStartTimes, seqValueVec, 'o')
         plt.ylabel("ant {} ".format(ant_idx_main[iAnt]) )
         if iAnt == 0:
            plt.title("nSequences_per_period={}, tbeam={}, rfreq={}".format(parDict["nSequences_per_period"], parDict["tbeam"] , parDict["rfreq"]), )
         elif iAnt in [nAntennas-1,nAntennas-2]:
            plt.xlabel(xAxis_label)

      # plot beamformed data
  #     plt.subplot(413)
  #     if iChannel == 0:
  #        plt.cla()
  #     plt.plot([i /xAxis_scaling for i in range(len(beamformedPlotData))], 20*np.log10(np.abs(beamformedPlotData)/np.iinfo(np.int16).max/np.sqrt(2)))
  #     pulse_times = np.array(parDictList[0]['pulse_sequence_offsets_vector'] ) * parDictList[0]['baseband_samplerate']
  #     plt.scatter(pulse_times, pulse_times*0)
  #     plt.grid(True)
  #     plt.ylabel("Beamformed (dB)")
  #     plt.xlabel(xAxis_label)

  # plt.subplot(414)
  # plt.cla()
  # idx2checkVec = [0, 70, 109]
  # colorList = cm.rainbow(np.linspace(0,1,len(idx2checkVec)))
  # for idx2check, plotColor in zip(idx2checkVec, colorList):
  #    exampleTXsamples = [antennaPlotData[0][iAnt][idx2check] for iAnt in range(nAntennas)]
  #    arrayAngle = np.angle(exampleTXsamples, deg=True)
  #    arrayAngle = arrayAngle - arrayAngle[0]
  #    plt.scatter(np.arange(nAntennas)+(np.random.rand(1)-0.5)/5, arrayAngle % 360, s=5*np.log(np.abs(exampleTXsamples)), color=plotColor)

  # beamsep = 3.24 
  # delta_x = 15.4
  # nBeams = 16
  # for iChannel in range(nChannels):
  #    alpha = beamsep * (parDictList[iChannel]["tbeam"] - (nBeams -1) /2 )
  #    print("alpha is {} (beam {})".format(alpha, parDictList[iChannel]['tbeam']))
  #    phaseDiff_per_ant = - delta_x / 3e8 * parDictList[iChannel]["rfreq"] *1000 * np.sin(alpha/180*np.pi) *360
  #    plt.plot(np.arange(nAntennas), [phaseDiff_per_ant*iAnt % 360 for iAnt in range(nAntennas)])
  # plt.grid(True)
  # plt.ylabel("phase difference in deg")
  # plt.xlabel("antenna number")
  # plt.axis([-0, nAntennas+3, 0, 360])
  # legendList = ["Measured sample {} (ch 0)".format(idx) for idx in idx2checkVec]
  # for iChannel in range(nChannels):
  #    legendList.insert(iChannel, "Theory (ch {})".format(iChannel))
  # plt.legend(legendList)
   plt.pause(0.05)

plt.figure()
#plt.axis([0, 10, 0, 1])
plt.ion()
with open(buffer_flag_file, "w+") as f:
   pass

while True:
   while os.path.isfile(buffer_flag_file):
      plt.pause(0.1)
   print("Starting read and plot")
   read_and_plot()

