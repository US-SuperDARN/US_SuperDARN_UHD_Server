#!/usr/bin/python3

import os
import matplotlib.pyplot as plt
import pickle

dump_path = '/data/diagnostic_samples/'

fileName = 'cuda_dump_tx_2023-06-15_075303.pkl'
with open(os.path.join(dump_path, fileName), 'rb') as f:
    # samples, bb_samples = pickle.load(f)
    samples = pickle.load(f)
print(len(samples))
print(len(bb_samples[0]))


for idx,ant_samples in enumerate( samples):
    print(idx)
    print(len(ant_samples))
    print(len(bb_samples[0][idx]))
  #  continue
    plt.subplot(10, 2, idx+1)
    plt.plot(ant_samples[500:2000:2])
    plt.plot(bb_samples[0][idx][0:20:2])
    plt.ylabel('idx {}'.format(idx))
plt.show()
