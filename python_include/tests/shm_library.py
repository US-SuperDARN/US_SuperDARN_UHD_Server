# library for accessing shared memory
# used by tests..
import posix_ipc
import mmap
import os
from cuda_driver import shm_namer

ANTENNA = 0

SWING0 = 0
SWING1 = 1
SIDEA = 0
SIDEB = 1

rx_shm_list = [[],[]]
tx_shm_list = []
rx_semaphore_list = [[],[]] # [side][swing]
swings = [SWING0, SWING1]
sides = [SIDEA]

shm_list = []
sem_list = []

rxshm_size = 160000000
txshm_size = 51200000


def shm_mmap(name):
    memory = posix_ipc.SharedMemory(name)
    shm = mmap.mmap(memory.fd, memory.size)
    os.close(memory.fd)
    return shm

'''
for swing in range(2):
    for side in range(2):
        rx_shm_list[side][swing] = shm_mmap(shm_namer(ANTENNA, swing, side, direction = 'rx'))

'''
