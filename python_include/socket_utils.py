import numpy as np
import pickle
import socket

verbose = False 
# DEL
# pack i/q samples into uint32 for sending samples over the network
# between usrp_server and arbyserver
# def complex_int32_pack(isamp, qsamp):
#     i_mask = 0xffff0000
#     q_mask = 0x0000ffff
#     maxAbsValues = 32767
#     for inputValue in [isamp, qsamp]:
#         if inputValue < - maxAbsValues or inputValue > maxAbsValues:
#             OverflowError("socket_utils.py:complex_int32_pack: Over/underflow Error. Input value: {}".format(inputValue)) 
# 
#     packed_sample = (i_mask & (np.int16(isamp) << 16)) + (q_mask & np.int16(qsamp))
#     # port of:
#     #client_main[isamp] = ((uint32_t) (temp_main[1] << 16) & 0xffff0000) | ((uint32_t) temp_main[0] & 0x0000ffff);
#     return np.uint32(packed_sample)

def recv_dtype(sock, dtype, nitems = 1):
    if dtype == str:
        data = sock.recv(nitems, socket.MSG_WAITALL)
    else:
        dstr = sock.recv(dtype().nbytes * nitems, socket.MSG_WAITALL)
        if verbose:
            print(' => {}  received ?? as {} ({} / {}  bytes): {}'.format(__file__, dtype, len(dstr), dtype().nbytes * nitems , dstr  ))
        data = np.fromstring(dstr, dtype=dtype, count=nitems)
    #except ValueError:
    #    import logging 
    #    print('timed out waiting for ' + str(dtype))
    #    import pdb
    #    pdb.set_trace()
    if nitems == 1:
        return data[0]
    return data

# pickle send/recv for sending arbitrary objects over the network between python processes
# could use json for compatibility with other languages..
# mostly to send large messy sequence objects to cuda_driver to generate baseband samples
def pickle_send(sock, data):
    serialized_data = pickle.dumps(data, pickle.HIGHEST_PROTOCOL)
    #print('sending pickle with size: {} bytes'.format(np.uint32(len(serialized_data))))
    transmit_dtype(sock, np.uint32(len(serialized_data)))
    sock.sendall(serialized_data)

def pickle_recv(sock):
    pickle_len = recv_dtype(sock, np.uint32)
    #print('expecting pickle with size: {} bytes'.format(pickle_len))
    pickle_data = sock.recv(pickle_len, socket.MSG_WAITALL)
    data = pickle.loads(pickle_data)
    return data

def transmit_dtype(sock, data, dtype = None):
    # TODO: handle vectors..
    if dtype != None:
        data = dtype(data)

    dstr = sock.sendall(data.tobytes())
    if verbose:
        print(' => {}  transmitted {} as {} ({} bytes): {}'.format(__file__, data, dtype, len(data.tobytes()), data.tobytes()))
