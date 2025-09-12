import numpy as np
import pickle
import socket

verbose = False


def recv_dtype(sock, dtype, nitems = 1):
    if dtype == str:
        data = sock.recv(nitems, socket.MSG_WAITALL)
    else:
        dstr = sock.recv(dtype().nbytes * nitems, socket.MSG_WAITALL)
        if verbose:
            print(' => {}  received ?? as {} ({} / {}  bytes): {}'.format(__file__, dtype, len(dstr), dtype().nbytes * nitems , dstr))
        data = np.fromstring(dstr, dtype=dtype, count=nitems)
    #except ValueError:
    #    import logging
    #    print('timed out waiting for ' + str(dtype))
    #    import pdb
    #    pdb.set_trace()
    if nitems == 1:
        return data[0]


def transmit_dtype(sock, data, dtype = None):
    # TODO: handle vectors..
    if dtype != None:
        data = dtype(data)

    dstr = sock.sendall(data.tobytes())
    if verbose:
        print(' => {}  transmitted {} as {} ({} bytes): {}'.format(__file__, data, dtype, len(data.tobytes()), data.tobytes()))
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
