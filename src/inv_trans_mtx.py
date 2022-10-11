import numpy as np


def inv_trans_mtx(T):
    # UNTITLED2 Summary of this function goes here
    # Detailed explanation goes here
    [a, b] = np.shape(T)
    print(a)
    R = T[0:a-1,0:a-1]
    p = T[0:a-1,a-1]

    p = p.reshape(-1,1)

    a1 = np.concatenate((np.transpose(R), -np.dot(np.transpose(R),p)),axis=1)
    a2 = np.append(np.zeros((1, a-1)), 1)
    inv_trans_mtx = np.concatenate((a1, [a2]), axis=0)
    return inv_trans_mtx
