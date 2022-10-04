import numpy as np
import trans_mtx_exp as tme


def FK_POE_S(Slist,thetalist,M):
    Ts = np.copy(M) # end-effector zero position
    for i in range(len(thetalist)-1, -1, -1):
        Ts = np.dot(tme.trans_mtx_exp(Slist[:, i], thetalist[i]), Ts)
    return Ts

