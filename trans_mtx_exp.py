import numpy as np
import skw_mtx as sk
import rot_mtx as rt


def trans_mtx_exp(S,theta):
    # UNTITLED3 Summary of this function goes here
    # Detailed explanation goes here
    S = S.reshape(-1,1)
    theta = theta.reshape(-1,1)
    w = np.copy(S[0:3, 0])
    v = np.copy(S[3:6, 0])
    if np.linalg.norm(w) == 1:
        R = rt.rot_mtx(w,theta)
        p = np.dot(np.identity(3)*theta+(1-np.cos(theta))*sk.skw_mtx(w)+(theta-np.sin(theta))*np.linalg.matrix_power(sk.skw_mtx(w), 2), v)

    if np.array_equal(w, np.array([[0], [0], [0]])) and np.linalg.norm(v) == 1:
        R = np.identity(3)
        p = np.dot(v, theta)

    p = p.reshape(-1,1)
    a1 = np.concatenate((R, p), axis=1)
    a2 = np.append(np.zeros((1,3)),1)
    T = np.concatenate((a1, [a2]),axis=0)
    return T

