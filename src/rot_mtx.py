import numpy as np
import skw_mtx as sk

def rot_mtx(w,theta):
    # Rotation matrix: Rodriguez Formula
    # Exponential e^[w]*theta
    R = np.identity(3)+np.sin(theta)*sk.skw_mtx(w)+(1-np.cos(theta))*np.linalg.matrix_power(sk.skw_mtx(w), 2)
    return R