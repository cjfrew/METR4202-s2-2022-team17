import numpy as np

def sqhtoscrew(s,q,h):
    # UNTITLED3 Summary of this function goes here
    # Detailed explanation goes here
    w = np.copy(s)
    v = np.cross(q, w) + h*w
    S = np.concatenate((w, v))
    return S

