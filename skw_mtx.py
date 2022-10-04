import numpy as np

def skw_mtx(w):
    # Get skew_symetric matrix
    w1 = w[0]
    w2 = w[1]
    w3 = w[2]
    skw_m = np.array([[0, -w3, w2], [w3, 0, -w1], [-w2, w1, 0]])
    return skw_m
