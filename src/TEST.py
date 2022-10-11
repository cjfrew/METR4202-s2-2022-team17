
import numpy as np
import modern_robotics as mr
L1 = 1
L2 = 1
L3 = 1
L4 = 1
L5 = 0.2
M=np.array([[1, 0, 0, 0],[0, 1, 0, -L5],[0, 0, 1, L1+L2+L3+L4],[0, 0, 0, 1]])
print(M)
M_inv=mr.TransInv(M)
print(M_inv)