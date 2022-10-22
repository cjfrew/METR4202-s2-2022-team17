#!/usr/bin/env python3

import numpy as np
import math

# This script takes into account of the block positions and whether they are positioned such that
# there is obstruction (i.e. the robot cannot pick up either blocks)

# x,y positions of both blocks
xb1 = ...
yb1 = ...
xb2 = ...
yb1 = ...
# Error Term
e = 2
def obstructions():
    # Two blocks side by side
    if (np.abs(np.sqrt((xb1-xb2)^2 + (yb1-yb2)^2)) < 50):
        if (np.abs(xb1-xb2) < np.abs(yb1-yb2)):
            obstruction = false
        else:
            obstruction = True

    if (yb1 == yb2) or (np.abs(xb1-xb2) > np.abs(yb1-yb2)):
        # If both blocks have the same/similar y-position, then the robot cannot pick up either blocks
        # or if the x distance between the two blocks are larger than the y distance, then there is obstruction 
        obstruction = true
    else:
        obstruction = false

    