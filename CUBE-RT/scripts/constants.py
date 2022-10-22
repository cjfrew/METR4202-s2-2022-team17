# Gripper
from pickle import REDUCE


DROP = 2000
GRIP = 1500
CLOSE = 1000

# distance of camera orogin from camera stand board center
CAM_CENTER_OFFSET_X = -27
CAM_CENTER_OFFSET_Y = 44

# Length of 4R robots
L1 = 55
L2 = 117
L3 = 95
L4 = 95
L5 = 15

CUBE_SIZE = 34

GENERAL_ERROR = 0.1

# Home joint angles 1-4
HOME_ANGLES = [0, 0, 0, 0]
# Colour detect joint angles 1-4
COLOR_ANGLES = [0, -0.15354802803469172, -1.0697179286416858, 0.025591338005781956]

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

ZONE_Z = 0
ZONE_RED = (-150, -50, ZONE_Z)
ZONE_BLUE = (-50, -150, ZONE_Z)
ZONE_GREEN = (50, -150, ZONE_Z)
ZONE_YELLOW = (150, -50, ZONE_Z)
