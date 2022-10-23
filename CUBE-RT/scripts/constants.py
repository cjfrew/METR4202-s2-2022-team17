# CONSTANTS

TASK_NUMBER = 3

# Set to zero for demo
Z_OFFSET = -15
THETA1_OFFSET = 0
INITIAL_THROW = [-1.6685552379769837, -1.9193503504336467, -1.2846851678902542, 0.056300943612720304]


DROP = 1980 #2000
GRIP = 1370 #1500
CLOSE = 1000#1000

# distance of camera orogin from camera stand board center
CAM_CENTER_OFFSET_X = 16.743
CAM_CENTER_OFFSET_Y = 23.089

# Length of 4R robots
L1 = 55
L2 = 117
L3 = 95
L4 = 95
L5 = 15

MAX_RANGE = 285

# Error distance between two obstructed blocks
OBSTRUCTION_ERROR = 30

# Aversge width of a block
BLOCK_SIZE = 34

# distance to change endeffector orientation
CHANGE_GRIPPER_ANGLE = 200 - L5 - 1

CUBE_SIZE = 34

GENERAL_ERROR = 0.1
ERROR_SPIN = 5 

# Home joint angles 1-4
HOME_ANGLES = [0, 0, 0, 0]
# Colour detect joint angles 1-4
COLOR_ANGLES = [-0.05, -0.98, -0, -0.9]

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

ZONE_Z = 20
ZONE_YELLOW = (150, 50, ZONE_Z)
ZONE_GREEN = (50, 150, ZONE_Z)
ZONE_BLUE = (-50, 150, ZONE_Z)
ZONE_RED = (-150, 50, ZONE_Z)

GO_ZONE = {
    'RED':ZONE_RED,
    'BLUE':ZONE_BLUE,
    'GREEN':ZONE_GREEN,
    'YELLOW':ZONE_YELLOW
    }