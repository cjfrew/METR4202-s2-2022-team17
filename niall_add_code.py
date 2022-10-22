import numpy as np
import modern_robotics as mr

class GetColor(sm.State):
    def __init__(self):
        self.same_color_tally = 0
        self.last_seen_color = 'NONE'
        self.block_color = 'NONE' # NONE RED BLUE GREEN YELLOW

    def update_color(self, msg):
        if self.last_seen_color == msg.data:
            self.same_color_tally += 1
        else:
            self.same_color_tally = 0
        
        if self.same_color_tally >= 10:
            self.block_color = msg.data
        else:
            self.block_color = 'NONE'

    def excecute(self, ud):
        while self.block_color == 'NONE':
            rp.sleep(0.2)

        color_output_map = {'RED':'red',
                            'BLUE': 'blue',
                            'GREEN':'green',
                            'YELLOW':'yellow'}
        return color_output_map[self.block_color]

get_color_angles = [0, 0.5, 0, 0] # 30 degree EE
get_color_angles = [0, 0.715, 0, -0.715] # 41.5 degree EE

L1 = 55     #54
L2 = 117    #116
L3 = 95     #94
L4 = 95     #96
L5 = 15     #15

CAM_Y = 117

while not reached_point(x, y, z):
    rp.sleep(0.1)

def reached_point(current_angles, goal_position):
    current_position = mr.FKinSpace(current_angles)

    for curr_dim, goal_dim in current_position, goal_position:
        
        if np.abs(curr_dim) < np.abs(goal_dim) - 5 \
        or np.abs(curr_dim) > np.abs(goal_dim) + 5:
            return False
    
    return True