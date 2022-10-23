#!/usr/bin/env python3

import rospy as rp
import smach as sm
import numpy as np
import modern_robotics as mr

from constants import *
from helpers import *

from std_msgs.msg import Header, String
from sensor_msgs.msg import JointState
from fiducial_msgs.msg import FiducialTransformArray


class Celebrate(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['celebrated'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('Celebrate')

        shut_gripper_client(False)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=[0, -1, 0.3, 0]))
        shut_gripper_client(True)
        rp.sleep(1)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=[0, -2.5, 0.5, 0]))
        shut_gripper_client(False)
        rp.sleep(1)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=[0, -1, -0.5, 0]))
        rp.sleep(1)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=HOME_ANGLES))
        rp.sleep(1)
        return 'at_home'


class GoHome(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_home'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoHome')

        shut_gripper_client(False)

        msg = JointState(name=JOINT_NAMES, position=HOME_ANGLES)
        self.joints_pub.publish(msg)

        rp.sleep(2)
        return 'at_home'

class GoCube(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_cube', 'no_valid_cube'])
        self.cubes = dict()
        self.camera_sub = rp.Subscriber('/fiducial_transforms', FiducialTransformArray, self.update_cubes)
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def update_cubes(self, data: FiducialTransformArray):
        self.cubes.clear()
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            rx,ry = cubPos_cam2rob(cx,cy)
            self.cubes[cubeID] = (rx,ry)

    def execute(self, ud):
        rp.loginfo('GoCube')
        new_cubes = self.cubes.copy()

        cubeID = best_of2(new_cubes)
        if cubeID == 'None':
            return 'no_valid_cube'
        x, y = new_cubes[cubeID]
        joint_angles = IKrob(x, y, 0)
        msg = JointState(name=JOINT_NAMES, position=joint_angles)
        self.joints_pub.publish(msg)
        rp.sleep(2)
        if y > CHANGE_GRIPPER_ANGLE:
            z_rot = joint_angles[0]
            scoop_joint_angles = [z_rot, joint_angles[1]-0.2, joint_angles[2]+0.2, joint_angles[3]+0.15]
            msg = JointState(name=JOINT_NAMES, position=scoop_joint_angles)
            self.joints_pub.publish(msg)
            sleep(0.1)
        shut_gripper_client(True)

        new_cubes.clear()
        return 'at_cube'

class GoColor(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_color'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoColor')
        msg = JointState(name=JOINT_NAMES, position=COLOR_ANGLES)
        self.joints_pub.publish(msg)
        rp.sleep(2)
        return 'at_color'

class DropZone(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['dropped'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.color_sub = rp.Subscriber('/detected_color', String, self.update_color)

        self.same_color_tally = 0
        self.last_seen_color = 'NONE'
        self.block_color = 'NONE' # NONE RED BLUE GREEN YELLOW

    def update_color(self, msg: String):
        if self.last_seen_color == msg.data:
            self.same_color_tally += 1
        else:
            self.same_color_tally = 0
        
        if self.same_color_tally >= 10:
            self.block_color = msg.data
        else:
            self.block_color = 'NONE'

        self.last_seen_color = msg.data

    def execute(self, ud):
        rp.loginfo('GetColor')
        self.same_color_tally = 0

        self.block_color = 'NONE'
        while self.block_color == 'NONE':
            rp.sleep(0.2)

        x, y, z = GO_ZONE[self.block_color]
        joint_angles = IKrob(x, y, z)

        print(f'CUBE-RT thinks his block is {self.block_color}')

        msg = JointState(name=JOINT_NAMES, position=[joint_angles[0], 0, 0, 0])
        self.joints_pub.publish(msg)

        rp.sleep(3)

        msg = JointState(name=JOINT_NAMES, position=joint_angles)
        self.joints_pub.publish(msg)

        rp.sleep(3)

        shut_gripper_client(False)
        return 'dropped'

def main():
    rp.init_node('Task1')
    t1 = sm.StateMachine(outcomes=['success', 'failure'])
    
    with t1:
        t1.add('GoHome', GoHome(), transitions={'at_home':'GoCube'})
        t1.add('GoCube', GoCube(), transitions={'at_cube':'GoColor', 'no_valid_cube':'GoHome'})
        t1.add('GoColor', GoColor(), transitions={'at_color':'DropZone'})
        t1.add('DropZone', DropZone(), transitions={'dropped':'GoHome'})

        t1.add('Celebrate', Celebrate(), transitions={'celebrated':'GoHome'})

    outcome = t1.execute()
    rp.spin()

if __name__ == '__main__':
    main()
        