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
            position=[0, -1.074836196242842, 0.26103164765897596, 0.03582787320809474]))
        shut_gripper_client(True)
        rp.sleep(2)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=[0, -2.5130693921677882, 0.49647195731217, 0.015354802803469174]))
        shut_gripper_client(False)
        rp.sleep(2)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=[0, -1.2335024918786903, -0.3633969996821038, 0.010236535202312783]))
        rp.sleep(2)

        self.joints_pub.publish(JointState(name=JOINT_NAMES, 
            position=HOME_ANGLES))
        rp.sleep(2)
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
        rp.sleep(5)
        return 'at_home'

class GoCube(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_cube'])
        self.cubes = dict()
        self.camera_sub = rp.Subscriber('/fiducial_transforms', FiducialTransformArray, self.update_cubes)
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def update_cubes(self, data: FiducialTransformArray):
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            rx,ry = cubPos_cam2rob(cx,cy)
            self.cubes[cubeID] = (rx,ry)

    def execute(self, ud):
        rp.loginfo('GoCube')

        while len(self.cubes) < 1:
            rp.sleep(0.1)
        
        cubeID = best_of(self.cubes)
        x, y = self.cubes[cubeID]
        msg = JointState(name=JOINT_NAMES, position=IKrob(x, y, 0))
        self.joints_pub.publish(msg)
        rp.sleep(3)

        shut_gripper_client(True)

        self.cubes.clear()
        print("WENT CUBE")
        return 'at_cube'

class GoColor(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_color'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoColor')
        msg = JointState(name=JOINT_NAMES, position=COLOR_ANGLES)
        self.joints_pub.publish(msg)
        rp.sleep(3)
        return 'at_color'

class GetColor(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['yellow', 'green', 'red', 'blue', 'fail'])
        self.color_sub = rp.Subscriber('/detected_color', String, self.update_color)

    def update_color(self, string: String):
        rp.sleep(3)
        if string.data == 'YELLOW':
            return 'yellow'
        elif string.data == 'GREEN':
            return 'green'
        elif string.data == 'RED':
            return 'red'
        elif string.data == 'BLUE':
            return 'blue' 
        else:
            return 'fail'

class GoBlue(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_blue'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoBlue')
        x, y, z = ZONE_BLUE
        msg = JointState(name=JOINT_NAMES, position=IKrob(x, y, z))
        self.joints_pub.publish(msg)
        rp.sleep(3)
        shut_gripper_client(False)
        return 'at_blue'

class GoRed(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_red'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoRed')
        x, y, z = ZONE_RED
        msg = JointState(name=JOINT_NAMES, position=IKrob(x, y, z))
        self.joints_pub.publish(msg)
        rp.sleep(3)
        shut_gripper_client(False)
        return 'at_red'

class GoGreen(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_green'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoGreen')
        x, y, z = ZONE_GREEN
        msg = JointState(name=JOINT_NAMES, position=IKrob(x, y, z))
        self.joints_pub.publish(msg)
        rp.sleep(3)
        shut_gripper_client(False)
        return 'at_green'

class GoYellow(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_yellow'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)

    def execute(self, ud):
        rp.loginfo('GoYellow')
        x, y, z = ZONE_YELLOW
        msg = JointState(name=JOINT_NAMES, position=IKrob(x, y, z))
        self.joints_pub.publish(msg)
        rp.sleep(3)
        shut_gripper_client(False)
        return 'at_yellow'

def main():
    rp.init_node('Task1')
    t1 = sm.StateMachine(outcomes=['success', 'failure'])
    
    with t1:
        t1.add('GoHome', GoHome(), transitions={'at_home':'GoCube'})
        t1.add('GoCube', GoCube(), transitions={'at_cube':'GoColor'})
        t1.add('GoColor', GoColor(), transitions={'at_color':'GoRed'})
        t1.add('GetColor', GetColor(), transitions={'red':'GoRed', 'blue':'GoBlue', 'green':'GoGreen', 'yellow':'GoYellow', 'fail':'GoHome'})
        t1.add('GoRed', GoRed(), transitions={'at_red':'Celebrate'})
        t1.add('GoBlue', GoBlue(), transitions={'at_blue':'Celebrate'})
        t1.add('GoGreen', GoGreen(), transitions={'at_green':'Celebrate'})
        t1.add('GoYellow', GoYellow(), transitions={'at_yellow':'Celebrate'})
        t1.add('Celebrate', Celebrate(), transitions={'celebrated':'GoHome'})

    outcome = t1.execute()
    rp.spin()

if __name__ == '__main__':
    main()
        