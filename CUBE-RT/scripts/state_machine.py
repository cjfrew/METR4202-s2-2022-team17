#!/usr/bin/env python3

import rospy as rp
import smach as sm
import numpy as np
import modern_robotics as mr

from constants import *
from helpers import *

from std_msgs.msg import String
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
        self.actual_joints = list()
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)

    def get_actual_joints(self, joints: JointState):
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def execute(self, ud):
        rp.loginfo('GoHome')

        shut_gripper_client(False)
        msg = JointState(name=JOINT_NAMES, position=HOME_ANGLES)

        while not at_joints(HOME_ANGLES, self.actual_joints):
            self.joints_pub.publish(msg)
            rp.sleep(0.1)

        return 'at_home'

class WaitCube(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['got_cube'])
        # create dict of current cube
        self.cubes = dict()
        self.rad_cubes = dict()

        self.camera_sub = rp.Subscriber('/fiducial_transforms', FiducialTransformArray, self.update_cubes)
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)
    
    def get_actual_joints(self, joints: JointState):
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def update_cubes(self, data: FiducialTransformArray):
        self.cubes.clear()
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            rx,ry = cubPos_cam2rob(cx,cy)
            self.cubes[cubeID] = (rx,ry)

    def catch_while_moving(self):
        pos_rot = True
        old_cubes = self.cubes.copy()
        rp.sleep(0.5)
        new_cubes = self.cubes.copy()
        self.rad_cubes = dict()

        for cubeID in new_cubes:
            x, y = new_cubes[cubeID]
            x_cent, y_cent = rob2cent(x,y)
            self.rad_cubes[cubeID] = np.sqrt(x_cent**2 + y_cent**2)
            if cubeID in old_cubes:
                x_diff = np.abs(new_cubes[cubeID][0] - old_cubes[cubeID][0])
            if x_diff > 2:
                pos_rot = False
            elif x_diff < 2:
                pos_rot = True
            else:
                continue

        return pos_rot

    def execute(self, ud):
        rp.loginfo('WaitCube')

        while len(self.cubes) < 1:
            rp.sleep(0.2)
    
        pos_rot = self.catch_while_moving()
        cubeID = furthest_of(self.cubes)
        x_pos = self.rad_cubes[cubeID]
        y_pos = np.sqrt(190**2 - x_pos**2) 
        if pos_rot:
            x, y = x_pos, y_pos
        else:
            x, y = -x_pos, y_pos
        
        print(x, y)
        wait_angles = IKrob2(x, y, Z_OFFSET, np.pi/2)
        print(wait_angles)
        msg = JointState(name=JOINT_NAMES, position=wait_angles)
        self.joints_pub.publish(msg)
        while not at_joints(wait_angles, self.actual_joints):
            rp.sleep(0.1)  

        while self.cubes[cubeID][1] > 190:
            rp.sleep(0.1)
        
        rp.sleep(3)
        shut_gripper_client(True)
        return 'got_cube'

class GoCube(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_cube'])
        # create dict of current cube
        self.cubes = dict()
        self.rad_cubes = dict()

        self.camera_sub = rp.Subscriber('/fiducial_transforms', FiducialTransformArray, self.update_cubes)
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)

    def get_actual_joints(self, joints: JointState):
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def update_cubes(self, data: FiducialTransformArray):
        self.cubes.clear()
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            rx,ry = cubPos_cam2rob(cx,cy)
            self.cubes[cubeID] = (rx,ry)

    def wait_while_moving(self):
        old_cubes = dict()
        new_cubes = dict()
        still_cubes = dict()

        while len(still_cubes) < 1:
            # find max distance between a cubes old and current position
            max_diff = 0
            new_cubes = self.cubes.copy()
            for cubeID in new_cubes:
                if cubeID in old_cubes.keys():
                    x_diff = np.abs(new_cubes[cubeID][0] - old_cubes[cubeID][0])
                    y_diff = np.abs(new_cubes[cubeID][1] - old_cubes[cubeID][1])
                    diff = np.sqrt(x_diff**2 + y_diff**2)
                    if diff > max_diff:
                        max_diff = diff
            
            # if the cubes positions are changing, they are moving and there is no still cubes
            if max_diff < 0.25 and len(old_cubes)>0 and len(new_cubes)>0:
                still_cubes = new_cubes.copy()

            old_cubes = new_cubes.copy()
            new_cubes.clear()

            rp.sleep(0.5)

        if len(still_cubes) < 1:
            return None
        else:
            return still_cubes

    def execute(self, ud):
        rp.loginfo('GoCube')

        while True:
            if TASK_NUMBER == 3:
                cubeID = 'None'
                while cubeID == 'None':
                    cubes = self.cubes.copy()
                    cubeID = best_of2(cubes)
                    if cubeID == 'None' or obstructions(cubeID, self.cubes):
                        rp.sleep(0.1)
                        continue
                    else:
                        x, y = cubes[cubeID]
            else:
                still_cubes = self.wait_while_moving()
                cubeID = best_of1(still_cubes)
                try:
                    x, y = self.cubes[cubeID]
                except Exception as e:
                    continue

            # make the loop a do while loop
            if np.sqrt(x**2 + y**2) > MAX_RANGE:
                rp.sleep(0.1)
                continue
            else:
                break

        if np.sqrt(x**2 + y**2) > 270:
            final_joint_angles = IKrob2(x ,y, Z_OFFSET, np.pi/2 + np.pi/6)
        else: 
            final_joint_angles = IKrob(x, y, Z_OFFSET)

        pre_A_joint_angles = final_joint_angles.copy()
        pre_A_joint_angles[1] = 0.0
        
        msg = JointState(name=JOINT_NAMES, position=pre_A_joint_angles)
        self.joints_pub.publish(msg)
        while not at_joints(pre_A_joint_angles, self.actual_joints):
            rp.sleep(0.1)         

        if np.sqrt(x**2 + y**2) > CHANGE_GRIPPER_ANGLE+30 and np.sqrt(x**2 + y**2) < 270:
            pre_grab = IKrob2(x-30*np.sin(final_joint_angles[0]), y-30*np.cos(final_joint_angles[0]), Z_OFFSET,np.pi/2)
            pre_grab[0] = final_joint_angles[0]
            msg = JointState(name=JOINT_NAMES, position=pre_grab)
            self.joints_pub.publish(msg)
            while not at_joints(pre_grab, self.actual_joints):
                rp.sleep(0.1)

        msg = JointState(name=JOINT_NAMES, position=final_joint_angles)
        self.joints_pub.publish(msg)
        while not at_joints(final_joint_angles, self.actual_joints):
            rp.sleep(0.1)

        rp.sleep(0.5)
        shut_gripper_client(True)
        rp.sleep(0.5)

        self.cubes.clear()

        return 'at_cube'

class GoColor(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_color'])
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)

    def get_actual_joints(self, joints: JointState):
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def execute(self, ud):
        rp.loginfo('GoColor')
        prelim_joint_angles = self.actual_joints.copy()
        prelim_joint_angles[1] = 0

        msg = JointState(name=JOINT_NAMES, position=prelim_joint_angles)
        self.joints_pub.publish(msg)
        
        while not at_joints(prelim_joint_angles, self.actual_joints):
            rp.sleep(0.1)

        msg = JointState(name=JOINT_NAMES, position=COLOR_ANGLES)
        self.joints_pub.publish(msg)
        
        while not at_joints(COLOR_ANGLES, self.actual_joints):
            rp.sleep(0.1)

        return 'at_color'

class DropZone(sm.State):
    def __init__(self):
        sm.State.__init__(self, outcomes=['dropped'])
        self.same_color_tally = 0
        self.last_seen_color = 'NONE'
        self.block_color = 'NONE' # NONE RED BLUE GREEN YELLOW

        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.color_sub = rp.Subscriber('/detected_color', String, self.update_color)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)

    def get_actual_joints(self, joints: JointState):
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def update_color(self, msg: String):
        self.last_seen_color = msg.data
        if self.last_seen_color == msg.data:
            self.same_color_tally += 1
        else:
            self.same_color_tally = 0
            
    def execute(self, ud):
        rp.loginfo('GetColor')
        self.same_color_tally = 0

        self.block_color = 'NONE'
        while self.block_color == 'NONE':
            if self.same_color_tally >= 10:
                self.block_color = self.last_seen_color
                print(f'CUBE-RT thinks his block is {self.block_color}')
            else:
                self.block_color = 'NONE'
            rp.sleep(0.1)

        if TASK_NUMBER == 5:
            pre_throw = [THROW_ANGLE[self.block_color], -np.pi/4, -np.pi/4, np.pi/4]
            release_throw = [0, 0, 0, 0]
            post_throw = [THROW_ANGLE[self.block_color], np.pi/4, np.pi/4, -np.pi/4]

            msg = JointState(name=JOINT_NAMES, position=pre_throw)
            self.joints_pub.publish(msg)
            while not at_joints(pre_throw, self.actual_joints):
                rp.sleep(0.1)

            msg = JointState(name=JOINT_NAMES, position=post_throw)
            self.joints_pub.publish(msg)

            rp.sleep(0.3)
            shut_gripper_client(False)

            while not at_joints(post_throw, self.actual_joints):
                rp.sleep(0.01)

        else:
            x, y, z = GO_ZONE[self.block_color]
            joint_angles = IKrob(x, y, z)*-1

            msg = JointState(name=JOINT_NAMES, position=joint_angles)
            self.joints_pub.publish(msg)

            while not at_joints(joint_angles, self.actual_joints):
                rp.sleep(0.1)

        rp.sleep(0.5)
        shut_gripper_client(False)
        rp.sleep(0.5)
        return 'dropped'

def main():
    #rp.sleep(10)
    task_number = TASK_NUMBER
    
    if task_number == 1:
        print('Asking CUBE-RT to run 2.1.1 Task Difficulty Level 1 ...')
        rp.init_node('TASK1')  
        t1 = sm.StateMachine(outcomes=[])
        with t1:
            t1.add('GoHome', GoHome(), transitions={'at_home':'GoCube'})
            t1.add('GoCube', GoCube(), transitions={'at_cube':'GoColor'})
            t1.add('GoColor', GoColor(), transitions={'at_color':'DropZone'})
            t1.add('DropZone', DropZone(), transitions={'dropped':'GoHome'})
        outcome = t1.execute()
    
    elif task_number == 2:
        print('Asking CUBE-RT to run 2.1.2 Task Difficulty Level 2 ...')
        rp.init_node('TASK2')  
        t2 = sm.StateMachine(outcomes=[])
        with t2:
            t2.add('GoHome', GoHome(), transitions={'at_home':'GoCube'})
            t2.add('GoCube', GoCube(), transitions={'at_cube':'GoColor'})
            t2.add('GoColor', GoColor(), transitions={'at_color':'DropZone'})
            t2.add('DropZone', DropZone(), transitions={'dropped':'GoHome'})
        outcome = t2.execute()
    
    elif task_number == 3:
        print('Asking CUBE-RT to run 2.1.3 Task Difficulty Level 3a ...')
        rp.init_node('TASK3')  
        t3 = sm.StateMachine(outcomes=[])
        with t3:
            t3.add('GoHome', GoHome(), transitions={'at_home':'GoCube'})
            t3.add('GoCube', GoCube(), transitions={'at_cube':'GoColor'})
            t3.add('GoColor', GoColor(), transitions={'at_color':'DropZone'})
            t3.add('DropZone', DropZone(), transitions={'dropped':'GoHome'})
        outcome = t3.execute()

    elif task_number == 4:
        print('Asking CUBE-RT to run 2.1.4 Task Difficulty Level 3b ...')
        rp.init_node('TASK4')  
        t4 = sm.StateMachine(outcomes=[])
        with t4:
            t4.add('GoHome', GoHome(), transitions={'at_home':'WaitCube'})
            t4.add('WaitCube', WaitCube(), transitions={'got_cube':'GoColor'})
            t4.add('GoColor', GoColor(), transitions={'at_color':'DropZone'})
            t4.add('DropZone', DropZone(), transitions={'dropped':'GoHome'})
        outcome = t4.execute()

    elif task_number == 5:
        print('Asking CUBE-RT to run 2.1.5 Fun Task ...')
        rp.init_node('TASK5')  
        t5 = sm.StateMachine(outcomes=[])
        with t5:
            t5.add('GoHome', GoHome(), transitions={'at_home':'GoCube'})
            t5.add('GoCube', GoCube(), transitions={'at_cube':'GoColor'})
            t5.add('GoColor', GoColor(), transitions={'at_color':'DropZone'})
            t5.add('DropZone', DropZone(), transitions={'dropped':'GoHome'})
        outcome = t5.execute()

    rp.spin()

if __name__ == '__main__':
    main()