#!/usr/bin/env python3
### Standard imports ###
import rospy as rp
import smach as sm
import numpy as np
import modern_robotics as mr

### Self developed libraries ###

from constants import *
from helpers import *

### ROS packages ###

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from fiducial_msgs.msg import FiducialTransformArray

class Celebrate(sm.State):
    """ A class to represent a "celebration" motion with the arm... Code breaks without

    Args:
        sm (State): A representation of the state machine 
    """
    def __init__(self):
        """ initialises the state machine and the joints 
        """
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
    """A class to represent the "go home" state for the robotic arm

    Args:
        sm (state): A state in a stage machine
    """
    def __init__(self):
        sm.State.__init__(self, outcomes=['at_home'])
        self.actual_joints = list()
        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)

    def get_actual_joints(self, joints: JointState):
        """Function to get the current position of the joints

        Args:
            joints (JointState): the actual state of each joint 
        """
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
        """Function to get the current position of the joints

        Args:
            joints (JointState): the actual state of each joint 
        """
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def update_cubes(self, data: FiducialTransformArray): 
        """Call back function for camera subscriber 

        Args:
            data (FiducialTransformArray): Holds transform data of the cubes (fiducial markers) from Camera
        """
        self.cubes.clear()
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            rx,ry = cubPos_cam2rob(cx,cy)
            self.cubes[cubeID] = (rx,ry)

    def check_if_moving(self):
        """Checks whether the conveyer is active (moving)

        Returns:
            Boolean: If there are multiple cubes that are classified as "still" 
        """
        old_cubes = dict()
        new_cubes = dict()
        still_cubes = dict()
        i = 1
        while i <= 4:
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
            if max_diff < 0.5 and len(old_cubes)>0 and len(new_cubes)>0:
                still_cubes = new_cubes.copy()

            old_cubes = new_cubes.copy()
            new_cubes.clear()

            rp.sleep(0.3)
            i=i+1

        if len(still_cubes) < 1:
            return True
        else:
            return False

    def catch_while_moving(self):
        """ Determines the radius and rotation direction of each cube 

        Returns:
            pos_rot (Bool): 
                True if rotating cw
                False if rotating ccw 
        """
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
                pos_rot = True
        
            elif x_diff < 2:
                pos_rot = False
        
            else:
                continue

        return pos_rot

    def execute(self, ud):
        rp.loginfo('WaitCube')

        moving = self.check_if_moving()
        print(moving)
        
        if moving and len(self.cubes) > 0:
        
            pos_rot = self.catch_while_moving()
            cubeID = furthest_of(self.cubes)
            x_pos = self.cubes[cubeID][0]
            y_pos = np.sqrt(190**2-x_pos**2) 
        
            if pos_rot:
                x, y = x_pos, y_pos
            else:
                x, y = -x_pos, y_pos

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
        """Function to get the current position of the joints

        Args:
            joints (JointState): the actual state of each joint 
        """
        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def update_cubes(self, data: FiducialTransformArray):
        """Call back function for camera subscriber 

        Args:
            data (FiducialTransformArray): Holds transform data of the cubes (fiducial markers) from Camera
        """
        self.cubes.clear()
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            rx,ry = cubPos_cam2rob(cx,cy)
            self.cubes[cubeID] = (rx,ry)

    def wait_while_moving(self):
        """ Itterates through all known cubes on platform and using the distance formula calculates which are stationary

        Returns:
            still_cubes (Dict): A dictionary containing all the cubes that are not moving 
                according to the distance formula 
        """
        old_cubes = dict()
        new_cubes = dict()
        still_cubes = dict()

        while len(still_cubes) < 1:
            # find max distance between a cubes old and current position
            # Uses distance formula d = sqrt(dx^2 + dy^2)
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
                x, y = self.cubes[cubeID]

            # make the loop a do while loop
            if np.sqrt(x**2 + y**2) > MAX_RANGE:
                rp.sleep(0.1)
                continue
            else:
                break
        
        # Checks if distance is greater and our distance (from base)
        # If true, reimplement IK algorithms to find new location
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
        """Function to get the actual position of the joints

        Args:
            joints (JointState): the actual state of each joint 
        """

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
    """ A class denoting where the drop / baggage zones are for the blocks to be placed

    Args:
        sm (State): A representation of state from a state machine
    """
    def __init__(self):
        sm.State.__init__(self, outcomes=['dropped'])
        self.same_color_tally = 0
        self.last_seen_color = 'NONE'
        self.block_color = 'NONE' # [NONE RED BLUE GREEN YELLOW]

        self.joints_pub = rp.Publisher('/desired_joint_states', JointState, queue_size=1)
        self.color_sub = rp.Subscriber('/detected_color', String, self.update_color)
        self.actual_joints_sub = rp.Subscriber('/joint_states', JointState, self.get_actual_joints)

    def get_actual_joints(self, joints: JointState):
        """Function to get the current position of the joints

        Args:
            joints (JointState): the actual state of each joint 
        """

        try:
            theta4, theta3, theta2, theta1 = joints.position
            self.actual_joints = [theta1,theta2,theta3,theta4]
        except Exception as e:
            pass

    def update_color(self, msg: String):
        """ Checks the colour of the block s.t. the block can be positioned in the correct block

        Args:
            msg (String): the subscribed data of the colour detected through the camera
        """
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
            pre_throw = [0, np.pi/4, +np.pi/2, -np.pi/2]
            release_throw = [0, np.pi/4, 0, 0]
            post_throw = [0, np.pi/4, 0, 0]

            msg = JointState(name=JOINT_NAMES, position=pre_throw)
            self.joints_pub.publish(msg)
            while not at_joints(pre_throw, self.actual_joints):
                rp.sleep(0.1)

            msg = JointState(name=JOINT_NAMES, position=post_throw)
            self.joints_pub.publish(msg)

            while release_throw[2] < self.actual_joints[2]:
                rp.sleep(0.01)
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