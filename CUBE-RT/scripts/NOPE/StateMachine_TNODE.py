#!/usr/bin/env python3

import rospy as rp
import numpy as np
import modern_robotics as mr

from constants import *
from helpers import *

from std_srvs.srv import SetBool

from std_msgs.msg import Header, String
from fiducial_msgs.msg import FiducialTransformArray
from sensor_msgs.msg import JointState

class Node(object):
    def __init__(self):
        pass

    def shut_gripper_client(self, grip: bool):
        rp.wait_for_service('shut_gripper')
        try:
            shut_gripper = rp.ServiceProxy('shut_gripper', SetBool)
            response = shut_gripper(grip)
            return response
        except Exception as e:
            print(f'{e}')

if __name__ == "__main__":
    while True:    
        node = Node()
        node.shut_gripper_client(True)
        rp.sleep(1)
        node.shut_gripper_client(False)
        rp.sleep(2)


    #rp.init_node('StateMachine')
    #rp.spin()

def corn2cent(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)
        print(yaw_z)

        L=20 #length of cube
        R = np.array([[np.cos(yaw_z), -np.sin(yaw_z), 0],[np.sin(yaw_z), np.cos(yaw_z), 0],[0, 0, 1,]]) 
        
        Pb=np.array([[17],[17],[0]])
        Ps=np.dot(R,Pb) #position of center of cube referenced to its corner
        #Cube position referenced to camera
        xcc=100 #x position get from camera
        ycc=100 #y position get from camera
        #camera position referenced to base
        xbc=100 #x position get from camera
        ybc=100 #y position get from camera
        
        #get center of cube referenced to base

        xbct=int(Ps[0]) #+xcc+xbc
        ybct=int(Ps[1]) #+ycc+ybc
     
        return xbct, ybct # in radians

"""
class StateMachine(object):
    def __init__(self):
        self.sndR = sender()
        self.rcvR = reciever()

class sender(object):
    def __init__(self):
        self.angles = list()
        self.angles_pub = rp.Publisher(
            'desired_joint_states', # Topic name
            JointState,             # Message type
            queue_size=10           # Topic size (optional)
        )

    def set_joint_states(self, angles):
        joint_state_angles = JointState(
            header = Header(stamp = rp.Time.now()),
            name = ['joint_1', 'joint_2', 'joint_3', 'joint_4'])

        joint_state_angles.position = angles
        self.angles_pub.publish(joint_state_angles)

    def shut_gripper_client(self, grip: bool):
        try:
            shut_gripper = rp.ServiceProxy('shut_gripper', SetBool)
            response = shut_gripper(grip)
            return response
        except Exception as e:
            print(f'{e}')

class reciever(object):
    def __init__(self):
        self.color = "NoneYet"
        self.detected_color_sub = rp.Subscriber(
            "/detected_color",      # Topic name
            String,                 # Message type
            self.report_color,      # Callback function (required)
            queue_size=1
        )

        self.cubes = dict()
        self.fiducial_trasnsforms_sub = rp.Subscriber(
            "/fiducial_transforms", # Topic name
            FiducialTransformArray, # Message type
            self.update_cubes,      # Callback function (required)
            queue_size=1
        )
        
    def report_color(self, data):
        pass

    def update_cubes(self, data: FiducialTransformArray) -> FiducialTransformArray:
        for transform in data.transforms:
            cubeID = transform.fiducial_id
            
            cx = transform.transform.translation.x * 1000
            cy = transform.transform.translation.y * 1000
            cz = transform.transform.translation.z * 1000

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            c2cx, c2cy = corn2cent(qx,qy,qz,qw)
            x, y, z = cx+c2cx, cy+c2cy, cz

            rx, ry = cubPos_cam2rob(cx,cy)
            rz = CUBE_SIZE/2

            self.cubes[cubeID] = (rx,ry,rz)

# Example of a state machine class
class StateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
        self.currentState.run()
    
    def runAll(self, inputs):
        # runAll method takes a list of input objects. This method not only moves 
        # to the next state, but it also calls run() for each state object

        for i in inputs:
            print(i)
            self.currentState = self.currentState.next(i)
            self.currentState.run()


def listener():
    global pub

    # Create publisher (The angles requred to get to the position)
    pub = rp.Publisher(
        'desired_pose', # Topic name
        Pose, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber (The position we want the endeffector at wrt robot first joint)
    sub = rp.Subscriber(
        "/fiducial_transforms", # Topic name
        FiducialTransformArray, # Message type
        fiducial_callback, # Callback function (required)
        queue_size=1
    )

    # Initialise node (Give the node a name)
    rp.init_node('PositionCam')

    # Loop the function so it runs continually
    rp.spin()
"""