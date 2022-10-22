#!/usr/bin/python

import rospy as rp
import numpy as np
import math

from constants import *
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Pose

# The goal of this script is to convert the quaternion and positional data from the camera to the ArUco Tag 
# into a transformation matrix and then convert that transformation matrix from the camera to the base of 
# the robot

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
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
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

def cubPos_cam2rob(x, y):
    # cube co's relative to center of camera stand board 'Distance Center'
    x_dc = x+CAM_CENTER_OFFSET_X
    y_dc = y+CAM_CENTER_OFFSET_Y

    # cube co's in space frame
    x_s, y_s = cam2rob_trans(x_dc, y_dc)

    return x_s, y_s

def cam2rob_trans(x, y):
    return -x, y+190

def fiducial_callback(data: FiducialTransformArray) -> FiducialTransformArray:
    cubes = dict()
    msg = FiducialTransformArray()
    print('New Blocks:')
    for transform in data.transforms:
        cubeID = transform.fiducial_id
        cx = transform.transform.translation.x * 1000
        cy = transform.transform.translation.y * 1000
        cz = transform.transform.translation.z * 1000

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        c2cx,c2cy = corn2cent(qx,qy,qz,qw)
        x = cx + c2cx
        y = cy + c2cy
        z = cz

        rx,ry = cubPos_cam2rob(cx,cy)
        rz = 17

        cubes[cubeID] = (rx,ry,rz)

        rp.loginfo(f'Box {cubeID}:\ncent: {x,y,z}\ncorn: {cx,cy,cz}\nrobot: {rx,ry}\n')
        #rp.loginfo("\nID = %s\nI heard translation:\n%s\nI heard rotation:\n%s\n", 
        #transform.fiducial_id, transform.transform.translation, transform.transform.rotation)

        # Create message of type JointState
        #msg = Pose()
        
        msg.transforms.fiducial_id = cubeID
        msg.transform.transform.translation.x = rx
        msg.transform.transform.translation.y = ry
        msg.transform.transform.translation.z = rz

        msg.transform.rotation.x = 0
        msg.transform.rotation.y = 0
        msg.transform.rotation.z = 0
        msg.transform.rotation.w = 0
        
    
    global pub
    pub.publish(msg)
    rp.sleep(2)

def listener():
    global pub

    # Create publisher (The angles requred to get to the position)
    pub = rp.Publisher(
        '/cube_pose', # Topic name
        FiducialTransformArray, # Message type
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

if __name__ == '__main__':
    listener()