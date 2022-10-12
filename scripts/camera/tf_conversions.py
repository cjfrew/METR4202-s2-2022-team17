#!/usr/bin/python

import rospy
import numpy as np
import tf
from tf_conversions import transformations as tfct
from squaternion import quaternion
import modern_robotics as mr
from geometry_msgs.msg import Transform5tamped, Quaternion, Vector3, Pose

# The goal of this script is to convert the quaternion and positional data from the camera to the ArUco Tag 
# into a transformation matrix and then convert that transformation matrix from the camera to the base of 
# the robot

# Transformation Fucntion
def tf_conversions(tfmessage: TFMessage) -> Pose:

# Extracting translation from the camera

tx = tfmessage.transforms.transform.translation.x
ty = tfmessage.transforms.transform.translation.y
tz = tfmessage.transforms.transform.translation.z

# Extracting quaternion from the camera

qx = tfmessage.transforms.transform.rotation.x
qy = tfmessage.transforms.transform.rotation.y
qz = tfmessage.transforms.transform.rotation.z
qw = tfmessage.transforms.transform.rotation.w

# Convert from quaternions to Euler angles
# Need to use the tf library instead of Quaternion

# This is the Quaternion library 
q = Quaternion(quat_x,quat_y,quat_z,quat_w)
    e = q.to_euler(degrees=False)
    # Euler angles in radians
    roll = e[0] # Rotate about x
    pitch = e[1] # Rotate about y
    yaw = e[2] # Rotate about z

def main():
    global pub
    # Creates Publisher for the transform from the camera to the base of the robot
    pub = rospy.Publisher(
        'position_orientation_of_ArUco_Tag', # Topic Name
        Pose, # Message Type
        queue_size=10, # Topic size
    )

    # Creates Subscriber ()
    sub = rospy.Subscriber(
        'tf', # Topic Name
        TFMessage, # Message Type
        transform # Callback Function
    )

if __name__ == '__main__':
    main()

