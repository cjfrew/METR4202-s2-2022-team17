#!/usr/bin/python

import rospy
import numpy as np


from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray



# The goal of this script is to convert the quaternion and positional data from the camera to the ArUco Tag 
# into a transformation matrix and then convert that transformation matrix from the camera to the base of 
# the robot

def fiducial_callback(data):
    print('New Blocks:')
    for transforms in data.transforms:
        
        #rospy.loginfo(rospy.get_caller_id() + "\nID = %s\nI heard translation:\n%s\nI heard rotation:\n%s\n", 
        #data.transforms[0].fiducial_id, data.transforms[0].transform.translation, data.transforms[0].transform.rotation)
        rospy.loginfo("\nID = %s\nI heard translation:\n%s\nI heard rotation:\n%s\n", 
        transforms.fiducial_id, transforms.transform.translation, transforms.transform.rotation)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_callback, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    listener()
