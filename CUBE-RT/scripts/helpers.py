#!/usr/bin/env python3

import rospy as rp
import numpy as np
import modern_robotics as mr

from std_srvs.srv import SetBool

from constants import *

def shut_gripper_client(grip: bool):
    rp.wait_for_service('shut_gripper')
    try:
        shut_gripper = rp.ServiceProxy('shut_gripper', SetBool)
        response = shut_gripper(grip)
        return response
    except Exception as e:
        print(f'{e}')

def best_of(cubes):
    min_dist = 999
    for cubeID in cubes:
        dist = distTo(cubes[cubeID])
        if dist < min_dist:
            min_dist = dist
            best_cubeID = cubeID
    return best_cubeID

def distTo(position):
    x, y = position[0], position[1]
    return np.sqrt(x**2 + y**2)

def cam2rob_trans(x, y):
    return -x, y+190

def cubPos_cam2rob(x, y):
    # cube co's relative to center of camera stand board 'Distance Center'
    x_dc = 0.9795*x+1.522
    y_dc = 0.9864*y+29.942

    # cube co's in space frame
    x_s, y_s = cam2rob_trans(x_dc, y_dc)

    return x_s, y_s

def IKrob(x_pos,y_pos,z_pos):
    # Inverse kinematics function
    # Determine x,y,z
    x = x_pos
    y = y_pos
    z = z_pos

    if y==0:
        y=1*10**(-10)

    mx_length=L2+L3+L4 # Max Length of arm
    #Quadrants
    if np.sqrt(x**2+y**2)>200:
        phi=np.pi/2
    else: 
        phi=np.pi

    # Flip bhind if need
    dir=1
    if y<0:
        dir=-1

    # XYZ limits
    if np.sqrt(x**2+y**2)>mx_length:
        ang = np.array([0,0,0,0])
        
    else:
        theta1 = -np.arctan2(x,y)

        Pz = z - L1 + L5*np.sin(phi)
        Py = np.sqrt(x**2+y**2)-L5*np.cos(phi)
        Wz = Pz-L4*np.cos(phi)
        Wy = Py-L4*np.sin(phi)
        c2 = (Wz**2+Wy**2-L2**2-L3**2)/(2*L2*L3)
        theta3 = np.arccos(c2)
        theta2 = np.arctan2(Wy,Wz)-np.arctan2((L3*np.sin(theta3)),(L2+L3*np.cos(theta3)))
        if theta2<0:
            theta2 = theta2+np.pi
        theta4 = phi-theta2-theta3
        ang = np.array([theta1,-theta2,-theta3,theta4])
        
        if theta2>2 or theta2<-2:
            ang = np.array([0,0,0,0])
        if theta3>2.2 or theta3<-2.2:
            ang = np.array([0,0,0,0])
        if theta4>2.2 or theta4<-2.2:
            ang = np.array([0,0,0,0])

           
    b=0
    for a in ang:
        while a>np.pi:
            a=a-2*np.pi
                
        while a<=-np.pi:
            a=a+2*np.pi
        
        ang[b]=a
        b=b+1
    
    return ang