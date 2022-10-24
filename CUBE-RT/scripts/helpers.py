#!/usr/bin/env python3

import rospy as rp
import numpy as np
import modern_robotics as mr

from std_srvs.srv import SetBool

from constants import *

def at_joints(want_joints, have_joints):
    if len(have_joints) < 4:
        return False

    for i in range(4):
        if np.abs(want_joints[i] - have_joints[i]) > 0.05:
            return False    
    return True

def shut_gripper_client(grip: bool):
    rp.wait_for_service('shut_gripper')
    try:
        shut_gripper = rp.ServiceProxy('shut_gripper', SetBool)
        response = shut_gripper(grip)
        return response
    except Exception as e:
        print(f'{e}')

def best_of1(cubes):
    best_cubeID = None
    min_dist = 999

    for cubeID in cubes:
        dist = distTo(cubes[cubeID])

        if dist < min_dist:
            min_dist = dist
            best_cubeID = cubeID

    return best_cubeID

def furthest_of(cubes):

    furthest_cubeID = None
    max_dist = 0

    for cubeID in cubes:
        dist = distTo(cubes[cubeID])

        if dist > max_dist:
            max_dist = dist
            furthest_cubeID = cubeID

    return furthest_cubeID

def best_of2(cubes):

    best_cubeID = 'None'
    min_dist = 999

    for cubeID in cubes:
        obstructed = obstructions(cubeID, cubes)

        if obstructed:
            continue

        dist = distTo(cubes[cubeID])

        if dist < min_dist:
            min_dist = dist
            best_cubeID = cubeID

    return best_cubeID

def obstructions(cubeID1, cubes):

    obstructed = False
    position1 = cubes[cubeID1]
    x1, y1 = position1[0], position1[1]

    for cubeID2 in cubes:
        position2 = cubes[cubeID2]
        x2, y2 = position2[0], position2[1]

        if x1 == x2 and y1 == y2:
            continue

        if (np.sqrt((x1-x2)**2 + (y1-y2)**2) <= BLOCK_SIZE + OBSTRUCTION_ERROR) and (np.abs(x1-x2) > np.abs(y1-y2)):
            obstructed = True
            break

        else:
            obstructed = False
    
    return obstructed

def distTo(position):
    x, y = position[0], position[1]
    return np.sqrt(x**2 + y**2)

def cam2rob_trans(x, y):
    return y, x+190

def cubPos_cam2rob(x, y):
    # cube co's relative to center of camera stand board 'Distance Center'
    x_dc = x+CAM_CENTER_OFFSET_X
    y_dc = y+CAM_CENTER_OFFSET_Y

    # cube co's in space frame
    x_s, y_s = cam2rob_trans(x_dc, y_dc)

    return x_s, y_s

def rob2cent(x,y):
    x_cent, y_cent = y, x-190
    return x_cent, y_cent

def IKrob(x_pos,y_pos,z_pos):
    # Inverse kinematics function
    # Determine x,y,z
    x = x_pos
    y = y_pos
    z = z_pos

    if y==0:
        y=1*10**(-10)

    mx_length1=L2+L3+L4 # Max Length of arm
    mx_length2=L2+L3
    
    if np.sqrt(x**2 + y**2) > CHANGE_GRIPPER_ANGLE:
        phi=np.pi/2 + 0.2 #2.065 #30 degrees down from horizontal
    else: 
        phi=np.pi

    # XYZ limits
    theta1 = -np.arctan2(x,y)
    Pz = z - L1 + L5*np.sin(phi)
    Py = np.sqrt(x**2+y**2)-L5*np.cos(phi)

    if np.sqrt(Pz**2+Py**2)>mx_length1:

        ang = np.array([0,0,0,0])

    else:

        Wz = Pz-L4*np.cos(phi)
        Wy = Py-L4*np.sin(phi)
        c2 = (Wz**2+Wy**2-L2**2-L3**2)/(2*L2*L3)
        
        try:
            theta3 = np.arccos(c2)
        except Exception as e:
            pass

        theta2 = np.arctan2(Wy,Wz)-np.arctan2((L3*np.sin(theta3)),(L2+L3*np.cos(theta3)))
        if theta2 < 0:
            theta2 = theta2+np.pi
        theta4 = phi-theta2-theta3
            
        if theta1 > 2.5:
            theta1 = 2.5
        if theta1 < -2.5:
            theta1 = -2.5
        if theta2 > 2 or theta2 < -2:
            theta2 = 0
        if theta3 > 2.5 or theta3 < -2.5:
            theta3 = 0
        if theta4 > 1.8 or theta4 < -1.8:
            theta4 = 0

        ang = np.array([theta1 + THETA1_OFFSET,-theta2,-theta3,theta4])
           
    b=0
    for a in ang:
        while a>np.pi:
            a=a-2*np.pi
                
        while a<=-np.pi:
            a=a+2*np.pi
        
        ang[b]=a
        b=b+1
    
    return ang

def IKrob2(x_pos,y_pos,z_pos,phi):
    # Inverse kinematics function
    # Determine x,y,z
    x = x_pos
    y = y_pos
    z = z_pos

    if y==0:
        y=1*10**(-10)

    mx_length1=L2+L3+L4 # Max Length of arm
    mx_length2=L2+L3

    # XYZ limits
    theta1 = -np.arctan2(x,y)
    Pz = z - L1 + L5*np.sin(phi)
    Py = np.sqrt(x**2+y**2)-L5*np.cos(phi)
    if np.sqrt(Pz**2+Py**2)>mx_length1:
        ang = np.array([0,0,0,0])
    else:
        Wz = Pz-L4*np.cos(phi)
        Wy = Py-L4*np.sin(phi)
        c2 = (Wz**2+Wy**2-L2**2-L3**2)/(2*L2*L3)
        
        try:
            theta3 = np.arccos(c2)
        except Exception as e:
            pass

        theta2 = np.arctan2(Wy,Wz)-np.arctan2((L3*np.sin(theta3)),(L2+L3*np.cos(theta3)))
        
        if theta2 < 0:
            theta2 = theta2+np.pi
        
        theta4 = phi-theta2-theta3
            
        if theta1 > 2.5:
            theta1 = 2.5
        if theta1 < -2.5:
            theta1 = -2.5
        if theta2 > 2 or theta2 < -2:
            theta2 = 0
        if theta3 > 2.5 or theta3 < -2.5:
            theta3 = 0
        if theta4 > 1.8 or theta4 < -1.8:
            theta4 = 0

        ang = np.array([theta1 + THETA1_OFFSET,-theta2,-theta3,theta4])
           
    b=0
    for a in ang:
        while a>np.pi:
            a=a-2*np.pi
                
        while a<=-np.pi:
            a=a+2*np.pi
        
        ang[b]=a
        b=b+1
    
    return ang