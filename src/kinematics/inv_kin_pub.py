#!/usr/bin/env python3

# Inverse Kinematics Libs
import numpy as np
import sqhtoscrew as sq
import FK_POE_S as fk
import inv_trans_mtx as itm
import modern_robotics as mr
from squaternion import Quaternion
import random

# Always need this
import rospy

# Import message types
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


# Inverse kinematics function
def inverse_kinematics(pose: Pose) -> JointState:
    # Determine x,y,z & quat_x,quat_y,quat_z,quat_w from pose
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    quat_x = pose.orientation.x
    quat_y = pose.orientation.y
    quat_z = pose.orientation.z
    quat_w = pose.orientation.w

    q = Quaternion(quat_x,quat_y,quat_z,quat_w)
    # Convert from quaternions to Euler angles
    e = q.to_euler(degrees=False)
    # Euler angles in radians
    roll = e[0] # Rotate about x
    pitch = e[1] # Rotate about y
    yaw = e[2] # Rotate about z

    # Length of 4R robots
    L1 = 55
    L2 = 117
    L3 = 95
    L4 = 115
    L5 = 15
    # Create Zero-position robot
    M = np.array([[1, 0, 0, 0],[0, 1, 0, -L5],[0, 0, 1, L1+L2+L3+L4],[0, 0, 0, 1]]) #Robot in vertical position
    N = 4 # define # joints
    #Newton-Raphson conditions
    Ew = 0.001  # Error rotation
    Ev = 0.1   # error position
    mx_it=10 #define max iterations
    theta = np.array([0,0,0,0]) #Initial guest
    #Create Slist
    w = np.array([[0, 0, 1], [1, 0, 0], [1, 0, 0], [1, 0, 0]])
    w = np.transpose(w)
    q = np.array([[0, 0, 0], [0, 0, L1], [0, 0, L1+L2], [0, 0, L1+L2+L3]])
    q = np.transpose(q)
    S_M = np.array([])
    for i in range(N):
        S_M = sq.sqhtoscrew(w[:, i], q[:, i], 0)
        if i == 0:
            A = np.array([np.zeros(len(S_M))])
        A = np.concatenate((A, [S_M]), axis=0)   
    S_M = np.transpose(A[1:, :])
    # Just for comparisson purposes
    theta_d = np.array([np.pi/10, np.pi/6, np.pi/3, np.pi/2]) # Final position TEST
    Rd = np.array([[np.cos(yaw),-np.sin(yaw),0], [np.sin(yaw),np.cos(yaw),0], [0,0,1]])
    Pd = np.array([[x],[y],[z]])
    Tsd = np.array([[np.cos(yaw),-np.sin(yaw),0,x], [np.sin(yaw),np.cos(yaw),0,y], [0,0,1,z], [0,0,0,1]])
    # Inverse Kinematics algorithm
    Tsb = fk.FK_POE_S(S_M,theta,M) #End-effector position for initial guest Theta
    J=mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb),Tsd)))
    Vs=np.dot(mr.Adjoint(Tsb),J) # Get twist 
    Vs=np.transpose(Vs)
    error_w=np.linalg.norm(Vs[0:3])
    error_v=np.linalg.norm(Vs[3:6])
    i=0
    [theta_l,suc]=mr.IKinSpace(S_M,M,Tsd,theta,Ew,Ev) #Just to verify if math is correct
    while error_w>Ew or error_v>Ev and i<mx_it: #stop iterations if errors are less than desired or iteratiosn are over the limit
        Js_inv=np.linalg.pinv(mr.JacobianSpace(S_M,theta)) # Get pseudo inverse of Space Jacobian
        theta=np.add(theta,np.dot(Js_inv,Vs)) #get new theta
        Tsb = fk.FK_POE_S(S_M,theta,M)
        J=mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb),Tsd))) #Get Twist part 1
        Vs=np.dot(mr.Adjoint(Tsb),J) # Get twist part 2
        Vs=np.transpose(Vs)
        error_w=np.linalg.norm(Vs[0:3]) #get new rotation error
        error_v=np.linalg.norm(Vs[3:6]) #get new position error
        i=i+1 #count number of iterations

    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Desired joint theta for joint 1 - 4
    msg.position = [
        theta[0],
        theta[1],
        theta[2],
        theta[3]
    ]

    global pub
    # TODO
    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(msg)




def main():
    global pub

    # Create publisher (The angles requred to get to the position)
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber (The position we want the endeffector at wrt robot first joint)
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node (Give the node a name)
    rospy.init_node('Inverse_Kinematics_Node')

    # Loop the function so it runs continually
    rospy.spin()


if __name__ == '__main__':
    main()