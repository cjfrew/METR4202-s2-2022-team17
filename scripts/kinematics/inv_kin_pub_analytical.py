#!/usr/bin/env python3

# Inverse Kinematics Libs
import numpy as np

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose


# Inverse kinematics function
def inverse_kinematics(pose: Pose) -> JointState:
    # Determine x,y,z & quat_x,quat_y,quat_z,quat_w from pose
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    # Length of 4R robots
    L1 = 55
    L2 = 117
    L3 = 95
    L4 = 115

    mx_length=L2+L3+L4 # Max Length of arm
    #Quadrants
    if np.sqrt(x**2+y**2)>mx_length/2:
        phi=-np.pi/2
    else: 
        phi=np.pi

    # XYZ limits
    if np.sqrt(x**2+y**2)>mx_length:
        ang = np.array([0,0,0,0])
        
    else:
        theta1 = -np.arctan(x/y)
        Pz = z - L1
        Py = np.sqrt(x**2+y**2)
        Wz = Pz-L4*np.cos(phi)
        Wy = Py-L4*np.sin(phi)
        c2 = (Wz**2+Wy**2-L2**2-L3**2)/(2*L2*L3)
        theta3 = np.arccos(c2)
        theta2 = np.arctan(Wy/Wz)-np.arctan((L3*np.sin(theta3))/(L2+L3*np.cos(theta3)))
        theta4 = phi-theta2-theta3
        ang = np.array([theta1,theta2,theta3,theta4])
            
    b=0
    for a in ang:
        while a>np.pi:
            a=a-2*np.pi
            
        while a<=-np.pi:
            a=a+2*np.pi
        
        ang[b]=a
        b=b+1
	
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Desired joint theta for joint 1 - 4
    msg.position = [
        ang[0],
        ang[1],
        ang[2],
        ang[3]
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
