import numpy as np
import sqhtoscrew as sq
import FK_POE_S as fk
import inv_trans_mtx as itm
import modern_robotics as mr
import rospy
from std_msgs.msg import Float64MultiArray

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
Tsd = fk.FK_POE_S(S_M,theta_d,M) # Transformation matrix desired for comparisson purposes
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

print(theta) # publish angles from own IK function
print(theta_l)# publish angle from ModernRobotics IK function

#Publish once IK angles
pub=rospy.Publisher('ThetaIK',Float64MultiArray, queue_size=10)
rospy.init_node('IK',anonymous=True)
rate=rospy.Rate(1)
angles=Float64MultiArray()
angles.data=theta
rospy.loginfo("Data is send")
pub.publish(angles)
rate.sleep()
