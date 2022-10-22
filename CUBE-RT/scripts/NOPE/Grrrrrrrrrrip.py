#!/usr/bin/env python3

import rospy as rp

from std_msgs.msg import Bool

def inverse_kinematics() -> Bool:

    msg = Bool(data = True)
    global pub
    pub.publish(msg)

def main():
    global pub
    pub = rp.Publisher('/gripper_state', Bool, queue_size=1)
    rp.init_node('Inverse_Kinematics')
    rp.spin()

if __name__ == '__main__':
    main()