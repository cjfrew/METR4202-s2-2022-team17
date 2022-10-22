#!/usr/bin/env python3

import rospy as rp

import pigpio as pp

from std_msgs.msg import Bool
from constants import *

def shut_gripper(grip: Bool):
    rpi = pp.pi()
    rpi.set_mode(18, pp.OUTPUT)

    if grip.data:
        rpi.set_servo_pulsewidth(18, GRIP)
        rp.loginfo('!!! YOU BEEN GRIPPED BRAH !!!')
    else:
        rpi.set_servo_pulsewidth(18, DROP)
        rp.loginfo('!!! GET DROPPED SUCKER !!!')

def main():
    sub = rp.Subscriber('/gripper_state', Bool, shut_gripper)
    rp.init_node('Gripper')
    rp.spin()

if __name__ == "__main__":
    main()