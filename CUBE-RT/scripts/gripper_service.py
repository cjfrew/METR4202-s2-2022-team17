#!/usr/bin/env python3

import rospy as rp
import numpy as np

import pigpio as pp

from std_srvs.srv import SetBool
from constants import *

def shut_gripper(grip: bool):
    rpi = pp.pi()
    rpi.set_mode(18, pp.OUTPUT)

    if grip:
        rpi.set_servo_pulsewidth(18, GRIP)
    else:
        rpi.set_servo_pulsewidth(18, DROP)

def handle_shut_gripper(request: SetBool):
    shut_gripper(request.data)
    
    if request.data:
        message = '!!! YOU BEEN GRIPPED BRAH !!!'
    else:
        message = '!!! GET DROPPED SUCKER !!!'

    return [True, message]

def shut_gripper_server():
    rp.init_node('Gripper')
    service = rp.Service('shut_gripper', SetBool, handle_shut_gripper)
    print('Gripper standing by...')
    rp.spin()

if __name__ == "__main__":
    shut_gripper_server()