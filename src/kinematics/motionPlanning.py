"""


.___  ___.   ______   .___________. __    ______   .__   __.                                           
|   \/   |  /  __  \  |           ||  |  /  __  \  |  \ |  |                                           
|  \  /  | |  |  |  | `---|  |----`|  | |  |  |  | |   \|  |                                           
|  |\/|  | |  |  |  |     |  |     |  | |  |  |  | |  . `  |                                           
|  |  |  | |  `--'  |     |  |     |  | |  `--'  | |  |\   |                                           
|__|  |__|  \______/      |__|     |__|  \______/  |__| \__|                                           
                                                                                                       
.______    __          ___      .__   __. .__   __.  __  .__   __.   _______    .______   ____    ____ 
|   _  \  |  |        /   \     |  \ |  | |  \ |  | |  | |  \ |  |  /  _____|   |   _  \  \   \  /   / 
|  |_)  | |  |       /  ^  \    |   \|  | |   \|  | |  | |   \|  | |  |  __     |  |_)  |  \   \/   /  
|   ___/  |  |      /  /_\  \   |  . `  | |  . `  | |  | |  . `  | |  | |_ |    |   ___/    \_    _/   
|  |      |  `----./  _____  \  |  |\   | |  |\   | |  | |  |\   | |  |__| |  __|  |          |  |     
| _|      |_______/__/     \__\ |__| \__| |__| \__| |__| |__| \__|  \______| (__) _|          |__|     
                                                                                                       

Code written by team 17: 
Last edit: 

file dictates what blocks can and cannot be picked up based on the placement 
of tags present.
"""

################################### Imports ###################################

import rospy
import math
import numpy as np
import tf2_ros as tf2

from tf.transformations import euler_from_quaternion

################################## variables ##################################

length1 = 0.055 #m
length2 = 0.117 #m

max_reach = length1 + length2
min_reach = length2 - length1

class publishMotionPlan: 
    """
        This class handles finding and moving the robotic arm to multiple tag 
        locations. Classes are also cool  
    """
    
    def __init__(self) -> None:
        pass
    
    def tag_detection(self, msg): 
        """Checks for tags (excluding tags at the base) and aims to go towards 
            the first found one.

        Args:
            msg (_type_): a message passed to obtain data from another script.
        """
        
        #Check the base is not rotating 
        
        #loop through message transforms - try and find the tag id's 
        
        #compare ID's to base ID
        
        #If diff, 
            #check reachability (idk if this is a word??¯\_(ツ)_/¯)
                #try moving arm into position and update transformations
                #except if collides (also idk how to check this... yet)

        #if all success: 
            #publish the data (position) to where ever it is needed.
        pass
    
    def rotation(self, rotation): 
        """Checks whether the blocks are rotating and returns a bool.
        1. moving
        0. not moving

        Args:
            rotation (_type_): a message variable of rotating state 
        """
        self.moving = rotation.data
    
    def pos(self, msg): 
        """_summary_

        Args:
            msg (_type_): _description_
        """
        #Read in position data from transform translations. X and y
        pass
    
    def reachable(self, msg): 
        #Check if the dist. > max or < min
        
        pass