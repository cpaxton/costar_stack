'''
(c) 2016 Chris Paxton
'''

import rospy

# import moveit messages
from moveit_msgs.msg import *
from moveit_msgs.srv import *


class SimplePlanning:
    
    def __init__(self,base_link,end_link,group,move_group_ns="/move_group",planning_scene_topic="/planning_scene");
        self.base_link = base_link
        self.end_link = end_link
        self.group = group
