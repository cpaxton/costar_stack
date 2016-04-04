#!/usr/bin/env python


import rospy
from predicator_msgs.msg import *
from predicator_msgs.srv import *

rospy.init_node('test_smart_move_node')

obj_class = "node_uniform" # based on object detection output
right_of = PredicateStatement(predicate="right_of",params=['*','ar_marker_2','world'])

