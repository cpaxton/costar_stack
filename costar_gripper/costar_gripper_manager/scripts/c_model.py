#!/usr/bin/env python

import rospy
from costar_gripper import SimpleCModelServer

rospy.init_node("simple_c_model_server")
verbose = rospy.get_param('~verbose',False)
server = SimpleCModelServer("c_model",verbose=verbose)
server.open_gripper()
#server.basic_mode()
rospy.spin()
