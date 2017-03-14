#!/usr/bin/env python

import rospy
from costar_gripper import SimpleCModelServer

rospy.init_node("simple_c_model_server")
verbose = rospy.get_param('~verbose',False)
server = SimpleCModelServer("/costar/gripper",verbose=verbose)
server.open_gripper()
rospy.sleep(.5)
server.open_gripper()
#server.basic_mode()
rospy.spin()
