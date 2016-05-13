#!/usr/bin/env python

import rospy
from costar_gripper import SimpleSModelServer

rospy.init_node("simple_s_model_server")
server = SimpleSModelServer("s_model")
server.open_gripper()
server.basic_mode()
rospy.spin()
