#!/usr/bin/env python

import rospy
from simple_s_model import SimpleSModelServer

rospy.init_node("simple_s_model_server")
server = SimpleSModelServer("s_model")
server.open_gripper()
server.basic_mode()
rospy.spin()
