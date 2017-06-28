#!/usr/bin/env python

import rospy
from gripper_dvrk import SimplePSMGripperServer

# rospy.init_node("simple_psm_gripper_server")
verbose = rospy.get_param('~verbose',False)
server = SimplePSMGripperServer("/costar/gripper",verbose=verbose)
server.init_gripper()
rospy.sleep(.5)
server.open_gripper()
rospy.spin()