#!/usr/bin/env python

import rospy
from predicator import *

rospy.init_node('predicator_get_waypoints_service')

srv = GetWaypointsService()

rospy.spin()
