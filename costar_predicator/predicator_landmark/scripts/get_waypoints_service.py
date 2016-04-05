#!/usr/bin/env python

import rospy
from predicator_landmark import *

rospy.init_node('predicator_get_waypoints_service')

srv = GetWaypointsService()

rospy.spin()
