#!/usr/bin/env python

import rospy
from costar_robot import WaypointManager

rospy.init_node('costar_waypoint_manager')
wm = WaypointManager()

rate = rospy.Rate(60)

while not rospy.is_shutdown():
  rate.sleep()
