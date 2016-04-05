#!/usr/bin/env python

import rospy
from smart_waypoint_manager import *

rospy.init_node('test_smartmove_waypoints')
swm = SmartWaypointManager()

swm.load_all()

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    swm.update_tf()
    #print swm.get_new_waypoint('gbeam_link_1/gbeam_link')
    print swm.get_new_waypoint('Obj::link_uniform::1')
    rate.sleep()
