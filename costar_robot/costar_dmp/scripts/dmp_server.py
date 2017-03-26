#!/usr/bin/env python

import rospy
from costar_dmp import CostarDMP

rospy.init_node('costar_dmp_server')
server = CostarDMP()

rate = rospy.Rate(10)

while not rospy.is_shutdown():

	server.tick()
	rate.sleep()