#!/usr/bin/env python

import rospy
from costar_robot import CostarUR5Driver

rospy.init_node('costar_ur5_driver')

is_sim = rospy.get_param('~simulation',False)
ipaddr = rospy.get_param('~ip_address','192.168.1.155')

driver = CostarUR5Driver(ip_address=ipaddr,simulation=is_sim)

rate = rospy.Rate(1)

while not rospy.is_shutdown():
  driver.tick()
  rate.sleep()

