#!/usr/bin/env python

import rospy
from simple_robot import CostarIIWADriver

rospy.init_node('simple_iiwa_driver')
driver = CostarIIWADriver()

rate = rospy.Rate(60)

while not rospy.is_shutdown():
  driver.tick()
  rate.sleep()

