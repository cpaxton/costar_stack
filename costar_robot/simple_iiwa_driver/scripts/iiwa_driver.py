#!/usr/bin/env python

import rospy
from simple_iiwa_driver import *

rospy.init_node('simple_iiwa_driver')
driver = SimpleIIWADriver()

rate = rospy.Rate(60)

while not rospy.is_shutdown():
  driver.tick()
  rate.sleep()

