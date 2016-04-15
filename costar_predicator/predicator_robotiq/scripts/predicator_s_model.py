#!/usr/bin/env python

import rospy
from predicator_robotiq import SModelPredicator

rospy.init_node('predicator_s_model')

smp = SModelPredicator()

rospy.spin()
