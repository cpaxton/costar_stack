#!/usr/bin/env python

import rospy
from predicator_dvrk import dvrkPredicator

rospy.init_node('predicator_dvrk')

dvrkpre = dvrkPredicator()
dvrkpre.spin()
