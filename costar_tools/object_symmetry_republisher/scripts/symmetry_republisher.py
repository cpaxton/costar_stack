#!/usr/bin/env python

import rospy
from object_symmetry_republisher import ObjectSymmetryRepublisher

rospy.init_node('object_symmetry_republisher_node')

osr = ObjectSymmetryRepublisher()

rospy.spin()
