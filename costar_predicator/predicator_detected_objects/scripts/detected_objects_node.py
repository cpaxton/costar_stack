#!/usr/bin/env python

import rospy
from predicator_detected_objects import DetectedObjectsPublisher
#from predicator_detected_objects import *

print "Starting node..."
rospy.init_node('predicator_detected_objects')

print "Creating publisher..."
dop = DetectedObjectsPublisher()

rospy.spin()
