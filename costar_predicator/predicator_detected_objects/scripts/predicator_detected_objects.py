#!/usr/bin/env python

import rospy
from predicator import DetectedObjectsPublisher

rospy.init_node('predicator_detected_objects_node')

dop = DetectedObjectsPublisher()

rospy.spin()
