#!/usr/bin/env python

import rospy

rospy.init_node('predicator_detected_objects_node')

dop = DetectedObjectsPublisher()

rospy.spin()
