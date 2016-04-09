#!/usr/bin/env python

import rospy
from simple_handeye_calibration import *

rospy.init_node('handeye_server')

marker = rospy.get_param("~camera_marker","ar_marker_0")
camera_link = rospy.get_param("~camera_link","camera_link")
base_link = rospy.get_param("~base_link","base_link")
ee_marker = rospy.get_param("~ee_marker","ee_marker")

srv = SimpleHandeyeCalibration(
    marker=marker,
    camera_link=camera_link,
    ee_marker=ee_marker,
    base_link=base_link)

srv.spin(rospy.Rate(10))
