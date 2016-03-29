#!/usr/bin/env python

import rospy
from costar import TrajectoryRecorder

rospy.init_node('costar_trajectory_recorder_node')

tr = TrajectoryRecorder()

rospy.spin()

print "done"