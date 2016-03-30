#!/usr/bin/env python

import rospy
from costar import TrajectoryRecorder

rospy.init_node('costar_trajectory_replayer_node')
filename = rospy.get_param('~filename','trajectory.yml')
service_name = rospy.get_param('~service_name','/record_camera')

tr = TrajectoryRecorder()
tr.load(filename)
#rospy.spin()

print "Playing..."
tr.play_and_call_empty_service(service=service_name)
print "Done."

