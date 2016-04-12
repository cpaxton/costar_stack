#!/usr/bin/env python

import rospy
from costar import TrajectoryRecorder

rospy.init_node('costar_trajectory_replayer_node')
filename = rospy.get_param('~filename','trajectory.yml')
hz = rospy.get_param('~hz',0.2)

tr = TrajectoryRecorder()
tr.load(filename)
#rospy.spin()

print "Playing..."
tr.play(hz=hz)
print "Done."

