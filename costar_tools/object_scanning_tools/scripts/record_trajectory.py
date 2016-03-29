#!/usr/bin/env python

import rospy
from costar import TrajectoryRecorder

rospy.init_node('costar_trajectory_recorder_node')
filename = rospy.get_param('~filename','trajectory.yml')

tr = TrajectoryRecorder()

rospy.spin()

print 'Done. Saving to file "%s"...'%(filename)
tr.save(filename)
print 'Saved!'

