#!/usr/bin/env python

import rospy
from costar_robot import CostarPSMDriver
from trajectory_msgs.msg import *

# rospy.init_node('costar_psm_driver')
driver = CostarPSMDriver()

pos1 = [0., 0., 0., 0., 0., 0.]
pos2 = [1., 0., 1., 0., 1., 0.]

traj1 = JointTrajectory()
traj2 = JointTrajectory()

traj1.points.append(JointTrajectoryPoint(positions=pos1))
traj2.points.append(JointTrajectoryPoint(positions=pos2))

rate = rospy.Rate(60)

while not rospy.is_shutdown():

  # debug command here
  driver.send_trajectory(traj1)
  rospy.sleep(0.5)
  driver.send_trajectory(traj2)
  rospy.sleep(0.5)

  driver.tick()
  rate.sleep()

