#!/usr/bin/env python

from costar_robot_manager import PsmDriver
import rospy
from trajectory_msgs import JointTrajectory
from trajectory_msgs import JointTrajectoryPoint

rospy.init_node('test_psm_driver_send_trajectory')


# create a joint trajectory

psm1 = PsmDriver()

psm1.send_trajectory(traj)

