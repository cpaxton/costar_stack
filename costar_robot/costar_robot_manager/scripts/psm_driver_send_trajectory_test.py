#!/usr/bin/env python
import rospy
from costar_robot import CostarPSMDriver
from trajectory_msgs.msg import *
from costar_robot_msgs.srv import *
#from costar_robot_msgs.msg import *
# from trajectory_msgs import JointTrajectory
#from trajectory_msgs import JointTrajectoryPoint

#def traj_send():

#rospy.init_node('test_psm_driver_send_trajectory')


# create a joint trajectory
traj = JointTrajectory()
pt = JointTrajectoryPoint()
pt.positions = [0.0,0.0,-0.05]
traj.points.append(pt)

psm1 = CostarPSMDriver()

#req = EmptyCommandRequest()
psm1.home()
psm1.insert_tool()
psm1.send_trajectory(traj)

