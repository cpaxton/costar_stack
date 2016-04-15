#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

controllers = []

def joint_traj_pt_cb(msg):
    global controllers
    print controllers


for i in range(1,8)
    controllers

    sub = rospy.Subscriber("joint_traj_pt_cmd",JointTrajectoryPoint,joint_traj_pt_cb)


