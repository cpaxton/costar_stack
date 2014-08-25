#!/usr/bin/env python

import rospy
from predicator_planning.srv import *
from trajectory_msgs.msg import JointTrajectory

rospy.init_node('test_predicate_planning_node')
planner = rospy.ServiceProxy('predicator/plan', PredicatePlan)
pub = rospy.Publisher('gazebo/traj_rml/joint_traj_cmd', JointTrajectory)

req = PredicatePlanRequest()
req.group = 'arm'
req.robot = 'wam'
res = planner(req)

print res

pub.publish(res.path)
