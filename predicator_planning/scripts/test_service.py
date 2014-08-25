#!/usr/bin/env python

import rospy
from predicator_planning.srv import *
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import *

rospy.init_node('test_predicate_planning_node')
planner = rospy.ServiceProxy('predicator/plan', PredicatePlan)
pub = rospy.Publisher('gazebo/traj_rml/joint_traj_cmd', JointTrajectory)
display_pub = rospy.Publisher('predicator/display_plan', DisplayTrajectory)

req = PredicatePlanRequest()
req.group = 'arm'
req.robot = 'wam'
res = planner(req)

print res

disp = DisplayTrajectory()
traj = RobotTrajectory()
traj.joint_trajectory = res.path
disp.trajectory_start.is_diff = True
disp.trajectory.append(traj)

pub.publish(res.path)
display_pub.publish(disp)

rospy.spin()
