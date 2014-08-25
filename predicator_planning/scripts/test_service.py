#!/usr/bin/env python

import rospy
from predicator_msgs.msg import PredicateStatement
from predicator_planning.srv import *
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import *

rospy.init_node('test_predicate_planning_node')
planner = rospy.ServiceProxy('predicator/plan', PredicatePlan)
pub = rospy.Publisher('gazebo/traj_rml/joint_traj_cmd', JointTrajectory)
display_pub = rospy.Publisher('predicator/display_plan', DisplayTrajectory)

others = ['wam2', 'peg1', 'peg2', 'ring1', 'stage']

req = PredicatePlanRequest()
req.group = 'arm'
req.robot = 'wam'

for obj in others:
    req.required_false.append(PredicateStatement(predicate="touching",params=['wam',obj,''],num_params=3))
    req.goal_false.append(PredicateStatement(predicate="touching",params=['wam',obj,''],num_params=3))

#req.goal_true.append(PredicateStatement(predicate="in_front_of",params=['wam/wrist_palm_link','peg1/peg_top_link','world']))
req.goal_true.append(PredicateStatement(predicate="near",params=['wam/wrist_palm_link','peg1/peg_top_link','world']))

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
