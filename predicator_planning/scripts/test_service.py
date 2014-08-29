#!/usr/bin/env python

import rospy
from predicator_msgs.msg import PredicateStatement
from predicator_planning.srv import *
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.msg import *

from control_msgs.msg import *
import actionlib

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

req.required_true.append(PredicateStatement(predicate="near_xy",params=['wam/wrist_palm_link','peg1/peg_top_link','']))

req.goal_true.append(PredicateStatement(predicate="above",params=['wam/wrist_palm_link','peg1/peg_top_link','world']))
req.goal_true.append(PredicateStatement(predicate="near_xy",params=['wam/wrist_palm_link','peg1/peg_top_link','']))
#req.goal_true.append(PredicateStatement(predicate="in_front_of",params=['wam/wrist_palm_link','peg1/peg_top_link','world']))
req.goal_true.append(PredicateStatement(predicate="behind",params=['wam/wrist_palm_link','peg1/peg_top_link','world']))
#req.goal_true.append(PredicateStatement(predicate="left_of",params=['wam/wrist_palm_link','peg1/peg_top_link','world']))
#req.goal_false.append(PredicateStatement(predicate="near",params=['wam/wrist_palm_link','peg1/peg_top_link','']))

res = planner(req)

print res

disp = DisplayTrajectory()
traj = RobotTrajectory()
traj.joint_trajectory = res.path
disp.trajectory_start.is_diff = True
disp.trajectory.append(traj)

#pub.publish(res.path)
display_pub.publish(disp)

client = actionlib.SimpleActionClient('/gazebo/traj_rml/action', control_msgs.msg.FollowJointTrajectoryAction)
goal = FollowJointTrajectoryGoal()

next_t = 0
for point in res.path.points:
    point.time_from_start.secs = next_t
    next_t = next_t + 2

print res.path

goal.trajectory = res.path

rospy.loginfo("Sending trajectory to trajectory action...")

client.wait_for_server()
client.send_goal(goal)

rospy.loginfo("Waiting...")

client.wait_for_result()

rospy.loginfo("Trajectory action finished.")

res = client.get_result()

print res

rospy.spin()
