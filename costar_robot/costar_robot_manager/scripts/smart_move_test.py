#!/usr/bin/env python


import rospy
from predicator_msgs.msg import *
from predicator_msgs.srv import *

from simple_iiwa_driver import *
from simple_planning import *
from smart_waypoint_manager import *
from geometry_msgs.msg import Pose

rospy.init_node('test_smart_move_node')

obj_class = "node_uniform" # based on object detection output
right_of = PredicateStatement(predicate="left_of",params=['*','world','world'])
pose = Pose()
pose.position.x = 0.05
pose.position.y = -0.35
pose.position.z = 0.01
pose.orientation.x = -0.63
pose.orientation.y = -0.77
pose.orientation.z = 0.02
pose.orientation.w = 0.07

driver = SimpleIIWADriver()
swm = SmartWaypointManager()

rospy.sleep(2.0)

rate = rospy.Rate(60)

req = SmartMoveRequest()
req.predicates = [right_of]
req.obj_class = obj_class
req.pose = pose

print req

print driver.smart_move_call(req)

rate = rospy.Rate(10)
while not rospy.is_shutdown():
    swm.update_tf()
    rate.sleep()
