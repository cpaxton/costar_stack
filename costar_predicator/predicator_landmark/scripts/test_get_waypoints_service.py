#!/usr/bin/env python
import rospy
from predicator_msgs.srv import *
from predicator_msgs.msg import *
from geometry_msgs.msg import Pose

srv = rospy.ServiceProxy('predicator/get_waypoints',GetWaypoints)

p2 = PredicateStatement(predicate='up_from',params=['*','world','world'],num_params=2)
p3 = PredicateStatement(predicate='in_front_of',params=['*','world','world'],num_params=2)

p = Pose()
p.orientation.w = 1.0

print p

resp = srv(frame_type="link",predicates=[p2,p3],transforms=[p])

print resp
