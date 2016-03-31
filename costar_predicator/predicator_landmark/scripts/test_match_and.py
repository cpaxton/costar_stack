#!/usr/bin/env python
import rospy
from predicator_msgs.srv import *
from predicator_msgs.msg import *

p1 = PredicateStatement(predicate='link',params=['*','',''],num_params=1)
p2 = PredicateStatement(predicate='up_from',params=['*','world','world'],num_params=2)
p3 = PredicateStatement(predicate='in_front_of',params=['*','world','world'],num_params=2)

srv = rospy.ServiceProxy('predicator/get_assignment',GetAssignment)
and_srv = rospy.ServiceProxy('predicator/match_AND',Query)

print srv(p1)
print srv(p2)
print srv(p3)

print and_srv([p1,p2,p3])

