#!/usr/bin/env python

import rospy
from predicator_msgs.msg import *

'''
FAKE CLASSIFICATION MODULE
This module exists to publish known class information about a robot and its world.
Predicates published should be things like:

    robot(ur5)
    object(ur5)
    object(Block1)
    Block(Block1)

These messages are used when reasoning about the environment.
'''

if __name__ == '__main__':
    rospy.init_node('predicator_fake_class_node')

    spin_rate = rospy.get_param('rate',10)
    rate = rospy.Rate(spin_rate)

    print "starting fake classification node"

    msg = PredicateList()

    class_info = rospy.get_param('~class_info')
    print "BASIC CLASS INFO: " + str(class_info)
    param_class_info = rospy.get_param('~param_class_info')
    print "PARAMETERIZED CLASS INFO: " + str(param_class_info)

    try:

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException: pass
