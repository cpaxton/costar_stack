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

    spin_rate = rospy.get_param('rate',1)
    rate = rospy.Rate(spin_rate)

    print "starting fake classification node"

    pub = rospy.Publisher('/predicator/input', PredicateList, queue_size = 100)
    vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size = 100)

    msg = PredicateList()
    valid_msg = ValidPredicates()
    valid_msg.pheader.source = rospy.get_name()
    msg.pheader.source = rospy.get_name()
    valid_msg.pheader.source = rospy.get_name()

    class_info = rospy.get_param('~class_info')
    print "BASIC CLASS INFO: " + str(class_info)
    try:
        param_class_info = rospy.get_param('~param_class_info')
        print "PARAMETERIZED CLASS INFO: " + str(param_class_info)
    except KeyError:
        print "No parameterized class info!"
        param_class_info = []


    for group in class_info:
        members = group["members"]
        if not group["name"] in valid_msg.predicates:
            valid_msg.predicates.append(group["name"])
        for mem in members:
            ps = PredicateStatement()
            ps.predicate = group["name"]
            ps.num_params = 1
            ps.params[0] = mem
            msg.statements.append(ps)
            if not mem in valid_msg.assignments:
                valid_msg.assignments.append(mem)

    for group in param_class_info:
        members = group["members"]
        if not group["name"] in valid_msg.predicates:
            valid_msg.predicates.append(group["name"])
        for mem in members:
            ps = PredicateStatement()
            ps.predicate = group["name"]
            ps.num_params = 1
            ps.params[0] = mem
            ps.params[1] = group["parent"]
            msg.statements.append(ps)
            if not mem in valid_msg.assignments:
                valid_msg.assignments.append(mem)
            if not group["parent"] in valid_msg.assignments:
                valid_msg.assignments.append(group["parent"])

    print msg
    print valid_msg

    try:

        while not rospy.is_shutdown():
            pub.publish(msg)
            vpub.publish(valid_msg)

            rate.sleep()

    except rospy.ROSInterruptException: pass
