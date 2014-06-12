#!/usr/bin/python

import rospy
from predicator_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('predicator_dummy_module')
    pub = rospy.Publisher('predicator/input', PredicateList)

    ps = PredicateList()
    ps.header.frame_id = rospy.get_name()

    ps.statements = [
            PredicateStatement(
                predicate='left_of',
                num_params=3,
                params=['Block1', 'Block2', 'ur5']),
            PredicateStatement(
                predicate='right_of',
                num_params=3,
                params=['Block2', 'Block1', 'ur5']),
            PredicateStatement(
                predicate='behind',
                num_params=3,
                params=['Block1', 'Block2', 'Block1']),
            PredicateStatement(
                predicate='ahead',
                num_params=3,
                params=['Block2', 'Block1', 'Block2']),
            PredicateStatement(
                predicate='left_of',
                num_params=3,
                params=['ur5', 'Block1', 'Block2']),
            PredicateStatement(
                predicate='right_of',
                num_params=3,
                params=['ur5', 'Block2', 'Block2']),
            PredicateStatement(
                predicate='touching',
                num_params=2,
                params=['Block1', 'Block2', '']),
            PredicateStatement(
                predicate='touching',
                num_params=2,
                params=['Block1', 'Block2', '']),
            PredicateStatement(
                predicate='found_object',
                num_params=1,
                params=['Block1','','']),
            PredicateStatement(
                predicate='found_object',
                num_params=1,
                params=['Block2','',''])
            ]

    rate = rospy.Rate(1)

    print ps
    print type(ps.statements[0].num_params)

    try:
        while not rospy.is_shutdown():
            pub.publish(ps)
            rate.sleep()
    except rospy.ROSInterruptException: pass
