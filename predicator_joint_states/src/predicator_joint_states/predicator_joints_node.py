#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *
from predicator_msgs.msg import *

if __name__ == "__main__":
    rospy.init_node("predicator_joints_node")
    pub = rospy.Publisher('predicator/input', PredicateList)
    vpub = rospy.Publisher('predicator/valid_input', ValidPredicates)

    rate = rospy.Rate(30)
    
    # read in some information
    config = rospy.get_param('~js_config')
    print config

    while not rospy.is_shutdown():

        # publish predicates message
        # publish 

        rate.sleep()


