#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf_conversions as tfc

from predicator_msgs import *

class MovementPredicator(object):

    def __init__(self, topic, valid_topic):
        pass

if __name__ == "__main__":
    rospy.init_node('predicator_movement_node')

    spin_rate = rospy.get_param('rate',10)

    print "starting movement node"

    try:
        mp = MovementPredicator('/predicator/input','/predicator/valid_input')

        while not rospy.is_shutdown()
            msg = mp.getMessage()
            gp.publish(msg)
        
            valid_msg = mp.getValidMessage()
            gp.publishValid(valid_msg)

            rate.sleep()

    except rospy.ROSInterruptException: pass

