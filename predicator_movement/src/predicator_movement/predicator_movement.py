#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf_conversions as tfc

from predicator_msgs import *

class MovementPredicator(object):

    def __init__(self, topic, valid_topic):
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(topic, PredicateList)
        self._valid_publisher = rospy.Publisher(valid_topic, ValidPredicates)
        self._frames = rospy.get_param('~frames')
        self._world_frame = rospy.get_param('~world_frame','world')
        self._trans_threshold = rospy.get_param('~translation_velocity_threshold', 0.01)
        self._rot_threshold = rospy.get_param('~rotation_velocity_threshold', 0.01)
        self._s0 = 0
        self._s1 = 0
        pass

    '''
    update()
    Read in more messages
    '''
    def update(self):
        try:
            for frame in self._frames:

                # get position in world coordinates
                (trans, rot) = self._listener.lookupTransform(self._world_frame, frame, rospy.Time(0))

                # 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        pass

    '''
    getMessage()
    '''
    def getMessage(self):
        pass

    '''
    getValidMessage()
    '''
    def getValidMessage(self):
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

