#!/usr/bin/env python

import rospy
import tf

from predicator_msgs.msg import *

class GeometryPredicator(object):

    def __init__(self, pub_topic):
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(pub_topic, PredicateList)
        self._frames = rospy.get_param('~frames')
        print self._frames

    def getTransform(self, frame1, frame2):
        try:
            (trans, rot) = listener.lookupTransform(frame1, frame2, rospy.Time(0))
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

    '''
    getPredicateMessage()
    loop over all frames we are supposed to examine
    determine if there are any interesting relationships between them
    '''
    def getPredicateMessage(self):

        msg = PredicateList()
        msg.header.frame_id = rospy.get_name()

        for frame1 in frames:
            for frame2 in frames:
                if frame1 == frame2:
                    continue
                else:

            

if __name__ == "__main__":

    rospy.init_node('predicator_geometry_node')

    spin_rate = rospy.get_param('rate',10)
    rate = rospy.Rate(spin_rate)

    print "starting geometry node"

    try:

        gp = GeometryPredicator('/predicator/input')

        while not rospy.is_shutdown():
            rate.sleep()

    except rospy.ROSInterruptException: pass
