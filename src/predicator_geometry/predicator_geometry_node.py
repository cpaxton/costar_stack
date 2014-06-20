#!/usr/bin/env python

import rospy
import tf

from predicator_msgs.msg import *

class GeometryPredicator(object):

    def __init__(self, pub_topic):
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(pub_topic, PredicateList)
        self._frames = rospy.get_param('~frames')
        self._world_frame = rospy.get_param('~world_frame','world')
        self._higher_margin = rospy.get_param('~height_threshold',0.1)
        print self._frames

    '''
    addHeightPredicates()
    Determines if some objects are higher than others.
    Can be passed a margin/threshold, in case things aren't different enough to matter.
    higher(A, B) means A is higher than B
    '''
    def addHeightPredicates(self, msg, frame1, frame2):
        
        try:
            (trans1, rot1) = self._listener.lookupTransform(self._world_frame, frame1, rospy.Time(0))
            (trans2, rot2) = self._listener.lookupTransform(self._world_frame, frame2, rospy.Time(0))
        
            height_difference = trans2[2] - trans1[2]

            if height_difference > self._higher_margin:
                ps = PredicateStatement()
                ps.predicate = 'higher'
                ps.params[0] = frame2
                ps.params[1] = frame1
                ps.num_params = 2
                ps.value = height_difference
                msg.statements.append(ps)
            elif height_difference < -1 * self._higher_margin:
                ps = PredicateStatement()
                ps.predicate = 'lower'
                ps.params[0] = frame2
                ps.params[1] = frame1
                ps.num_params = 2
                ps.value = height_difference
                msg.statements.append(ps)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

        return msg

    def addRelativeDirectionPredicates(self, msg, frame1, frame2, ref):

        try:
            (trans1, rot1) = self._listener.lookupTransform(ref, frame1, rospy.Time(0))
            (trans2, rot2) = self._listener.lookupTransform(ref, frame2, rospy.Time(0))

            x_diff = trans2[0] - trans1[0]
            y_diff = trans2[1] - trans1[1]
            z_diff = trans2[2] - trans1[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

        return msg

    '''
    getPredicateMessage()
    loop over all frames we are supposed to examine
    determine if there are any interesting relationships between them
    '''
    def getPredicateMessage(self):

        msg = PredicateList()
        msg.header.frame_id = rospy.get_name()

        for frame1 in self._frames:
            for frame2 in self._frames:
                if frame1 == frame2:
                    continue
                else:
                    # check height predicates
                    msg = self.addHeightPredicates(msg, frame1, frame2)

        return msg

    '''
    publish()
    Send a message to whatever topic this GeometryPredicator node was set up to handle.
    '''
    def publish(self, msg):
        self._publisher.publish(msg)

if __name__ == "__main__":

    rospy.init_node('predicator_geometry_node')

    spin_rate = rospy.get_param('rate',10)
    rate = rospy.Rate(spin_rate)

    print "starting geometry node"

    try:

        gp = GeometryPredicator('/predicator/input')

        while not rospy.is_shutdown():
            msg = gp.getPredicateMessage()
            print msg
            gp.publish(msg)
            rate.sleep()

    except rospy.ROSInterruptException: pass
