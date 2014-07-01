#!/usr/bin/env python

import rospy
import numpy as np
import tf
import tf_conversions as tfc

from predicator_msgs.msg import *

def norm3(t):
    return np.sqrt((t[0]*t[0]) + (t[1]*t[1]) + (t[2]*t[2]))

def norm4(t):
    return np.sqrt((t[0]*t[0]) + (t[1]*t[1]) + (t[2]*t[2]))


'''
MovementPredicator
Module running exponential filtering/smoothing of new inputs and 
'''
class MovementPredicator(object):

    def __init__(self, topic, valid_topic):
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(topic, PredicateList)
        self._valid_publisher = rospy.Publisher(valid_topic, ValidPredicates)
        self._frames = rospy.get_param('~frames')
        self._world_frame = rospy.get_param('~world_frame','world')
        self._trans_threshold = rospy.get_param('~translation_velocity_threshold', 0.01)
        self._rot_threshold = rospy.get_param('~rotation_velocity_threshold', 0.01)
        self._dist_threshold = rospy.get_param('~distance_change_threshold', 0.01)
        self._alpha = rospy.get_param('~alpha', 0.50)
        self._st0 = {}
        self._st1 = {}
        self._sr0 = {}
        self._sr1 = {}
        self._t0 = {}
        self._t1 = {}
        self._r0 = {}
        self._r1 = {}
        self._started = False
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

                # save to things
                if frame in self._t1:
                    self._t0[frame] = self._t1[frame]
                    self._r0[frame] = self._r1[frame]

                    self._st0[frame] = self._st1[frame]
                    self._sr0[frame] = self._sr1[frame]

                    self._st1[frame] = (self._alpha * self._t0[frame]) + ((1.0 - self._alpha) * self._st0[frame])
                    self._sr1[frame] = (self._alpha * self._r0[frame]) + ((1.0 - self._alpha) * self._sr0[frame])

                self._t1[frame] = np.asarray(trans)
                self._r1[frame] = np.asarray(rot)
                self._st1[frame] = self._t1[frame]
                self._sr1[frame] = self._r1[frame]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        #print self._st1
        #print self._sr1

    '''
    getMessage()
    Checks for movement, computes velocity, etc.
    '''
    def getMessage(self):
        msg = PredicateList()

        '''
        iterate over all frames to get movement stuff
        '''
        for frame in self._frames:

            if not frame in self._st0:
                continue

            tvel = np.linalg.norm(self._st1[frame] - self._st0[frame], ord=2)
            rvel = np.linalg.norm(self._sr1[frame] - self._sr0[frame], ord=2)

            tps = PredicateStatement()
            tps.predicate = "translation_velocity"
            tps.num_params = 1
            tps.params[0] = frame
            tps.value = tvel

            rps = PredicateStatement()
            rps.predicate = "rotation_velocity"
            rps.num_params = 1
            rps.params[0] = frame
            rps.value = rvel

            msg.statements.append(tps)
            msg.statements.append(rps)

            if tvel > self._trans_threshold:
                ps = PredicateStatement()
                ps.predicate = "moving"
                ps.num_params = 1
                ps.params[0] = frame
                msg.statements.append(ps)

            if rvel > self._rot_threshold:
                ps = PredicateStatement()
                ps.predicate = "rotating"
                ps.num_params = 1
                ps.params[0] = frame
                msg.statements.append(ps)

            '''
            iterate over list of framey things
            go through other frames and see how they do compared to one another
            '''
            for other_frame in self._frames:
                if not other_frame == frame and other_frame in self._st0:
                    tdiff0 = self._st0[frame] - self._st0[other_frame]
                    tdiff1 = self._st1[frame] - self._st1[other_frame]
                    rdiff0 = self._sr0[frame] - self._sr0[other_frame]
                    rdiff1 = self._sr1[frame] - self._sr1[other_frame]

                    tdiff = np.linalg.norm(tdiff1 - tdiff0, ord=2)
                    rdiff = np.linalg.norm(rdiff1 - rdiff0, ord=2)
                    
                    tps = PredicateStatement()
                    tps.predicate = "relative_translation_velocity"
                    tps.num_params = 2
                    tps.params[0] = frame
                    tps.params[1] = other_frame
                    tps.value = tdiff
                    msg.statements.append(tps)


                    rps = PredicateStatement()
                    rps.predicate = "relative_rotation_velocity"
                    rps.num_params = 2
                    rps.params[0] = frame
                    rps.params[1] = other_frame
                    rps.value = rdiff
                    msg.statements.append(rps)

                    if tdiff > self._trans_threshold:
                        ps = PredicateStatement()
                        ps.predicate = "moving_relative_to"
                        ps.num_params = 2
                        ps.params[0] = frame
                        ps.params[1] = other_frame
                        msg.statements.append(ps)

                    if rvel > self._rot_threshold:
                        ps = PredicateStatement()
                        ps.predicate = "rotating_relative_to"
                        ps.num_params = 2
                        ps.params[0] = frame
                        ps.params[1] = other_frame
                        msg.statements.append(ps)


                    '''
                    section: departing/approaching frame of reference
                    '''
                    tnorm1 = np.linalg.norm(tdiff1, ord=2)
                    tnorm0 = np.linalg.norm(tdiff0, ord=2)
                    if tnorm1 - tnorm0 > self._dist_threshold:
                        ps = PredicateStatement()
                        ps.predicate = "departing"
                        ps.num_params = 2
                        ps.params[0] = frame
                        ps.params[1] = other_frame
                        msg.statements.append(ps)
                    elif tnorm0 - tnorm1 > self._dist_threshold:
                        ps = PredicateStatement()
                        ps.predicate = "approaching"
                        ps.num_params = 2
                        ps.params[0] = frame
                        ps.params[1] = other_frame
                        msg.statements.append(ps)


        return msg

    '''
    getValidMessage()
    Return information about what all of the possible values are for predicates.
    This information is important for UI plugins, among other things.
    '''
    def getValidMessage(self):
        msg = ValidPredicates()
        msg.predicates = ['approaching', 'departing', 'moving', 'rotating', 'moving_relative_to', 'rotating_relative_to']
        msg.value_predicates = ['translation_velocity', 'rotation_velocity', 
            'relative_translation_velocity',
            'relative_rotation_velocity']
        msg.assignments = [frame for frame in self._frames]

        return msg

    def publishValid(self, msg):
        self._valid_publisher.publish(msg)

    def publish(self, msg):
        self._publisher.publish(msg)

if __name__ == "__main__":
    rospy.init_node('predicator_movement_node')

    spin_rate = rospy.get_param('rate',10)
    rate = rospy.Rate(spin_rate)

    print "starting movement node"

    try:
        mp = MovementPredicator('/predicator/input','/predicator/valid_input')

        while not rospy.is_shutdown():
            
            mp.update()

            msg = mp.getMessage()
            mp.publish(msg)
        
            valid_msg = mp.getValidMessage()
            mp.publishValid(valid_msg)

            rate.sleep()

    except rospy.ROSInterruptException: pass

