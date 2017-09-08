#!/usr/bin/env python

import rospy
import numpy as np
import tf
import PyKDL as kdl
import tf_conversions.posemath as pm
from predicator_msgs.msg import *
from costar_objrec_msgs.msg import * 

class GeometryPredicator(object):

    def __init__(self, pub_topic, valid_pub_topic, value_pub_topic):
        self._listener = tf.TransformListener()
        self._publisher = rospy.Publisher(pub_topic, PredicateList, queue_size=1000)
        self._valid_publisher = rospy.Publisher(valid_pub_topic, ValidPredicates, queue_size=1000)
        self._value_publisher = rospy.Publisher(value_pub_topic, FeatureValues, queue_size=1000)
        self._detected_objects_subsriber = rospy.Subscriber('/costar/detected_object_list', DetectedObjectList, self.updateAvailableFrames)
        self._default_frames = rospy.get_param('~frames')
        self._frames = self._default_frames + []
        self._cached_frames = set(self._frames)
        self._reference_frames = rospy.get_param('~reference_frames', [])
        if len(self._reference_frames) == 0:
            self._reference_frames = self._frames
        self._value_predicates = rospy.get_param('~publish_values',0) == 1
        self._world_frame = rospy.get_param('~world_frame','world')
        self._higher_margin = rospy.get_param('~height_threshold',0.1)
        self._x_threshold = rospy.get_param('~rel_x_threshold',0.1)
        self._y_threshold = rospy.get_param('~rel_y_threshold',0.1)
        self._z_threshold = rospy.get_param('~rel_z_threshold',0.1)
        self._near_2d_threshold = rospy.get_param('~near_2D_threshold',0.2)
        self._near_3d_threshold = rospy.get_param('~near_3D_threshold',0.2)
        self._values = {}

        self._previous_frame = dict()
        self._previous_msg = dict()

    '''
    addHeightPredicates()
    Determines if some objects are higher than others.
    Can be passed a margin/threshold, in case things aren't different enough to matter.
    higher_than(A, B) means B is higher than A
    '''
    def addHeightPredicates(self, frame1, frame2):
        predicate_statement_list = list()
        try:
            (trans1, rot1) = self._listener.lookupTransform(self._world_frame, frame1, rospy.Time(0))
            (trans2, rot2) = self._listener.lookupTransform(self._world_frame, frame2, rospy.Time(0))
        
            height_difference = trans2[2] - trans1[2]

            self._values["height_difference_" + str(frame1) + "_" + str(frame2)] = height_difference

            if height_difference < -1 * self._higher_margin:
                ps = PredicateStatement()
                ps.predicate = 'higher_than'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.num_params = 2
                ps.value = height_difference
                predicate_statement_list.append(ps)
            elif height_difference < self._higher_margin:
                ps = PredicateStatement()
                ps.predicate = 'lower_than'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.num_params = 2
                ps.value = height_difference
                predicate_statement_list.append(ps)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            pass

        return predicate_statement_list

    '''
    addNearPredicates()
    Are the two frames close together in the (X, Y) world plane?
    Are the two frames close together in all (X, Y, Z) coordinates?
    '''
    def addNearPredicates(self, frame1, frame2):
        predicate_statement_list = list()
        try:
            (trans1, rot1) = self._listener.lookupTransform(self._world_frame, frame1, rospy.Time(0))
            (trans2, rot2) = self._listener.lookupTransform(self._world_frame, frame2, rospy.Time(0))

            dx = trans2[0] - trans1[0]
            dy = trans2[1] - trans1[1]
            dz = trans2[2] - trans1[2]

            dist = np.sqrt((dx*dx)+(dy*dy))
            dist_xyz = np.sqrt((dx*dx)+(dy*dy)+(dz*dz))

            self._values["distance_xy_" + str(frame1) + "_" + str(frame2)] = dist
            self._values["distance_xyz_" + str(frame1) + "_" + str(frame2)] = dist_xyz

            if dist <= self._near_2d_threshold:
                ps = PredicateStatement()
                ps.predicate = 'near_xy'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.num_params = 2
                ps.value = dist
                predicate_statement_list.append(ps)
            if dist_xyz <= self._near_3d_threshold:
                ps = PredicateStatement()
                ps.predicate = 'near_xyz'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.num_params = 2
                ps.value = dist
                predicate_statement_list.append(ps)


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

        return predicate_statement_list


    '''
    addRelativeDirectionPredicates()
    Adds predicates indicating that another object is in a certain direction from a given object.
    Also needs a frame of reference, which can be anything.
    Frames of reference include the objects involved.

    Example:
    left_of(A,B,C) means B is left from A, from the perspective of C.
    
    In the context of the peg demo, this might mean that the robot arm is to "stage left" of a peg.
    '''
    def addRelativeDirectionPredicates(self, frame1, frame2, ref):
        predicate_statement_list = list()
        try:
            (trans1, rot1) = self._listener.lookupTransform(ref, frame1, rospy.Time(0))
            (trans2, rot2) = self._listener.lookupTransform(ref, frame2, rospy.Time(0))

            x_diff = trans2[0] - trans1[0]
            y_diff = trans2[1] - trans1[1]
            z_diff = trans2[2] - trans1[2]

            self._values["x_diff_" + str(frame1) + "_" + str(frame2)] = x_diff
            self._values["y_diff_" + str(frame1) + "_" + str(frame2)] = y_diff
            self._values["z_diff_" + str(frame1) + "_" + str(frame2)] = z_diff

            if x_diff < -1 * self._x_threshold:
                ps = PredicateStatement()
                ps.predicate = 'in_front_of'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.params[2] = ref
                ps.num_params = 3
                ps.value = x_diff
                predicate_statement_list.append(ps)
            elif x_diff > self._x_threshold:
                ps = PredicateStatement()
                ps.predicate = 'in_back_of'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.params[2] = ref
                ps.num_params = 3
                ps.value = x_diff
                predicate_statement_list.append(ps)

            if y_diff < -1 * self._y_threshold:
                ps = PredicateStatement()
                ps.predicate = 'left_of'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.params[2] = ref
                ps.num_params = 3
                ps.value = y_diff
                predicate_statement_list.append(ps)
            elif y_diff > self._y_threshold:
                ps = PredicateStatement()
                ps.predicate = 'right_of'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.params[2] = ref
                ps.num_params = 3
                ps.value = y_diff
                predicate_statement_list.append(ps)

            if z_diff < -1 * self._z_threshold:
                ps = PredicateStatement()
                ps.predicate = 'up_from'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.params[2] = ref
                ps.num_params = 3
                ps.value = z_diff
                predicate_statement_list.append(ps)
            elif z_diff > self._z_threshold:
                ps = PredicateStatement()
                ps.predicate = 'down_from'
                ps.params[0] = frame1
                ps.params[1] = frame2
                ps.params[2] = ref
                ps.num_params = 3
                ps.value = z_diff
                predicate_statement_list.append(ps) 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

        return predicate_statement_list

    '''
    addDistancePredicates()
    Get the distance between two transforms
    '''
    def addDistancePredicates(self, frame1, frame2):
        predicate_statement_list = list()
        try:
            (trans1, rot1) = self._listener.lookupTransform(self._world_frame, frame1, rospy.Time(0))
            (trans2, rot2) = self._listener.lookupTransform(self._world_frame, frame2, rospy.Time(0))

            dx = trans2[0] - trans1[0]
            dy = trans2[1] - trans1[1]
            dz = trans2[2] - trans1[2]

            distance = np.sqrt((dx*dx)+(dy*dy)+(dz*dz))
            
            ps = PredicateStatement()
            ps.predicate = 'tf_distance'
            ps.params[0] = frame1
            ps.params[1] = frame2
            ps.num_params = 2
            ps.value = distance
            predicate_statement_list.append(ps) 

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass

        return predicate_statement_list

    '''
    updateAvailableFrames()
    Add detected object frames into the set of available frames to manipulate 
    '''
    def updateAvailableFrames(self, msg):
        for detected_object in msg.objects:
            self._cached_frames.add(detected_object.id)

        self._frames = list(self._cached_frames)

    '''
    getPredicateMessage()
    loop over all frames we are supposed to examine
    determine if there are any interesting relationships between them
    '''
    def getPredicateMessage(self):

        msg = PredicateList()
        msg.pheader.source = rospy.get_name()

        for frame1 in self._frames:
            for frame2 in self._frames:
                if frame1 == frame2:
                    continue
                else:
                    try:
                        cur_frame = pm.fromTf(self._listener.lookupTransform(frame1, frame2, rospy.Time(0)))
                        f12_msg = None
                        prev_frame = None
                        if (frame1, frame2) in self._previous_frame:
                            prev_frame = self._previous_frame[(frame1,frame2)]

                        if prev_frame == cur_frame:
                            f12_msg = self._previous_msg[(frame1,frame2)]
                        else:
                            f12_msg = list()
                            # check height predicates
                            f12_msg += self.addHeightPredicates(frame1, frame2)
                            f12_msg += self.addNearPredicates(frame1, frame2)
                            if self._value_predicates:
                                f12_msg += self.addDistancePredicates(frame1, frame2)

                            for ref in self._reference_frames:
                                f12_msg += self.addRelativeDirectionPredicates(frame1, frame2, ref)

                            self._previous_frame[(frame1,frame2)] = cur_frame
                            self._previous_msg[(frame1,frame2)] = f12_msg

                        if f12_msg is not None:
                            msg.statements = msg.statements + f12_msg
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): pass
        #print self._frames
        #print self._reference_frames
        #print msg
        return msg

    '''
    getValidPredicatesMessage()
    return information about what possible things this can publish
    '''
    def getValidPredicatesMessage(self):
        msg = ValidPredicates()
        msg.pheader.source = rospy.get_name()
        msg.predicates = ['left_of', 'right_of', 'up_from',
                'down_from',
                'in_front_of',
                'in_back_of',
                'near_xy',
                'near_xyz',
                'higher_than',
                'lower_than']
        msg.predicate_length = [3, 3, 3,
                3,
                3,
                3,
                2,
                2,
                2,
                2]
        msg.value_predicates = ['tf_distance']
        msg.assignments = [frame for frame in self._frames]
        msg.assignments.append(self._world_frame)

        return msg

    '''
    getFeatureValues()
    returns most recent updated set of values
    '''
    def getValuesMessage(self):
        msg = FeatureValues()
        msg.pheader.source = rospy.get_name()

        for k, v in self._values.iteritems():
            msg.name.append(k)
            msg.value.append(v)

        return msg

    def publishValues(self, msg):
        self._value_publisher.publish(msg)

    def publishValid(self, msg):
        self._valid_publisher.publish(msg)

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

    rospy.sleep(1.0)
    print "starting geometry node"

    try:

        gp = GeometryPredicator('/predicator/input', '/predicator/valid_input', '/predicator/values')

        while not rospy.is_shutdown():
            msg = gp.getPredicateMessage()
            gp.publish(msg)

            valid_msg = gp.getValidPredicatesMessage()
            gp.publishValid(valid_msg)

            value_msg = gp.getValuesMessage()
            gp.publishValues(value_msg)

            rate.sleep()

    except rospy.ROSInterruptException: pass
