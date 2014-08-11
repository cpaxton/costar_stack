#!/usr/bin/env python

import rospy
import moveit_msgs
import tf
import tf_conversions as tfc

# import predicator to let us see what's going on
from predicator_msgs.msg import *
import predicator_msgs.srv as pcs

# import moveit messages
from moveit_msgs.msg import *
from moveit_msgs.srv import *

# import transform messages
from geometry_msgs.msg import *


def flip_rotation_frame(trans, rot):
    f = tfc.fromTf((trans, rot))
    ir = f.M.Inverse()

    return tfc.toTf(tfc.Frame(ir, f.p))

'''
PredicatorReachability
Module that produces messages given some group names
Looks up the predicator groups, and takes all of their members
Uses these members to compute reachable and inverse_reachable predicates
'''
class PredicatorReachability(object):

    def __init__(self):

        self.getter = rospy.ServiceProxy('/predicator/get_possible_assignment', predicator_msgs.srv.GetTypedList)
        self.getter2 = rospy.ServiceProxy("/predicator/get_assignment", pcs.GetAssignment)
        self.verbose = rospy.get_param("~verbose", 1) == 1

        self.ids = []
        self.robots = {}

        self.tfl = tf.TransformListener()

        if self.verbose == 1:
            print "starting"

        groups = rospy.get_param("~groups")

        if self.verbose == 1:
            print groups

        for group in groups:
            ids = self.getter(group).data
            for id_ in ids:
                self.ids.append(id_)

        if self.verbose == 1:
           print self.ids

        robots = self.getter("robot").data

        # NOTE: this only runs with the IROS code for now!
        # you need the peg_assist_demo settings working
        for robot_ in robots:
            statement = PredicateStatement()
            statement.predicate = "robot_namespace"
            statement.params[1] = robot_
            statement.params[0] = "*"
            resp = self.getter2(statement)

            self.robots[robot_] = resp.values[0].params[0]

        print self.robots

    def getPredicateMessage(self):
        msg = PredicateList()
        msg.pheader.source = rospy.get_name()

        for robot, namespace in self.robots.items():

            # service to compute ik for this position
            srv = rospy.ServiceProxy(namespace + "/compute_ik", moveit_msgs.srv.GetPositionIK)

            for frame in self.ids:

                if self.verbose:
                    print "Robot: " + robot + " , target frame: " + frame + ", getting transform..."

                tf_done = False

                while not tf_done:
                    try:
                        (trans, rot) = self.tfl.lookupTransform("/world", frame, rospy.Time(0))
                        tf_done = True
                    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                        continue

                # determine if the frame can be reached
                is_reachable = self.reachable(robot, trans, rot, srv)

                # determine if the inverse of the frame can be reached
                (itrans, irot) = flip_rotation_frame(trans, rot)

                is_inv_reachable = self.reachable(robot, itrans, irot, srv)

                if is_reachable:
                    ps = PredicateStatement()
                    ps.predicate = "reachable"
                    ps.params[0] = frame
                    ps.params[1] = robot
                    ps.num_params = 2
                    ps.param_classes.append("location")
                    ps.param_classes.append("robot")
                    ps.confidence = PredicateStatement.TRUE
                    ps.value = PredicateStatement.TRUE
                    msg.statements.append(ps)

                if is_inv_reachable:
                    ps = PredicateStatement()
                    ps.predicate = "inverse_reachable"
                    ps.params[0] = frame
                    ps.params[1] = robot
                    ps.num_params = 2
                    ps.param_classes.append("location")
                    ps.param_classes.append("robot")
                    ps.confidence = PredicateStatement.TRUE
                    ps.value = PredicateStatement.TRUE
                    msg.statements.append(ps)


        return msg

    def reachable(self, robot, trans, rot, srv):
        p = geometry_msgs.msg.PoseStamped()
        p.pose.position.x = trans[0]
        p.pose.position.y = trans[1]
        p.pose.position.z = trans[2]
        p.pose.orientation.x = rot[0]
        p.pose.orientation.y = rot[1]
        p.pose.orientation.z = rot[2]
        p.pose.orientation.w = rot[3]
        p.header.frame_id = "/world"

        ik_req = moveit_msgs.msg.PositionIKRequest()
        #ik_req.robot_state.joint_state = self.js
        ik_req.avoid_collisions = True
        ik_req.timeout = rospy.Duration(3.0)
        ik_req.attempts = 1
        ik_req.group_name = "arm"
        ik_req.pose_stamped = p

        if self.verbose:
            print "Getting IK position..."
        ik_resp = srv(ik_req)

        if self.verbose:
            print "IK RESULT ERROR CODE = %d"%(ik_resp.error_code.val)

        return ik_resp.error_code.val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS

    def getValidPredicatesMessage(self):
        msg = ValidPredicates()
        msg.pheader.source = rospy.get_name()
        msg.predicates = ["reachable", "inverse_reachable"]
        msg.assignments = []

        for id_ in self.ids:
            msg.assignments.append(id_)
        
        for robot_ in self.robots:
            msg.assignments.append(robot_)

        return msg

if __name__=="__main__":
    rospy.init_node("predicator_reachability_node")

    try:

        rospy.wait_for_service('predicator/get_assignment')

        # need to wait for everything to load
        delay = rospy.get_param('load_delay', 0.5)
        spin_rate = rospy.get_param('rate', 1)

        print "loaded params, waiting for service..."

        rospy.wait_for_service('/predicator/get_possible_assignment')

        print "found service"

        if (delay > 0):
            rospy.sleep(delay)

        rate = rospy.Rate(spin_rate)
    
        pr = PredicatorReachability()
        pub = rospy.Publisher('/predicator/input', PredicateList)
        vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates)

        valid = pr.getValidPredicatesMessage()

        while not rospy.is_shutdown():
            msg = pr.getPredicateMessage()
            
            pub.publish(msg)

            rate.sleep()

    except rospy.ROSInterruptException: pass
