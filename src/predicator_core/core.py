#!/usr/bin/env python

import rospy
import copy
from predicator_msgs.msg import *
from predicator_core.srv import *

def predicate_to_tuple(predicate):
    if predicate.num_params == 1:
        return (predicate.predicate, predicate.params[0])
    elif predicate.num_params == 2:
        return (predicate.predicate, predicate.params[0], predicate.params[1])
    elif predicate.num_params == 3:
        return (predicate.predicate, predicate.params[0], predicate.params[1], predicate.params[2])
    else:
        return (predicate.predicate)

'''
Predicator()
Class containing functions to process and access the different predicator functions.
Aggregates lists of predicates arriving on a list topic, and publishes them.
'''
class Predicator(object):

    def __init__(self, sub_topic, pub_topic):
        self._subscriber = rospy.Subscriber(sub_topic, PredicateList, self.callback)
        self._publisher = rospy.Publisher(pub_topic, PredicateSet)
        self._testService = rospy.Service('predicator/test_predicate',TestPredicate, self.test_predicate)
        self._getService = rospy.Service('predicator/get_assignment',GetAssignment, self.get_assignment)
        self._latest = {}

    def callback(self, msg):
        self._latest[msg.header.frame_id] = msg.statements

    def aggregate(self):
        print self._latest
        d = {}
        for source, lst in self._latest.items():
            print "---"
            for predicate in lst:
                pred = predicate_to_tuple(predicate)
                print pred
                print "---"
                d[pred] = []
                '''
                for j in range(predicate.num_params):
                    gen_predicate = copy.deepcopy(predicate)
                    gen_predicate.params[j] = '*'
                    if not gen_predicate in d:
                        d[gen_predicate] = []
                    d[gen_predicate].append(predicate.params[j])
                '''

        print '++++++'
        print d

    def get_assignment(self, req):
        pass

    def test_predicate(self, req):
        pass

if __name__ == '__main__':
    rospy.init_node('predicator_core')
    
    spin_rate = rospy.get_param('rate',10)

    rate = rospy.Rate(spin_rate)

    try:

        pc = Predicator('predicator/input', 'predicator/output')

        while not rospy.is_shutdown():
            pc.aggregate()
            rate.sleep()

    except rospy.ROSInterruptException: pass
