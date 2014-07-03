#!/usr/bin/env python

import rospy
from predicator_msgs.msg import *
from predicator_core.srv import *

import sets

'''
return a key based on a predicate
'''
def get_key(predicate, params):
    return "(%s,%s,%s,%s)"%(predicate,
            params[0],
            params[1],
            params[2])

class PredicatorParams(object):

    def __init__(self):
        self.subscriber_ = rospy.Subscriber('predicator/update_param', UpdateParam, self.callback)
        self.publisher_ = rospy.Publisher('predicator/input', PredicateList)
        self.valid_publisher_ = rospy.Publisher('predicator/valid_input', ValidPredicates)

        self.params_ = {}

    '''
    callback()
    update the set of parameters we have stored to predicator
    '''
    def callback(self, msg):

        key = get_key(msg.statement.predicate, msg.statement.params)

        if msg.operation == UpdateParam.PUBLISH_PREDICATE:
            print "Adding parameter with key %s"%(key)
            self.params_[key] = msg.statement
        elif msg.operation == UpdateParam.REMOVE_PREDICATE and key in self.params_:
            print "Removing parameter with key %s"%(key)
            del self.params_[key]

    '''
    publish()
    stores and sends out 
    '''
    def publish(self):
        msg = PredicateList()
        valid_msg = ValidPredicates()
        msg.header.frame_id = rospy.get_name()
        valid_msg.header.frame_id = rospy.get_name()

        unique_preds = sets.Set()
        unique_params = sets.Set()

        for key, pred in self.params_.items():
            unique_preds.add(pred.predicate)
            for i in range(pred.num_params):
                unique_params.add(pred.params[i])
            msg.statements.append(pred)

        self.publisher_.publish(msg)

        valid_msg.predicates = [pred for pred in unique_preds]
        valid_msg.assignments = [param for param in unique_params]

        self.valid_publisher_.publish(valid_msg)

'''
main loop just runs the predicator node
'''
if __name__ == "__main__":
    rospy.init_node('predicator_params_node')
    spin_rate = rospy.get_param('rate',10)
    rate = rospy.Rate(spin_rate)

    try:

        pp = PredicatorParams()

        while not rospy.is_shutdown():
            pp.publish()
            rate.sleep()

    except rospy.ROSInterruptException: pass

