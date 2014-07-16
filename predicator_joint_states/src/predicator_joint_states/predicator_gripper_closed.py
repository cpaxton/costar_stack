#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *
from predicator_msgs.msg import *

js = []

def cb(msg):
    #print msg.position
    js = [x for x in msg.position]

    msg = PredicateList()
    msg.pheader.source = rospy.get_name()
    ps = PredicateStatement()
    ps.predicate = 'is_closed'
    ps.num_params = 1
    ps.params = [config['id'],'','']
    msg.statements.append(ps)


    if len(js) > 0 and js[idx] < threshold:
        #print js[idx]
        #print threshold
        pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("predicator_joints_node")
    pub = rospy.Publisher('predicator/input', PredicateList)
    vpub = rospy.Publisher('predicator/valid_input', ValidPredicates)

    rate = rospy.Rate(30)
    
    # read in some information
    config = rospy.get_param('~js_config')
    print config

    sub = rospy.Subscriber(config['topic'], JointState, cb)
    threshold = float(config['thresholds'])
    idx = int(config['idx'])

    while not rospy.is_shutdown():

        # publish predicates message
        # publish


        rate.sleep()


