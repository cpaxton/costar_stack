# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 08:02:13 2015

@author: Chris
"""

import rospy
import tf
import tf_conversions.posemath as pm
from geometry_msgs.msg import PoseArray

last_pose = None

def callback(msg):
    if len(last_pose) > 0:
        last_pose = msg[0]
        
if __name__ == '__main__':
    rospy.init_node('posearray_tf_republisher')
    
    sub = rospy.Subscriber('poses_out',PoseArray,callback)

    br = tf.TransformBroadcaster()
    try:
        rate = rospy.Rate(10)    
        while not rospy.is_shutdown():
            (trans, rot) = pm.toTF(last_pose)
            br.sendTransform(trans, rot, rospy.Time.now(), 'drill', 'world')
            rate.sleep()
    except rospy.ROSInterruptException, e:
        pass