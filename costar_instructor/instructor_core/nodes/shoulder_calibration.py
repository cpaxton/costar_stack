#!/usr/bin/env python

import numpy as np

import roslib
roslib.load_manifest('instructor_core')
import rospy
import rospkg

import tf
import tf_conversions as tf_c
import PyKDL as kdl
from semi_static_transform_publisher.srv import *

class Calibration():

    def __init__(self):

        rospy.init_node('shoulder_calibration',anonymous=True)
        self.calibrated = False
        # Add listener to detect AR markers
        self.tf_listen = tf.TransformListener()
        self.tf_broadcast = tf.TransformBroadcaster()

        rospy.logwarn('CALIBRATION STARTED')
        while not rospy.is_shutdown():
            self.update()
            if self.calibrated:
                rospy.logwarn('CALIBRATION FINISHED')
                break


    def update(self):
        if not self.calibrated:
            try:
                T_camera_to_marker = tf_c.fromTf(self.tf_listen.lookupTransform('camera_link','ar_marker_0', rospy.Time(0)))
                T_base_endpoint = tf_c.fromTf(self.tf_listen.lookupTransform('base_link','endpoint', rospy.Time(0)))
                T_base_camera_current = tf_c.fromTf(self.tf_listen.lookupTransform('base_link','camera_link', rospy.Time(0)))
                T_endpoint_marker = tf_c.fromTf(self.tf_listen.lookupTransform('endpoint','endpoint_marker', rospy.Time(0)))

                T_base_camera = T_base_endpoint*T_endpoint_marker*T_camera_to_marker.Inverse()

                # Extract position and rotation for the new transformation
                xyz = tf_c.toTf(T_base_camera)[0]
                rpy = T_base_camera.M.GetRPY() 

                # Call to service ###########
                # First check to see if service exists
                try:
                    rospy.wait_for_service('/semi_static_transform_publisher/UpdateTransform')
                except rospy.ROSException as e:
                    rospy.logerr('Could not find semi-static transform service')
                    return
                # Make servo call to set pose
                try:
                    # Setup transform to send to sem-static node
                    update_transform_proxy = rospy.ServiceProxy('/semi_static_transform_publisher/UpdateTransform', UpdateTransform)
                    msg = UpdateTransformRequest()
                    msg.x, msg.y, msg.z = xyz
                    msg.roll, msg.pitch, msg.yaw = rpy

                    # Call to service
                    update_transform_proxy(msg)

                    self.calibrated = True
                except (rospy.ServiceException), e:
                    rospy.logwarn('There was a problem with the service:')
                    rospy.logwarn(e)


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(e)
                return
                

            



if __name__ == "__main__":
    c = Calibration()
