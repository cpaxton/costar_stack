#!/usr/bin/env python

import rospy
import tf
from costar_waypoint_manager import WaypointManager

'''
Offer up a list of services to access possible waypoints and joint state waypoints.
'''
def waypoint_manager_service(req):
  pass

if __name__ == '__main__':
  rospy.init_node('costar_waypoint_manager')

  broadcaster = tf.TransformBroadcaster()

  # Note that while the waypoint manager is currently a part of CostarArm
  # If we wanted to set this up for multiple robots it should be treated
  # as an independent component.
  wm = WaypointManager(service=True,
          broadcaster=broadcaster)

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()

    # publish TF messages to display frames
    wm.publish_tf()
