#!/usr/bin/env python

import rospy
from costar_robot import WaypointManager

'''
Offer up a list of services to access possible waypoints and joint state waypoints.
'''
def waypoint_manager_service(req):
  pass

if __name__ == '__main__':
  rospy.init_node('costar_waypoint_manager')
  wm = WaypointManager()

  rate = rospy.Rate(60)

  while not rospy.is_shutdown():
    rate.sleep()
