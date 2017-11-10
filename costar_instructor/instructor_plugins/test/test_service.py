#!/usr/bin/env python
import instructor_plugins
from instructor_plugins.srv import *
import rospy
# Test Service -----------------------------------------------------------------
def run_service():
    rospy.init_node('instructor_test_service')
    test_service = rospy.Service('instructor_plugins/TestService', instructor_plugins.srv.TestService, handle)
    rospy.loginfo('Test Service started')
    rospy.spin()

def handle(req):
  rospy.loginfo('Test Service called')
  rospy.sleep(1)
  rospy.loginfo('Test Service did something')
  return req.val

if __name__ == "__main__":
    run_service()