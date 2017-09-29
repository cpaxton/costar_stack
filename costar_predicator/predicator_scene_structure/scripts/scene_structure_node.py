#!/usr/bin/env python

import rospy
from predicator_scene_structure import SceneStructurePublisher

print "Starting node..."
rospy.init_node('predicator_scene_structure')

print "Creating publisher..."
dop = SceneStructurePublisher()

rospy.spin()
