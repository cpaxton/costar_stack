#!/usr/bin/env python

import rospy
from costar_structure_msgs.msg import *

rospy.init_node('publish_fake_scene_structures')

ls = SceneGraph()
ls.structure = []
ls.base_objects_id = ['fake_object_1','fake_object_2','fake_object_3']

ls.structure.append(StructureGraph())

ls.structure.append(
		StructureGraph(nodes_level=[
				SceneNodes(object_names=["fake_s_object_1"]),
				SceneNodes(object_names=["fake_s_object_2"])
			])
	)

ls.structure.append(
		StructureGraph(nodes_level=[
				SceneNodes(object_names=["fake_s_object_3","fake_s_object_4"]),
				SceneNodes(object_names=["fake_s_object_5"]),
				SceneNodes(object_names=["fake_s_object_6"])
			])
	)

rate = rospy.Rate(10)
pub = rospy.Publisher("scene_structure_list",SceneGraph,queue_size=1000)

rospy.loginfo("Sending fake scene structures...")

try:
    while not rospy.is_shutdown():
        pub.publish(ls)

        rate.sleep()
except rospy.ROSInterruptException, e:
    pass

