#!/usr/bin/env python

import rospy
# from costar_structure_msgs.msg import *
from costar_objrec_msgs.msg import DetectedObjectList
from costar_objrec_msgs.msg import DetectedObject
import tf
import csv
import numpy as np
# import PyKDL as kdl

def callback_det_objects(msg):
	global object_list_updated
	global frame_number
	# global scene_updated

	object_list = list()
	object_list_updated = True

	for obj in msg.objects:
		object_list.append(obj.id)
	
	frame_number += 1

	write_poses(object_list)

# def callback_scene_structure_list(msg):
# 	global object_list_updated
# 	global scene_updated

	
# 	object_list_updated = True
# 	write_poses_result()

def write_poses(object_list):
	# global object_list_updated
	# global scene_updated
	
	# do nothing if both data has not been received
	# if not (object_list_updated and scene_updated):
	# 	return
	# else:
	# 	object_list_updated = False
	# 	scene_updated = False

	global listener
	global frame_number
	global write_poses_result

	for obj in object_list:
		# listener.canTransform('table_frame',obj,rospy.Time(0))
		listener.waitForTransform('table_frame',obj, rospy.Time(), rospy.Duration(1.0))
		try:
			(trans,rot) = listener.lookupTransform('table_frame',obj,rospy.Time(0))

			if trans[1] > 0: # all objects are stacked at y > 0
				rpy = tf.transformations.euler_from_quaternion(rot)
				transform = (np.array(trans),np.array(rpy))
			else:
				continue
		except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print "Fail to get tf %s to %s"%('table_frame',obj)
			continue

		row = [frame_number, obj, transform[0][0], transform[0][1], transform[0][2],
				transform[1][0], transform[1][1], transform[1][2]]
		write_poses_result.writerow(row)
		print row

if __name__ == "__main__":
	rospy.init_node('test_workspace_server')
	
	# global scene_updated
	global object_list_updated
	scene_updated = False
	object_list_updated = False
	rospy.Subscriber('/costar/detected_object_list', DetectedObjectList,callback_det_objects)
	# rospy.Subscriber('/sequential_scene_parsing/scene_structure_list', SceneGraph, callback_scene_structure_list)
	
	global listener
	listener = tf.TransformListener()

	global frame_number
	frame_number = 0

	global write_poses_result
	file = open('stacked_transform.csv','ab')
	write_poses_result = csv.writer(file)
	# write_poses_result.writerow(['Frame', 'Object name', 'x', 'y', 'z', 'r', 'p', 'y'])

	rate = rospy.Rate(10)
	rospy.spin()