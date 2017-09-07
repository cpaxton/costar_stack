#!/usr/bin/env python

import rospy
import tf
from costar_objrec_msgs.msg import *
import numpy as np
import threading

# mtx = None
# broadcaster = None 
# listener = None

# republish_frame_list = []

def detected_objects_cb(msg):
	rospy.loginfo("Received detected object messages")
	global republish_frame_list
	# global mtx

	# mtx.acquire()

	for obj in msg.objects:
		frame_id = obj.id.split('/')[-1]
		
		republish_frame_list.append((obj.id,frame_id))

	# mtx.release()
		

def frame_broadcaster():
	global republish_frame_list
	global broadcaster
	# global mtx

	# mtx.acquire()
	for obj_id, renamed_frame_id in republish_frame_list:
		broadcaster.sendTransform((0.,0.,0.),tf.transformations.quaternion_from_euler(0.,0.,0.),
			rospy.Time.now(),
			renamed_frame_id,
			obj_id)
	# mtx.release()
	
if __name__ == '__main__':
	rospy.init_node('detected_obj_republisher')
	rospy.loginfo("Detected object republisher node is run")
	
	use_scene_parsing_pose = False
	if rospy.has_param("/costar/smartmove/scene_parsing_pose"):
		use_scene_parsing_pose = rospy.get_param("/costar/smartmove/scene_parsing_pose")
	source_namespace = ""

	if use_scene_parsing_pose:
		rospy.loginfo("Using scene parsing pose")
		source_namespace = rospy.get_param("/costar/smartmove/scene_parsing_namespace")
	else:
		rospy.loginfo("Using sp_segmenter pose")
		source_namespace = "costar_sp_segmenter"

	global listener
	global broadcaster
	global republish_frame_list
	republish_frame_list = []
	listener = tf.TransformListener()
	broadcaster = tf.TransformBroadcaster()
	
	# global mtx
	# mtx = threading.Lock()

	detected_objects = rospy.Subscriber(source_namespace+'/detected_object_list', DetectedObjectList, detected_objects_cb)
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		frame_broadcaster();
		r.sleep()
