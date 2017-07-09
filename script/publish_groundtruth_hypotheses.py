#!/usr/bin/env python

import rospy
from costar_objrec_msgs.msg import DetectedObjectList
from costar_objrec_msgs.msg import DetectedObject
from costar_objrec_msgs.msg import ObjectSymmetry

from objrec_hypothesis_msgs.msg import AllModelHypothesis
from objrec_hypothesis_msgs.msg import ModelHypothesis
from objrec_hypothesis_msgs.msg import Hypothesis
from sequential_scene_parsing.srv import hypothesis_request

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from sensor_msgs.msg import PointCloud2

import tf
# from std_srvs.srv import Empty
from string import digits

import csv
import os, sys, getopt

import numpy as np
import PyKDL as kdl

deg_to_rad = 0.0174533

def cache_point_cloud(data):
	global point_cloud_data
	point_cloud_data = data

def hypothesis_generator(current_frame_tf_accumulator):
	message_to_publish = AllModelHypothesis()
	object_hypothesis_dict = dict()
	# ModelHypothesis()	

	for transform_list_information in current_frame_tf_accumulator:
		obj_frame_name, ref_frame, (x,y,z), (qx,qy,qz,qw), object_type, ransac_confidence = transform_list_information
		if ransac_confidence < 0.1:
			continue

		if obj_frame_name not in object_hypothesis_dict:
			object_hypothesis_dict[obj_frame_name] = ModelHypothesis(tf_name=obj_frame_name, model_name=object_type)

		# generate 15 hypotheses with noise depending on the ransac confidence
		# ransac confidence
		# > 0.4: error max 2 mm; 2 degrees in all axes
		# > 0.3: error max 4 mm; 4 degrees in all axes
		# > 0.2: error max 8 mm; 8 degrees in all axes
		for i in range(15):
			tmp_hypothesis = Hypothesis(model_name=object_type, match=0, confidence=float(ransac_confidence))
			position_error = 0
			rotation_error = 0
			if ransac_confidence > 0.4:
				position_error = 3e-3
				rotation_error = 3 * deg_to_rad
			elif ransac_confidence > 0.3:
				position_error = 6e-3
				rotation_error = 6 * deg_to_rad
			elif ransac_confidence > 0.2:
				position_error = 9e-3
				rotation_error = 9 * deg_to_rad
			else:
				position_error = 12e-3
				rotation_error = 12 * deg_to_rad

			position_with_noise = np.array((x,y,z)) + position_error * np.random.rand(3)
			rot_error = rotation_error * np.random.rand(3)

			q = kdl.Rotation.Quaternion(qx,qy,qz,qw) * kdl.Rotation.EulerZYX(*rot_error)
			# q = np.array().astype(np.float)
			input_q = [float(q_i) for q_i in q.GetQuaternion()]
			# print Quaternion(*input_q)
			tmp_hypothesis.transform = Transform(translation=Vector3(*position_with_noise), rotation=Quaternion(*input_q))

			object_hypothesis_dict[obj_frame_name].model_hypothesis.append(tmp_hypothesis)

	for key in object_hypothesis_dict:
		message_to_publish.all_hypothesis.append(object_hypothesis_dict[key])

	return message_to_publish

def publish_detected_object_msgs(req):
	global detected_object_in_frame

	input_frame_num = req.frame_num
	if len(detected_object_in_frame) < input_frame_num:
		print 'Error, the frame number request (%d) > number of available frame(%d)'%(input_frame_num,len(detected_object_in_frame))
	else:
		global frame_to_publish
		frame_to_publish = input_frame_num

		global seg_cloud_pub
		seg_cloud_pub.publish(point_cloud_data)
		rospy.sleep(0.3)

		global seq
		global det_obj_pub
		obj_list_to_publish, current_frame_tf_accumulator = detected_object_in_frame[frame_to_publish]
		obj_list_to_publish.header.seq = seq
		obj_list_to_publish.header.stamp = rospy.Time.now()

		det_obj_pub.publish(obj_list_to_publish)
		rospy.sleep(0.3)

		global hypo_obj_pub
		hypo_obj_pub.publish(hypothesis_generator(current_frame_tf_accumulator))

		seq += 1
	return []

def main(file_path):
	if not os.path.isfile(file_path):
		print 'Input file [%s] is not a valid file.'%file_path
	file_groundtruth_csv = open(file_path, 'r')
	groundtruth_tf_list = csv.reader(file_groundtruth_csv, delimiter=',')
	
	ref_frame = 'camera_link'

	global detected_object_in_frame
	detected_object_in_frame = list()
	current_frame_tf_accumulator = list()
	current_frame_list_of_objects = DetectedObjectList()
	current_frame = 0

	obj_symmetry_dict = dict()
	obj_symmetry_dict['block'] = ObjectSymmetry(x_symmetries=4,x_rotation=90*deg_to_rad,
		y_symmetries = 2, y_rotation = 180*deg_to_rad,
		z_symmetries = 2, z_rotation = 180*deg_to_rad)

	obj_symmetry_dict['cube'] = ObjectSymmetry(x_symmetries=4,x_rotation=90*deg_to_rad,
		y_symmetries = 4, y_rotation = 90*deg_to_rad,
		z_symmetries = 4, z_rotation = 90*deg_to_rad)
	
	global seq 
	seq = 0

	for row in groundtruth_tf_list:
		# print len(row)
		if len(row) == 10:
			frame_num, obj_name, x, y, z, qx, qy, qz, qw, ransac_confidence = row
			if frame_num == 'Frame':
				continue
			elif current_frame != int(frame_num):
				print 'Frame #%d'%current_frame, len(current_frame_tf_accumulator)
				# current_frame_list_of_objects.header.seq = current_frame
				current_frame_list_of_objects.header.frame_id = ref_frame
				
				## NEED TO ADD THE TIME STAMP WHEN PUBLISHING

				detected_object_in_frame.append((current_frame_list_of_objects,current_frame_tf_accumulator))
				current_frame_tf_accumulator = []
				current_frame_list_of_objects = DetectedObjectList()
				current_frame = int(frame_num)

			ransac_confidence = float(ransac_confidence)
			
			# Add detected object
			det_object = DetectedObject()
			object_type = obj_name.translate(None, digits)
			obj_index = 0
			if object_type == obj_name:
				obj_index = 1
			else:
				obj_index = obj_name[len(object_type):]

			det_object.id = "objects/%s/%s"%(object_type,obj_index)
			det_object.object_class = object_type
			det_object.symmetry = obj_symmetry_dict[object_type]

			if ransac_confidence > 0.1:
				# skip publishing object with very low confidence
				current_frame_list_of_objects.objects.append(det_object)

			# Add frame information
			p = np.array((x,y,z)).astype(np.float16)
			q = np.array((qx,qy,qz,qw)).astype(np.float)
			q /= np.linalg.norm(q)
			rot = kdl.Rotation.Quaternion(*q)
			transform_to_add = [det_object.id, ref_frame, p, rot.GetQuaternion(), object_type, ransac_confidence]
			current_frame_tf_accumulator.append(transform_to_add)

	if len(current_frame_tf_accumulator) > 0:
		print 'Frame #%d'%current_frame, len(current_frame_tf_accumulator)
		# current_frame_list_of_objects.header.seq = current_frame
		current_frame_list_of_objects.header.frame_id = ref_frame
		
		## NEED TO ADD THE TIME STAMP WHEN PUBLISHING

		detected_object_in_frame.append((current_frame_list_of_objects,current_frame_tf_accumulator))
		current_frame_tf_accumulator = []
		current_frame_list_of_objects = DetectedObjectList()
		current_frame = int(frame_num)

	rospy.init_node('groundtruth_h_generator')
	object_pub_service = rospy.Service('pub_objects', hypothesis_request, publish_detected_object_msgs)
	
	global det_obj_pub
	det_obj_pub = rospy.Publisher('/SPServer/detected_object_list', DetectedObjectList,queue_size=1)

	global hypo_obj_pub
	hypo_obj_pub = rospy.Publisher('/SPServer/object_hypothesis', AllModelHypothesis, queue_size=1) 

	global seg_cloud_pub
	seg_cloud_pub = rospy.Publisher('/SPServer/points_out',PointCloud2,queue_size=1)
	
	rospy.Subscriber('/camera/depth_registered/points', PointCloud2, cache_point_cloud)

	global frame_to_publish
	frame_to_publish = 0

	rate = rospy.Rate(10)
	br = tf.TransformBroadcaster()

	while not rospy.is_shutdown():
		if len(detected_object_in_frame) > frame_to_publish:
			_, current_frame_tfs = detected_object_in_frame[frame_to_publish]
			for transform in current_frame_tfs:
				obj_frame_name, parent_frame, p, q, _, _ = transform 
				br.sendTransform(p, q, rospy.Time.now(), obj_frame_name, parent_frame)
		rate.sleep()

if __name__ == "__main__":
	print len(sys.argv), sys.argv
	if len(sys.argv) != 2:
		print 'This script needs 2 arguments.'
	else:
		main(sys.argv[1])
