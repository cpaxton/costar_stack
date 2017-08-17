#!/usr/bin/env python

import rospy
import rospkg

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

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty

import python_pcd

import tf
from string import digits

import csv
import os, sys, getopt

import numpy as np
import PyKDL as kdl

deg_to_rad = 0.0174533

def quaternion_angle(q_1,q_2):
	np_q1 = np.array([float(q_i) for q_i in q_1])
	np_q2 = np.array([float(q_i) for q_i in q_2])
	return np.arccos(2. * (np_q1.dot(np_q2))**2 - 1)

class GroundtruthHypothesisGenerator(object):
	def __init__(self,folder_name):
		self.point_cloud_data = None
		self.seq = 0
		package_dir = rospkg.RosPack().get_path('costar_objects')
		test_dir_path = os.path.join(package_dir,'data',folder_name)
		if not os.path.exists(test_dir_path):
			print 'Directory: %s does not exist.'%test_dir_path
			# return

		filename = 'object_poses_rel_to_camera.csv'
		file_path = os.path.join(test_dir_path,filename)	
		if not os.path.isfile(file_path):
			print 'Input file [%s] is not a valid file.'%file_path
			# return

		self.filewriter = open(os.path.join(test_dir_path,'scene_parsing_result.csv'), 'w')
		self.writer = csv.writer(self.filewriter, delimiter='\t', quoting=csv.QUOTE_NONE)
		self.writer.writerow(['Frame','Object Name','dx','dy','dz','dq.x','dq.y','dq.z','dq.w', 'rot_error',
			'n_x_min','n_y_min','n_z_min',
			'n_x_max','n_y_max','n_z_max',
			'n_x_mean','n_y_mean','n_z_mean',
			'n_x_var','n_y_var','n_z_var',
			'n_rot_min','n_rot_max','n_rot_mean','n_rot_var'])
		
		file_groundtruth_csv = open(file_path, 'r')
		groundtruth_tf_list = csv.reader(file_groundtruth_csv, delimiter='\t')
		ref_frame = 'camera_link'

		self.detected_object_in_frame = list()
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

		obj_symmetry_dict['other'] = ObjectSymmetry(x_symmetries=1,x_rotation=360*deg_to_rad,
			y_symmetries = 1, y_rotation = 360*deg_to_rad,
			z_symmetries = 1, z_rotation = 360*deg_to_rad)
		
		self.object_id_by_frame_ = dict()
		objects_in_one_frame = []
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

					self.detected_object_in_frame.append((current_frame_list_of_objects,current_frame_tf_accumulator))
					self.object_id_by_frame_[current_frame] = objects_in_one_frame
					current_frame_tf_accumulator = []
					current_frame_list_of_objects = DetectedObjectList()
					current_frame = int(frame_num)

				ransac_confidence = float(ransac_confidence)
				
				# Add detected object
				det_object = DetectedObject()
				object_type = obj_name.translate(None, digits)
				obj_index = 0
				if object_type != obj_name:
					obj_index = obj_name[len(object_type):]

				det_object.id = "objects/%s/%s"%(object_type,obj_index)
				det_object.object_class = object_type

				objects_in_one_frame.append(det_object.id)

				if object_type in obj_symmetry_dict:
					det_object.symmetry = obj_symmetry_dict[object_type]
				else:
					det_object.symmetry = obj_symmetry_dict['other']

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
			current_frame_list_of_objects.header.frame_id = ref_frame

			self.detected_object_in_frame.append((current_frame_list_of_objects,current_frame_tf_accumulator))
			self.object_id_by_frame_[current_frame] = objects_in_one_frame
			current_frame_tf_accumulator = []
			current_frame_list_of_objects = DetectedObjectList()
			current_frame = int(frame_num)

		self.saved_cloud = list()
		cloud_dir = os.path.join(test_dir_path,'seg_cloud')

		for frame_idx in range(len(self.detected_object_in_frame)):
			cloud_name = os.path.join(cloud_dir,"seg_cloud_%05d.pcd"%frame_idx)
			print cloud_name
			# cache the point cloud data to publish
			self.saved_cloud.append(python_pcd.read_pcd(cloud_name,
				cloud_header='camera_link',get_tf=False))
		self.frame_to_publish = 0

	def cache_point_cloud(self,data):
		self.point_cloud_data = data

	def hypothesis_generator(self,current_frame_tf_accumulator):
		message_to_publish = AllModelHypothesis()
		object_hypothesis_dict = dict()
		self.current_hypothesis_noise = dict()
		induced_noise = dict()
		# ModelHypothesis()	

		for transform_list_information in current_frame_tf_accumulator:
			obj_frame_name, ref_frame, (x,y,z), (qx,qy,qz,qw), object_type, ransac_confidence = transform_list_information
			if ransac_confidence < 0.1:
				continue

			if obj_frame_name not in object_hypothesis_dict:
				object_hypothesis_dict[obj_frame_name] = ModelHypothesis(tf_name=obj_frame_name, model_name=object_type)
				induced_noise[obj_frame_name] = list()
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

				position_noise = position_error * np.random.rand(3)
				position_with_noise = np.array((x,y,z)) + position_noise
				rot_noise = rotation_error * np.random.rand(3)
				q_rot_noise = kdl.Rotation.EulerZYX(*rot_noise).GetQuaternion()
				rot_noise_angle = quaternion_angle([0,0,0,1],q_rot_noise)
				induced_noise[obj_frame_name].append(position_noise.tolist() + [rot_noise_angle])

				q = kdl.Rotation.Quaternion(qx,qy,qz,qw) * kdl.Rotation.EulerZYX(*rot_noise)
				# q = np.array().astype(np.float)
				input_q = [float(q_i) for q_i in q.GetQuaternion()]
				# print Quaternion(*input_q)
				tmp_hypothesis.transform = Transform(translation=Vector3(*position_with_noise), rotation=Quaternion(*input_q))

				object_hypothesis_dict[obj_frame_name].model_hypothesis.append(tmp_hypothesis)

		for key in object_hypothesis_dict:
			message_to_publish.all_hypothesis.append(object_hypothesis_dict[key])

		for obj_frame_name in induced_noise:
			data = np.array(induced_noise[obj_frame_name])
			pos_data = data[:,:3]
			r_noise_data = data[:,3]
			pos_error_min = np.amin(pos_data,axis=0).tolist()
			pos_error_max = np.amax(pos_data,axis=0).tolist()
			pos_error_avg = np.mean(pos_data,axis=0).tolist()
			pos_error_var = np.var(pos_data,axis=0).tolist()
			r_noise_profile = [np.amin(r_noise_data),np.amax(r_noise_data),np.mean(r_noise_data),np.var(r_noise_data)]
			self.current_hypothesis_noise[obj_frame_name] = pos_error_min+pos_error_max+pos_error_avg+pos_error_var+r_noise_profile

		return message_to_publish

	def publish_detected_object_msgs(self,req):
		input_frame_num = req.frame_num
		if len(self.detected_object_in_frame) < input_frame_num:
			print 'Error, the frame number request (%d) > number of available frame(%d)'%(input_frame_num,len(self.detected_object_in_frame))
		else:
			self.frame_to_publish = input_frame_num

			point_cloud_data =  self.saved_cloud[self.frame_to_publish]
			point_cloud_data.header = Header(seq=self.seq, stamp=rospy.Time.now(),frame_id='camera_link')

			self.seg_cloud_pub.publish(point_cloud_data)
			# print "Cloud published"
			rospy.sleep(0.3)

			obj_list_to_publish, current_frame_tf_accumulator = self.detected_object_in_frame[self.frame_to_publish]
			obj_list_to_publish.header.seq = self.seq
			obj_list_to_publish.header.stamp = rospy.Time.now()

			self.det_obj_pub.publish(obj_list_to_publish)
			# print "Obj list published"
			rospy.sleep(0.3)

			# self.hypo_obj_pub.publish(self.hypothesis_generator(current_frame_tf_accumulator))
			# print "all data published"
			self.seq += 1
		return []

	def evaluate_scene_result(self,data):
		obj_list_to_publish, current_frame_tf_accumulator = self.detected_object_in_frame[self.frame_to_publish]
		available_objects = self.object_id_by_frame_[self.frame_to_publish]
		for detected_object in available_objects: # obj_list_to_publish.objects:
			# object_id = detected_object.id
			object_id = detected_object
			result_id = 'scene/'+object_id
			transform = [-1]*7
			self.listener.waitForTransform(object_id,result_id,rospy.Time(), rospy.Duration(1.0))
			self.listener.canTransform(object_id,result_id,rospy.Time(0))
			try:
				(trans,rot) = self.listener.lookupTransform(object_id,result_id,rospy.Time(0))
				rot_error = quaternion_angle([0,0,0,1],rot)
				transform = np.array(trans).tolist() + np.array(rot).tolist()
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print "Fail to get tf %s to %s"%(object_id,result_id)

			noise_data = ['n/a']*16
			if object_id in self.current_hypothesis_noise:
				noise_data = self.current_hypothesis_noise[object_id]
			self.writer.writerow([self.frame_to_publish,object_id] + transform + [rot_error] + noise_data)

		print "Scene Parsing Performace had been evaluated"

		return []

	def main(self):
		rospy.init_node('groundtruth_h_generator')
		object_pub_service = rospy.Service('pub_objects', hypothesis_request, self.publish_detected_object_msgs)
		
		rospy.Subscriber('/sequential_scene_ros/done_hypothesis_msg', Empty, self.evaluate_scene_result)

		self.det_obj_pub = rospy.Publisher('/SPServer/detected_object_list', DetectedObjectList,queue_size=1)
		self.hypo_obj_pub = rospy.Publisher('/SPServer/object_hypothesis', AllModelHypothesis, queue_size=1) 
		self.seg_cloud_pub = rospy.Publisher('/SPServer/points_out',PointCloud2,queue_size=1)
		
		# rospy.Subscriber('/camera/depth_registered/points', PointCloud2, cache_point_cloud)

		rate = rospy.Rate(10)
		br = tf.TransformBroadcaster()

		self.listener = tf.TransformListener()

		while not rospy.is_shutdown():
			if len(self.detected_object_in_frame) > self.frame_to_publish:
				_, current_frame_tfs = self.detected_object_in_frame[self.frame_to_publish]
				for transform in current_frame_tfs:
					obj_frame_name, parent_frame, p, q, _, _ = transform 
					br.sendTransform(p, q, rospy.Time.now(), obj_frame_name, parent_frame)
			rate.sleep()

if __name__ == "__main__":
	print len(sys.argv), sys.argv
	if len(sys.argv) != 2:
		print 'This script needs 2 arguments.'
	else:
		generator = GroundtruthHypothesisGenerator(sys.argv[1])
		generator.main()
