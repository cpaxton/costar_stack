#!/usr/bin/env python

import rospy
from costar_objrec_msgs.msg import DetectedObjectList
from costar_objrec_msgs.msg import DetectedObject
import tf

from std_srvs.srv import Empty

# if __name__ == "__init__":
# 	pub = 0
# 	

def publish_detected_object_msgs(req):
	list_of_objects = DetectedObjectList()
	node_object = DetectedObject()
	node_object.id = "/objects/node_uniform/1"
	node_object.object_class = "node_uniform"
	node_object2 = DetectedObject()
	node_object2.id = "/objects/node_uniform/2"
	node_object2.object_class = "node_uniform"
	link_object = DetectedObject()
	link_object.id = "/objects/link_uniform/1"
	link_object.object_class = "link_uniform"

	link_object2 = DetectedObject()
	link_object2.id = "/objects/link_uniform/2"
	link_object2.object_class = "link_uniform"

	link_object3 = DetectedObject()
	link_object3.id = "/objects/link_uniform/3"
	link_object3.object_class = "link_uniform"


	list_of_objects.objects.append(node_object)
	list_of_objects.objects.append(node_object2)
	list_of_objects.objects.append(link_object)
	list_of_objects.objects.append(link_object2)
	list_of_objects.objects.append(link_object3)


	global seq
	list_of_objects.header.seq = seq
	list_of_objects.header.frame_id = "world"
	list_of_objects.header.stamp = rospy.Time.now()
	seq += 1

	global pub
	pub.publish(list_of_objects)
	return []

if __name__ == "__main__":
	rospy.init_node('test_workspace_server')
	object_pub_service = rospy.Service('pub_objects', Empty, publish_detected_object_msgs)
	
	global pub
	pub = rospy.Publisher('/SPServer/detected_object_list', DetectedObjectList,queue_size=1)
	global seq
	seq = 0
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		br = tf.TransformBroadcaster()
		br.sendTransform((0.248603, 0.0910154, 0.918521),
							(0.41052, 0.640302, -0.607897, 0.22792),
							rospy.Time.now(),
							'objects/node_uniform/1',
							"world")
		br.sendTransform((-0.13513, 0.0850241, 0.94148),
							(0.823494, 0.357022, -0.173502, 0.405328),
							rospy.Time.now(),
							'objects/node_uniform/2',
							"world")

		br.sendTransform((0.255369, -0.010304, 0.837136),
							(0.818496, 0.369117, -0.14515, 0.415629),
							rospy.Time.now(),
							'objects/link_uniform/1',
							"world",)
		br.sendTransform((0.0882621, -0.15488, 1.002),
							( 0.591846, 0.677976, -0.317259,  0.29902),
							rospy.Time.now(),
							'objects/link_uniform/2',
							"world")
		br.sendTransform((-0.0233503, 0.0190892, 1.0213),
							( 0.42496, 0.232822, 0.423159, 0.765597),
							rospy.Time.now(),
							'objects/link_uniform/3',
							"world")