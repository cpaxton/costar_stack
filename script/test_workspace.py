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
	pub = rospy.Publisher('/SPServer/detected_object_list/', DetectedObjectList,queue_size=1)
	global seq
	seq = 0
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		br = tf.TransformBroadcaster()
		br.sendTransform((0.0856636, -0.158265, 0.999503),
							(0.89781, -0.434588, 0.0148354, 0.0696412),
							rospy.Time.now(),
							'objects/link_uniform/1',
							"world")
		br.sendTransform((0.248011, 0.0903798, 0.91842),
							(0.946214, 0.105377, 0.299076, -0.0642442),
							rospy.Time.now(),
							'objects/node_uniform/1',
							"world")
		br.sendTransform((-0.137252, 0.0867218, 0.939421),
							(0.890365, 0.321198, -0.290753, -0.139804),
							rospy.Time.now(),
							'objects/node_uniform/2',
							"world")
		br.sendTransform((-0.0249532, 0.0211767, 1.01974),
							(0.761211, 0.427905, 0.232149, 0.42844),
							rospy.Time.now(),
							'objects/link_uniform/2',
							"world")
		br.sendTransform((0.255402, -0.0097534, 0.83862),
							(0.850925, -0.37827, -0.192071, -0.309754),
							rospy.Time.now(),
							'objects/link_uniform/3',
							"world")