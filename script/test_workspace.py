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
		br.sendTransform((0.248011, 0.0903798, 0.91842),
							(-0.57472, 0.302057, 0.406916, 0.642556),
							rospy.Time.now(),
							'objects/node_uniform/1',
							"world")
		br.sendTransform((-0.137252, 0.0867218, 0.939421),
							(-0.186296, 0.374573, 0.819693, 0.391269),
							rospy.Time.now(),
							'objects/node_uniform/2',
							"world")
		br.sendTransform((0.254448, -0.00760714, 0.839513),
							(-0.198534, 0.380009, 0.757896, 0.491702),
							rospy.Time.now(),
							'objects/link_uniform/1',
							"world",)
		br.sendTransform((0.0847675, -0.154304, 1.00225),
							(-0.322911, 0.29201, 0.580116, 0.688422 ),
							rospy.Time.now(),
							'objects/link_uniform/2',
							"world")
		br.sendTransform((-0.0308435, 0.02324, 1.01637),
							( -0.208724,0.849739, 0.151435, 0.459831),
							rospy.Time.now(),
							'objects/link_uniform/3',
							"world")