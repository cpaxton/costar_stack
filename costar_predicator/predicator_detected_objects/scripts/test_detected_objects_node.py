#!/usr/bin/env python

import rospy
from costar_objrec_msgs.msg import *

rospy.init_node('publish_fake_object_detections')

ls = DetectedObjectList()
#obj1 = DetectedObject(id="gbeam_link_1/gbeam_link",object_class="link")
#obj2 = DetectedObject(id="gbeam_node_1/gbeam_node",object_class="node")
#obj3 = DetectedObject(id="gbeam_node_2/gbeam_node",object_class="node")
obj1 = DetectedObject(id="Obj::link_uniform::1",object_class="link_uniform")
obj2 = DetectedObject(id="Obj::node_uniform::1",object_class="node_uniform")
obj3 = DetectedObject(id="Obj::node_uniform::2",object_class="node_uniform")

ls.objects = [obj1,obj2,obj3]

print ls

rate = rospy.Rate(10)
pub = rospy.Publisher("detected_object_list",DetectedObjectList,queue_size=1000)

while not rospy.is_shutdown():
    pub.publish(ls)
    rate.sleep()

