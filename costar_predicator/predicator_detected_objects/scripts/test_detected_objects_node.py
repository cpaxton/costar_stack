#!/usr/bin/env python

import rospy
from costar_objrec_msgs.msg import *

rospy.init_node('publish_fake_object_detections')

ls = DetectedObjectList()
#obj1 = DetectedObject(id="gbeam_link_1/gbeam_link",object_class="link")
#obj2 = DetectedObject(id="gbeam_node_1/gbeam_node",object_class="node")
#obj3 = DetectedObject(id="gbeam_node_2/gbeam_node",object_class="node")
obj1 = DetectedObject(id="obj_link_uniform_1",object_class="link_uniform")
obj2 = DetectedObject(id="obj_node_uniform_1",object_class="node_uniform")
obj3 = DetectedObject(id="obj_node_uniform_2",object_class="node_uniform")
obj1.symmetry.x_symmetries = 4
obj1.symmetry.y_symmetries = 2
obj1.symmetry.z_symmetries = 2
obj2.symmetry.x_symmetries = 4
obj2.symmetry.y_symmetries = 4
obj2.symmetry.z_symmetries = 4
obj3.symmetry.x_symmetries = 4
obj3.symmetry.y_symmetries = 4
obj3.symmetry.z_symmetries = 4

ls.objects = [obj1,obj2,obj3]

rate = rospy.Rate(10)
pub = rospy.Publisher("detected_object_list",DetectedObjectList,queue_size=1000)

while not rospy.is_shutdown():
    pub.publish(ls)
    rate.sleep()

