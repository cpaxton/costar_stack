#!/usr/bin/env python

import rospy
import rosbag
from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from sensor_msgs.msg import PointCloud2
import time
from subprocess import Popen

pts = None
topic = '/camera/depth_registered/points'
num_recorded = 0
obj_id = 'unknown'
max_count = 30

#def cb(msg):
#    global pts
#    pts = msg

def record(req):

    global pts
    global num_recorded

    time_str = time.strftime("%Y_%m_%d_%H_%M_%S")
    print time_str

    filename = '%s_iter_%d_%s.bag'%(time_str,num_recorded,obj_id)
    test_proc = Popen(["rosbag","record","/camera/.*","/tf","/joint_states","--duration=5", "--size=150", "-e", "--output-name=%s"%(filename)])


    rospy.sleep(15.0);
    num_recorded += 1

    rospy.loginfo('Recorded example %d of class "%s"'%(num_recorded,obj_id))

    test_proc.terminate()

    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('point_cloud_recorder_node')
    obj_id = rospy.get_param('~id','unknown')
    #sub = rospy.Subscriber(topic,PointCloud2,cb)
    s = rospy.Service('record_camera', Empty, record)
    rospy.loginfo("Ready to record!")
    rospy.spin()

    
