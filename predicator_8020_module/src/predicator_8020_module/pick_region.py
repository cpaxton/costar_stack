#!/usr/bin/python
"""
This contains two methods of having the user input a bounding box

--Usage--
Ensure the Kinect is running with ROS.
    roslaunch openni_launch openni.launch depth_registration:=true
Run this program
    rosrun predicator_occupancy_analyzer pick_volume

--Output--
A volume is sent to the ROS parameter server (param="volume")

"""

import optparse
import rospy
from predicator_8020_module.utils_8020 import pick_bounding_box
from pyKinectTools.utils.pointcloud_conversions import *

from sensor_msgs.msg import PointCloud2
import cv2

# Global images
im_pos = None
im_rgb = None
# Bounding box
bounding_box = []


def pointcloud_callback(data):
    """
    data : ros msg data
    """
    global im_pos, im_rgb
    img_pos_raw = pointcloud2_to_array(data, split_rgb=True)
    im_pos = np.dstack([img_pos_raw['x'], img_pos_raw['y'], img_pos_raw['z']])
    im_pos = np.nan_to_num(im_pos)

    im_rgb = np.dstack([img_pos_raw['r'], img_pos_raw['g'], img_pos_raw['b']])
    im_rgb = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)

if __name__ == '__main__':
    rospy.init_node('pick_8020_region')

    parser = optparse.OptionParser()
    parser.add_option("-c", "--camera", dest="camera",
                      help="name of camera", default="camera")
    (options, args) = parser.parse_args()

    camera_name = options.camera

    # Update pointcloud
    cloud_uri = "/{}/depth_registered/points".format(camera_name)
    rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)

    # Wait until we have a pointcloud
    rate = rospy.Rate(1)
    while im_rgb is None or im_pos is None:
        rate.sleep()

    # Have user add volume
    # volume = learn_volume()
    # rospy.set_param("volume", volume)
    bounding_box = pick_bounding_box(im_rgb)
    rospy.set_param("bounding_box_8020", bounding_box)
