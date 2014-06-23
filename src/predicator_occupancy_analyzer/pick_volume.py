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

import rospy
from predicator_msgs.msg import *
from pyKinectTools.utils.pointcloud_conversions import *
from pyKinectTools.utils.transformations import *

from sensor_msgs.msg import PointCloud2
import cv2

# States
pause_video = False
waiting_state = True
drawing_box = False
drawing_box_done = False
# Global images
im_pos = None
im_rgb = None
# Bounding box
bbox_img = [[0, 0], [0, 0]]
bbox = np.zeros([2, 3], np.float)
bbox_sliders = np.zeros([2, 3], np.int)
box_center = [0, 0]


def pause_video_callback(event, x, y, flags, param):
    global pause_video, waiting_state
    if event == cv2.EVENT_LBUTTONDOWN:
        pause_video = True
    if event == cv2.EVENT_LBUTTONUP:
        waiting_state = False


def center_callback(event, x, y, flags, param):
    global box_center, waiting_state
    if event == cv2.EVENT_LBUTTONDOWN:
        box_center = [x, y]
    if event == cv2.EVENT_LBUTTONUP:
        waiting_state = False


def bounding_box_callback(event, x, y, flags, param):
    global drawing_box, drawing_box_done, bbox_img
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing_box = True
        [bbox_img[0][0], bbox_img[0][1], bbox_img[1][0], bbox_img[1][1]] = [x, y, 0, 0]
        print bbox[0]
    if event == cv2.EVENT_LBUTTONUP:
        drawing_box = False
        drawing_box_done = True
    if bbox_img[1][0] < 0:
        bbox_img[0][0] += bbox_img[1][0]
        bbox_img[0][1] *= -1
    if bbox_img[1][1] < 0:
        bbox_img[0][1] += bbox_img[1][1]
        bbox_img[1][1] *= -1

    if event == cv2.EVENT_MOUSEMOVE:
        if drawing_box:
            bbox_img[1][0] = x - bbox_img[0][0]
            bbox_img[1][1] = y - bbox_img[0][1]


def draw_box(img, box):
    """ Function to draw the rectangle """
    cv2.rectangle(img, (box[0][0], box[0][1]),
                 (box[0][0]+box[1][0], box[0][1]+box[1][1]),
                 (255, 0, 0))


def get_bounding_box_sliders():
    cv2.destroyAllWindows()
    cv2.namedWindow("learn_volume")

    pause_video = False

    txt = "Click once to pause the video"
    cv2.setMouseCallback("learn_volume", pause_video_callback)
    while not pause_video:
        img_pause_rgb = im_rgb.copy()
        img_pause_pos = im_pos.copy()

        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.imshow("learn_volume", img)
        cv2.waitKey(100)

    img_rgb_c = img_pause_rgb.copy()
    img_pos_c = img_pause_pos.copy()

    pos_min = np.nanmin(img_pos_c.reshape([-1, 3]), 0)
    pos_max = np.nanmax(img_pos_c.reshape([-1, 3]), 0)
    pos_diff = ((pos_max - pos_min)*1000.).astype(np.int)

    empty_fcn = lambda _: None
    cv2.createTrackbar("X Min", "learn_volume", 0, pos_diff[0], empty_fcn)
    cv2.createTrackbar("X Max", "learn_volume", pos_diff[0]-1, pos_diff[0], empty_fcn)
    cv2.createTrackbar("Y Min", "learn_volume", 0, pos_diff[1], empty_fcn)
    cv2.createTrackbar("Y Max", "learn_volume", pos_diff[1]-1, pos_diff[1], empty_fcn)
    cv2.createTrackbar("Z Min", "learn_volume", 0, pos_diff[2], empty_fcn)
    cv2.createTrackbar("Z Max", "learn_volume", pos_diff[2]-1, pos_diff[2], empty_fcn)
    cv2.imshow("learn_volume", img)

    while 1:
        bbox_sliders[0][0] = cv2.getTrackbarPos('X Min', 'learn_volume')
        bbox_sliders[1][0] = cv2.getTrackbarPos('X Max', 'learn_volume')
        bbox_sliders[0][1] = cv2.getTrackbarPos('Y Min', 'learn_volume')
        bbox_sliders[1][1] = cv2.getTrackbarPos('Y Max', 'learn_volume')
        bbox_sliders[0][2] = cv2.getTrackbarPos('Z Min', 'learn_volume')
        bbox_sliders[1][2] = cv2.getTrackbarPos('Z Max', 'learn_volume')
        bbox[0] = pos_min + bbox_sliders[0]/1000.
        bbox[1] = pos_min + bbox_sliders[1]/1000.

        print bbox_sliders
        x_mask = (img_pos_c[:, :, 0] > bbox[0][0]) * (img_pos_c[:, :, 0] < bbox[1][0])
        y_mask = (img_pos_c[:, :, 1] > bbox[0][1]) * (img_pos_c[:, :, 1] < bbox[1][1])
        z_mask = (img_pos_c[:, :, 2] > bbox[0][2]) * (img_pos_c[:, :, 2] < bbox[1][2])
        mask = x_mask & y_mask & z_mask
        mask[img_pos_c[:, :, 2] == 0] = False
        cv2.imshow("learn_volume", img_rgb_c * mask[:, :, None])
        cv2.waitKey(30)


def learn_volume():
    """ User clicks on a bounding box in the color image to specify the volume """
    global drawing_box, drawing_box_done, box_center, bbox_img
    global waiting_state, pause_video

    cv2.destroyAllWindows()
    box_center = [0, 0]
    drawing_box_done = False
    bbox_img = [[0, 0], [0, 0]]
    cv2.namedWindow("learn_volume")

    # Step 1: pause the video
    txt = "Click once to pause the video"
    print txt
    cv2.setMouseCallback("learn_volume", pause_video_callback)
    waiting_state = True
    pause_video = False
    while not pause_video:
        img_pause_rgb = im_rgb.copy()
        img_pause_pos = im_pos.copy()

        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.imshow("learn_volume", img)
        if cv2.waitKey(30) >= 0:
            break

    # Wait until mouse up
    while waiting_state:
        if cv2.waitKey(30) >= 0:
            break

    # Step 2: Click on point of interest
    txt = "Click the center of your volume"
    print txt
    cv2.setMouseCallback("learn_volume", center_callback)
    waiting_state = True
    while box_center[0] == 0:
        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.imshow("learn_volume", img)

        if cv2.waitKey(30) >= 0:
            break

    # Wait until mouse up
    while waiting_state:
        if cv2.waitKey(30) >= 0:
            break

    # Step 3: Draw box around point of interest
    txt = "Draw a box around the volume"
    cv2.setMouseCallback("learn_volume", bounding_box_callback)
    while not drawing_box_done:
        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))

        if drawing_box:
            draw_box(img, bbox_img)
        cv2.imshow("learn_volume", img)

        if cv2.waitKey(30) >= 0:
            break

    # (Step 4): Get min/max in bounding box
    bbox_pos = img_pause_pos[bbox_img[0][1]:bbox_img[0][1]+bbox_img[1][1],
                             bbox_img[0][0]:bbox_img[0][0]+bbox_img[1][0]]
    bbox_min = bbox_pos[bbox_pos[:, :, 2] != 0].reshape([-1, 3]).min(0)
    bbox_max = bbox_pos[bbox_pos[:, :, 2] != 0].reshape([-1, 3]).max(0)

    bounding_box = [bbox_min.tolist(), bbox_max.tolist()]

    return bounding_box


def pointcloud_callback(data):
    """
    data : ros msg data
    """
    global im_pos, im_rgb
    img_pos_raw = pointcloud2_to_array(data, split_rgb=True)
    im_pos = np.dstack([img_pos_raw['x'], img_pos_raw['y'], img_pos_raw['z']])
    im_pos = np.nan_to_num(im_pos)

    im_rgb = np.dstack([x['r'], x['g'], x['b']])
    im_rgb = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)

    # frame_time = data.header.stamp
    # frame_seq = data.header.seq


if __name__ == '__main__':
    rospy.init_node('learn_occupancy_volume')

    # Update pointcloud
    cloud_uri = "/camera/depth_registered/points"
    rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)

    # Wait until we have a pointcloud
    rate = rospy.Rate(1)
    while im_rgb is None or im_pos is None:
        rate.sleep()

    # Have user add volume
    volume = learn_volume()
    rospy.set_param("volume", volume)

    print "Volume added:"
    print volume
