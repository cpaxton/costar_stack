#!/usr/bin/python

import rospy
from predicator_msgs.msg import *
from pyKinectTools.utils.pointcloud_conversions import *
from pyKinectTools.utils.transformations import *

from sensor_msgs.msg import PointCloud2
import cv2


drawing_box = False
drawing_box_done = False
# bbox = [0, 0, 0, 0]
bbox_img = [[0, 0], [0, 0]]
bbox = np.zeros([2, 3], np.float)
bbox_sliders = np.zeros([2, 3], np.int)
box_center = [0, 0]
pause_video = False
waiting_state = True


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
    # if bbox_img[2] < 0:
        # bbox[0] += bbox[2]
        # bbox[2] *= -1
    if bbox_img[1][0] < 0:
        bbox_img[0][0] += bbox_img[1][0]
        bbox_img[0][1] *= -1
    # if bbox[3] < 0:
    #     bbox[1] += bbox[3]
    #     bbox[3] *= -1
    if bbox_img[1][1] < 0:
        bbox_img[0][1] += bbox_img[1][1]
        bbox_img[1][1] *= -1

    if event == cv2.EVENT_MOUSEMOVE:
        if drawing_box:
            bbox_img[1][0] = x - bbox_img[0][0]
            bbox_img[1][1] = y - bbox_img[0][1]
            # bbox[2] = x-bbox[0]
            # bbox[3] = y-bbox[1]


# def slider_x_min(val, x):
#     global bbox_sliders
#     bbox_sliders[0][0] = val
#     # print val, x


# def slider_x_max(val, x):
#     global bbox_sliders
#     bbox_sliders[0][1] = val
#     print val, x, "yay"


# def draw_box(img, box):
#     """ Function to draw the rectangle """
#     cv2.rectangle(img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (255, 0, 0))
def draw_box(img, box):
    """ Function to draw the rectangle """
    cv2.rectangle(img, (box[0][0], box[0][1]),
                 (box[0][0]+box[1][0], box[0][1]+box[1][1]),
                 (255, 0, 0))


def get_bounding_box_sliders():
    cv2.destroyAllWindows()
    cv2.namedWindow("get_bounding_box")

    pause_video = False

    txt = "Click once to pause the video"
    cv2.setMouseCallback("get_bounding_box", pause_video_callback)
    while not pause_video:
        img_pause_rgb = im_rgb.copy()
        img_pause_pos = im_pos.copy()

        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.imshow("get_bounding_box", img)
        cv2.waitKey(100)

    img_rgb_c = img_pause_rgb.copy()
    img_pos_c = img_pause_pos.copy()

    pos_min = np.nanmin(img_pos_c.reshape([-1, 3]), 0)
    pos_max = np.nanmax(img_pos_c.reshape([-1, 3]), 0)
    pos_diff = ((pos_max - pos_min)*1000.).astype(np.int)

    empty_fcn = lambda _: None
    cv2.createTrackbar("X Min", "get_bounding_box", 0, pos_diff[0], empty_fcn)
    cv2.createTrackbar("X Max", "get_bounding_box", pos_diff[0]-1, pos_diff[0], empty_fcn)
    cv2.createTrackbar("Y Min", "get_bounding_box", 0, pos_diff[1], empty_fcn)
    cv2.createTrackbar("Y Max", "get_bounding_box", pos_diff[1]-1, pos_diff[1], empty_fcn)
    cv2.createTrackbar("Z Min", "get_bounding_box", 0, pos_diff[2], empty_fcn)
    cv2.createTrackbar("Z Max", "get_bounding_box", pos_diff[2]-1, pos_diff[2], empty_fcn)
    cv2.imshow("get_bounding_box", img)

    while 1:
        bbox_sliders[0][0] = cv2.getTrackbarPos('X Min', 'get_bounding_box')
        bbox_sliders[1][0] = cv2.getTrackbarPos('X Max', 'get_bounding_box')
        bbox_sliders[0][1] = cv2.getTrackbarPos('Y Min', 'get_bounding_box')
        bbox_sliders[1][1] = cv2.getTrackbarPos('Y Max', 'get_bounding_box')
        bbox_sliders[0][2] = cv2.getTrackbarPos('Z Min', 'get_bounding_box')
        bbox_sliders[1][2] = cv2.getTrackbarPos('Z Max', 'get_bounding_box')
        bbox[0] = pos_min + bbox_sliders[0]/1000.
        bbox[1] = pos_min + bbox_sliders[1]/1000.

        print bbox_sliders
        x_mask = (img_pos_c[:, :, 0] > bbox[0][0]) * (img_pos_c[:, :, 0] < bbox[1][0])
        y_mask = (img_pos_c[:, :, 1] > bbox[0][1]) * (img_pos_c[:, :, 1] < bbox[1][1])
        z_mask = (img_pos_c[:, :, 2] > bbox[0][2]) * (img_pos_c[:, :, 2] < bbox[1][2])
        mask = x_mask & y_mask & z_mask
        mask[img_pos_c[:, :, 2] == 0] = False
        cv2.imshow("get_bounding_box", img_rgb_c * mask[:, :, None])
        cv2.waitKey(30)


def get_bounding_box():
    cv2.destroyAllWindows()
    global pause_video, drawing_box, drawing_box_done, bbox, waiting_state
    pause_video = False
    box_center = [0, 0]
    drawing_box_done = False
    bbox_img = [[0, 0], [0, 0]]
    cv2.namedWindow("get_bounding_box")

    txt = "Click once to pause the video"
    cv2.setMouseCallback("get_bounding_box", pause_video_callback)
    waiting_state = True
    while not pause_video:
        img_pause_rgb = im_rgb.copy()
        img_pause_pos = im_pos.copy()

        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        cv2.imshow("get_bounding_box", img)
        cv2.waitKey(30)
    while waiting_state:
        cv2.waitKey(30)

    txt = "Click the center of your occupancy volume"
    cv2.setMouseCallback("get_bounding_box", center_callback)
    waiting_state = True
    while box_center[0] == 0:
        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        if drawing_box:
            draw_box(img, bbox_img)
        cv2.imshow("get_bounding_box", img)
        cv2.waitKey(30)
    while waiting_state:
        cv2.waitKey(30)

    txt = "Draw a box around the volume"
    cv2.setMouseCallback("get_bounding_box", bounding_box_callback)
    while not drawing_box_done:
        img = img_pause_rgb.copy()
        cv2.putText(img, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
        if drawing_box:
            draw_box(img, bbox_img)
        cv2.imshow("get_bounding_box", img)
        cv2.waitKey(30)


    # volume_center = im_pos[box_center[1], box_center[1]]
    # box_min = volume_center.min()
    bbox_pos = im_pos[bbox_img[0][1]:bbox_img[0][1]+bbox_img[1][1], bbox_img[0][0]:bbox_img[0][0]+bbox_img[1][0]]
    bbox_min = bbox_pos[bbox_pos[:, :, 2] != 0].reshape([-1, 3]).min(0)
    bbox_max = bbox_pos[bbox_pos[:, :, 2] != 0].reshape([-1, 3]).max(0)

    bounding_box = [bbox_min.tolist(), bbox_max.tolist()]

    return bounding_box

    # imshow(im_pos[bbox_img[0][1]:bbox_img[0][1]+bbox_img[1][1], bbox_img[0][0]:bbox_img[0][0]+bbox_img[1][0]])


im_pos = None
im_depth = None
im_rgb = None
def pointcloud_callback(data):
    """
    data : ros msg data
    """
    global im_pos, im_depth, im_rgb
    x = pointcloud2_to_array(data, split_rgb=True)
    # with self.image_lock:
    im_pos = np.dstack([x['x'], x['y'], x['z']])
    im_pos = np.nan_to_num(im_pos)
    im_depth = im_pos[:, :, 2]

    im_rgb = np.dstack([x['r'], x['g'], x['b']])
    im_rgb = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)

    # frame_time = data.header.stamp
    # frame_seq = data.header.seq


rospy.init_node('occupancy_predicate')
# pub = rospy.Publisher('predicator/input', PredicateList)
cloud_uri = "/camera/depth_registered/points"
rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)

rate = rospy.Rate(1)
while im_rgb is None or im_pos is None:
    rate.sleep()
volume = get_bounding_box()
rospy.set_param("volume", volume)


if __name__ == '__main__':
    rospy.init_node('occupancy_predicate')
    # pub = rospy.Publisher('predicator/input', PredicateList)
    cloud_uri = "/camera/depth_registered/points"
    rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)

    rate = rospy.Rate(1)
    while im_rgb is None or im_pos is None:
        rate.sleep()
    volume = get_bounding_box()
    rospy.set_param("volume", volume)
    # volume = rospy.get_param("test_param")


    print ps
    print type(ps.statements[0].num_params)

    try:
        while not rospy.is_shutdown():
            # pub.publish(ps)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
