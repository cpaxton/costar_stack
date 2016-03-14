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
from predicator_8020_module.utils_8020 import *
from pyKinectTools.utils.pointcloud_conversions import *

from skimage.draw import circle, line, polygon
from sensor_msgs.msg import PointCloud2
import cv2

# Global images
im_pos = None
im_rgb = None
# Bounding box
bounding_box = []

def mouse_event(event, x, y, flags=None, params=None):
    # global bounding_box
    if event == cv2.EVENT_LBUTTONDOWN:
        # bounding_box += [[y, x]]
        params += [[y, x]]
        # print "New point:", y, x

def pick_bounding_box(im_rgb):
    # global bounding_box
    bounding_box = []

    im = im_rgb.copy()
    im_display = im_rgb.copy()

    # Display instructions
    txt = "Click on 4 points."
    cv2.putText(im_display, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))

    # Have user specify region
    cv2.namedWindow("pick_region")
    cv2.setMouseCallback("pick_region", mouse_event, bounding_box)
    while len(bounding_box) < 4:
        # Display circles on corners
        for pt in bounding_box:
            circ = circle(pt[0], pt[1], 5)
            im_display[circ[0], circ[1]] = 255
        # Display lines between points
        # if len(bounding_box) > 1:
            # for i in range(len(bounding_box)-1):
                # pt1 = boundin_box[i]
                # pt2 = boundin_box[i+1]

        cv2.imshow("pick_region", im_display)
        ret = cv2.waitKey(30)
    cv2.destroyWindow("pick_region")

    return np.array(bounding_box)

def pick_foreground(im_rgb):
    points = []

    im = im_rgb.copy()
    im_display = im_rgb.copy()

    # Have user specify region
    cv2.namedWindow("pick_region")
    cv2.setMouseCallback("pick_region", mouse_event, points)

    while len(points) < 1:
        # Display instructions
        txt = "Click background point."
        cv2.putText(im_display, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))

        cv2.imshow("pick_region", im_display)
        ret = cv2.waitKey(30)

    im_display = im_rgb.copy()
    while len(points) < 2:
        # Display instructions
        txt = "Click foregorund point."
        cv2.putText(im_display, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))

        # Display circles on corners
        for pt in points:
            circ = circle(pt[0], pt[1], 5)
            im_display[circ[0], circ[1]] = 255

        cv2.imshow("pick_region", im_display)
        ret = cv2.waitKey(30)
    cv2.destroyWindow("pick_region")

    return np.array(points)    


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
                      help="name of camera", default="camera_2")
    #                  help="name of camera", default="camera")
    (options, args) = parser.parse_args()

    camera_name = options.camera

    # Update pointcloud
    cloud_uri = "/{}/depth_registered/points".format(camera_name)
    rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)

    # Wait until we have a pointcloud
    rate = rospy.Rate(1)
    while im_rgb is None or im_pos is None:
        print "Waiting for image..."
        rate.sleep()

    # Have user add volume
    # volume = learn_volume()
    # rospy.set_param("volume", volume)
    bounding_box = pick_bounding_box(im_rgb)
    rospy.set_param("bounding_box_8020", bounding_box.tolist())

    # Show masked image
    poly_points = polygon(bounding_box[:,0], bounding_box[:,1])
    mask = np.zeros(im_rgb.shape[:2], np.int)
    mask[poly_points[0], poly_points[1]] = True

    im_display = im_rgb.copy()
    im_display = im_display*1. + np.array([-50,-50,50])*(mask[:,:,None]==0)
    # im_display = im_display*1. + np.array([0,0,100])
    im_display = im_display*1. + np.array([0,50,0])*(mask[:,:,None])
    im_display[im_display<0] = 0
    im_display[im_display>255] = 255
    im_display = im_display.astype(np.uint8)

    im, mask = extract_foreground_poly(im_rgb, bounding_box)

    #from IPython import embed
    #embed()

    #pts = pick_foreground(im)
    pts = pick_foreground(im_display)

    bg = im[pts[0][0], pts[0][1]]
    fg = im[pts[1][0], pts[1][1]]
    clf_mean = np.array([(bg+fg)/2])
    clf_w = np.array([1])
    print "Mean", clf_mean, bg, fg

    rospy.set_param("bounding_box_clf_mean", clf_mean.tolist())
    rospy.set_param("bounding_box_clf_w", clf_w.tolist())

    # clf_mean, clf_w = train_clf(im, mask)
    mask_pred, objects, props = get_foreground(im, clf_mean, clf_w)
    all_centroids, all_holes = extract_holes(im, mask_pred, objects, props, im_pos, im_rgb)
    im_display = plot_holes(im_rgb, all_holes)
    im_display = im_display*mask_pred[:,:,None]

    # Display instructions
    txt = "Press enter to exit"
    cv2.putText(im_display, txt, (10, 25), cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255))
    cv2.imshow("im", im_display)
    cv2.waitKey()

    
