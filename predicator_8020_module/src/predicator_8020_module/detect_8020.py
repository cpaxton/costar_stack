#!/usr/bin/python

"""
Detects if plates are on the table 
Plates have 3, 4, or 8 holes
"""

import numpy as np
import optparse
import rospy
import tf

import cv2
from pyKinectTools.utils.pointcloud_conversions import *

from sensor_msgs.msg import PointCloud2
from predicator_msgs.msg import *
from std_msgs.msg import Empty
from predicator_8020_module.utils_8020 import *

from threading import Lock
from copy import deepcopy

# Images
im_display = None
im_pos = None
im_depth = None
im_rgb = None
mask = np.ones([480, 640], bool)
image_lock = Lock()

# 8020 Detector
display = False
clf_mean = None
clf_w = None
closest_parts = {x:None for x in [3,4,8]}
pub_list = None


def draw_box(img, box):
    """ Function to draw the rectangle """
    cv2.rectangle(img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (255, 0, 0))


def pointcloud_callback(data):
    # rospy.logwarn('got image')
    """
    data : ros msg data
    """
    # global im_display
    global im_pos, im_depth, im_rgb, image_lock
    # , bounding_box, mask
    # global frame_time, frame_seq
    # global clf_mean, clf_w
    # global closest_parts

    with image_lock:
        frame_time = data.header.stamp
        frame_seq = data.header.seq

        x = pointcloud2_to_array(data, split_rgb=True)
        im_pos = np.dstack([x['x'], x['y'], x['z']])
        im_pos = np.nan_to_num(im_pos)
        im_depth = im_pos[:, :, 2]

        im_rgb = np.dstack([x['r'], x['g'], x['b']])
        im_rgb = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)
        # rospy.logwarn(3)
        # if markers is None:
            # return
        # if abs(markers.header.seq - data.header.seq) < 3:
        # marker_ids = [m.id for m in markers.markers]
        # if 1 in marker_ids:

        # px_thresh = 1000
        # occupied, mask = sphere_occupancy_check(im_pos.reshape([-1, 3]),
                                              # occupancy_center, occupancy_radius, px_thresh, output_mask=True)
        # occupied, mask = bbox_occupancy_check(im_pos.reshape([-1, 3]),
                                              # bounding_box, pct_filled, output_mask=True)
        # rospy.logwarn("Occupied:", occupied)

        # mask = mask.reshape([480, 640])

        # im, mask = extract_foreground_poly(im_rgb, bounding_box)
        # clf_mean, clf_w = train_clf(im, mask)
        # pred_mask, objects, props = get_foreground(im, clf_mean, clf_w)
        # all_centroids = all_holes = extract_holes(im, pred_mask, objects, props)

        # parts, classes = get_closest_part(all_centroids, all_holes)

        # rospy.logwarn("# Holes", len(all_holes))
        # im_display = plot_holes(im_rgb, all_holes)

        # closest_parts = parts


def process_plate_detector(data):
    global im_display
    global im_pos, im_depth, im_rgb, bounding_box, mask
    global frame_time, frame_seq
    global clf_mean, clf_w
    global closest_parts
    global pub_list

    rospy.logwarn("Trying to process plate")
    if im_rgb is None:
        return False
    rospy.logwarn("Processing plate")

    with image_lock:
        im, mask = extract_foreground_poly(im_rgb, bounding_box)
        # clf_mean, clf_w = train_clf(im, mask)
        pred_mask, objects, props = get_foreground(im, clf_mean, clf_w)
        all_centroids, all_holes = extract_holes(im, pred_mask, objects, props, im_pos, im_rgb)

        rospy.logwarn("# Plates:", len(all_holes))
        if len(all_centroids) > 0:
            closest_parts, classes = get_closest_part(all_centroids, all_holes)
            im_display = plot_holes(im_rgb, all_holes)
            # im_display = (pred_mask > 0)*255

        else:
            for c in closest_parts:
                closest_parts[c] = None

        ps = PredicateList()
        ps.pheader.source = rospy.get_name()
        ps.statements = []

        # Send predicates for each plate
        for c in [3,4,8]:
            # Check if the plate is available
            if closest_parts[c] is None:
                continue
            # Setup/send predicate
            plate_name = "plate_{}".format(c)
            statement = PredicateStatement(predicate=plate_name,
                                                confidence=1,
                                                value=PredicateStatement.TRUE,
                                                num_params=0,
                                                params=["", "", ""])
            ps.statements += [statement]
        pub_list.publish(ps)

        # Send TFs for each plate at every timestep
        for c in closest_parts:
            plate_name = "plate_{}".format(c)
            if closest_parts[c] is None:
                x, y, z = [-1, -1, -1]
            else:
                x, y, z = closest_parts[c]

            tf_broadcast.sendTransform([x, y, z],
                                        [0,0,0,1],
                                        rospy.Time.now(), plate_name, 
                                        "camera_2_rgb_optical_frame")


    return True


if __name__ == '__main__':

    try:
        parser = optparse.OptionParser()
        parser.add_option("-c", "--camera", dest="camera",
                          help="name of camera", default="camera_2")
        parser.add_option("-n", "--namespace", dest="namespace",
                          help="namespace for occupancy data", default="")    
        (options, args) = parser.parse_args()

        camera_name = options.camera
        namespace = options.namespace
    except:
        camera_name = 'camera'

    # Setup ros/publishers
    rospy.init_node('plate_detector_module')
    pub_list = rospy.Publisher('/predicator/input', PredicateList)
    pub_valid = rospy.Publisher('/predicator/valid_input', ValidPredicates)

    # Setup subscribers
    cloud_uri = "/{}/depth_registered/points".format(camera_name)
    rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)
    rospy.Subscriber("plates/detect", Empty, process_plate_detector, queue_size=10)

    # Get occupancy params
    # occupancy_center = np.array(rospy.get_param("/{}/occupancy_center".format(namespace)))
    # occupancy_radius = np.array(rospy.get_param("/{}/occupancy_radius".format(namespace)))

    display = rospy.get_param('display', True)


    # Setup TF
    tf_broadcast = tf.TransformBroadcaster()

    # Setup valid predicates
    predicate_param = 'plate_detector'
    pval = ValidPredicates()
    pval.predicates = ['plate_3', 'plate_4', 'plate_8']
    pval.value_predicates = ['']
    pval.assignments = [predicate_param]
    pub_valid.publish(pval)

    rospy.set_param('/instructor_landmark/plate_3','plate_3')
    rospy.set_param('/instructor_landmark/plate_4','plate_4')
    rospy.set_param('/instructor_landmark/plate_8','plate_8')

    rate = rospy.Rate(30)
    rate.sleep()

    while im_rgb is None or im_pos is None:
        rate.sleep()

    # Get user defined params
    bounding_box = rospy.get_param('bounding_box_8020', [])
    clf_mean = np.array(rospy.get_param("bounding_box_clf_mean"), None)
    clf_w = np.array(rospy.get_param("bounding_box_clf_w"), None)
    if clf_mean is None or clf_w is None:
        rospy.logwarn("Can't load user paramaters")

    # pick_bounding_box()
    # Setup classifier
    # im, mask = extract_foreground_poly(im_rgb, bounding_box)
    # clf_mean, clf_w = train_clf(im, mask)

    # pred_mask, objects, props = get_foreground(im, clf_mean, clf_w)
    # all_centroids, all_holes = extract_holes(im, pred_mask, objects, props, im_pos, im_rgb)

    # if len(all_centroids) > 0:
    #     closest_parts, classes = get_closest_part(all_centroids, all_holes)
    # else:
    #     for c in closest_parts:
    #         closest_parts[c] = None

    # rospy.logwarn("# Plates:", len(all_holes))
    # im_display = plot_holes(im_rgb, all_holes)


    display = True
    # ret = process_plate_detector([])

    while not rospy.is_shutdown():
        # rospy.logwarn("Running1")
        while im_pos == None:
            rate.sleep()

        # ret = process_plate_detector([])
        # Show colored image to reflect if a space is occupied
        if display and im_display is not None:
            # rospy.logwarn("Display")
            cv2.imshow("img", im_display)
            cv2.waitKey(30)
        # else:
            # rospy.logwarn("No display image")

        # Send TFs for each plate at every timestep
        for c in closest_parts:
            plate_name = "plate_{}".format(c)
            if closest_parts[c] is None:
                x, y, z = [-1, -1, -1]
            else:
                x, y, z = closest_parts[c]

            tf_broadcast.sendTransform([x, y, z],
                                        [0,0,0,1],
                                        rospy.Time.now(), plate_name, 
                                        "camera_2_rgb_optical_frame")

        rate.sleep()
