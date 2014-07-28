#!/usr/bin/python

"""
Detects if sphere is occupied or not.
"""

import rospy

import cv2
from pyKinectTools.utils.pointcloud_conversions import *
from pyKinectTools.utils.transformations import *

from sensor_msgs.msg import PointCloud2
# from ar_track_alvar.msg import AlvarMarkers
from predicator_msgs.msg import *

from threading import Lock
from copy import deepcopy

# Volume
bounding_box = np.zeros([2, 3], np.float)
occupied = False
pct_filled = 0.1
occupancy_center = None
occupancy_radius = None

# Images
im_pos = None
im_depth = None
im_rgb = None
mask = np.ones([480, 640], bool)
image_lock = Lock()
display = False


def draw_box(img, box):
    """ Function to draw the rectangle """
    cv2.rectangle(img, (box[0], box[1]), (box[0]+box[2], box[1]+box[3]), (255, 0, 0))


def bbox_occupancy_check(pts, bounding_box, pct_filled=.2, output_mask=False):
    x_check_lower = pts[:, 0] > bounding_box[0][0]
    x_check_upper = pts[:, 0] < bounding_box[1][0]
    y_check_lower = pts[:, 1] > bounding_box[0][1]
    y_check_upper = pts[:, 1] < bounding_box[1][1]
    z_check_lower = pts[:, 2] > bounding_box[0][2]
    z_check_upper = pts[:, 2] < bounding_box[1][2]

    in_bounds = x_check_lower & x_check_upper &\
        y_check_lower & y_check_upper &\
        z_check_lower & z_check_upper

    occupied = in_bounds.sum() / (pts.shape[0]*pct_filled) > pct_filled
    print "pct filled:", in_bounds.sum() / (pts.shape[0]*pct_filled)

    if output_mask:
        return occupied, in_bounds
    else:
        return occupied


def sphere_occupancy_check(pts, center, radius, px_thresh=1000, output_mask=False):
    global occupancy_center, occupancy_radius
    img_radius = np.sqrt(np.nansum((pts - center)**2, -1))
    mask = img_radius < radius

    occupied = np.sum(mask) > px_thresh

    if output_mask:
        return occupied, mask
    else:
        return occupied


def markers_callback(data):
    global markers
    markers = deepcopy(data)


def pointcloud_callback(data):
    rospy.logwarn('got image')
    """
    data : ros msg data
    """
    global im_pos, im_depth, im_rgb, image_lock, occupied, bounding_box, markers, pct_filled, mask
    global frame_time, frame_seq

    with image_lock:
        frame_time = data.header.stamp
        frame_seq = data.header.seq

        x = pointcloud2_to_array(data, split_rgb=True)
        im_pos = np.dstack([x['x'], x['y'], x['z']])
        im_pos = np.nan_to_num(im_pos)
        im_depth = im_pos[:, :, 2]

        im_rgb = np.dstack([x['r'], x['g'], x['b']])
        im_rgb = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)
        # print 3
        # if markers is None:
            # return
        # if abs(markers.header.seq - data.header.seq) < 3:
        # marker_ids = [m.id for m in markers.markers]
        # if 1 in marker_ids:

        px_thresh = 1000
        occupied, mask = sphere_occupancy_check(im_pos.reshape([-1, 3]),
                                              occupancy_center, occupancy_radius, px_thresh, output_mask=True)
        # occupied, mask = bbox_occupancy_check(im_pos.reshape([-1, 3]),
                                              # bounding_box, pct_filled, output_mask=True)
        # print "Occupied:", occupied

        mask = mask.reshape([480, 640])


if __name__ == '__main__':
    # Setup ros/publishers
    rospy.init_node('occupancy_module')
    pub_list = rospy.Publisher('/predicator/input', PredicateList)
    pub_valid = rospy.Publisher('/predicator/input', ValidPredicates)

    # Setup subscribers
    cloud_uri = "/camera/depth_registered/points"
    rospy.Subscriber(cloud_uri, PointCloud2, pointcloud_callback, queue_size=10)

    # marker_uri = "ar_pose_marker"
    # rospy.Subscriber(marker_uri, AlvarMarkers, markers_callback, queue_size=10)

    # Get occupancy params
    occupancy_center = np.array(rospy.get_param("occupancy_center"))
    occupancy_radius = np.array(rospy.get_param("occupancy_radius"))

    display = rospy.get_param('display', True)

    # Setup valid predicates
    pval = ValidPredicates()
    pval.predicates = ['occupied']
    pval.value_predicates = ['']
    pval.assignments = ['occupancy_sensor']
    pub_valid.publish(pval)

    rate = rospy.Rate(30)
    rate.sleep()
    
    while not rospy.is_shutdown():
        while im_pos == None:
            rate.sleep()

        # Tell predicator if the sensor is true/false
        occupied_confidence = (occupied*1-.5)*2

        # Publish occupancy predicates
        ps = PredicateList()
        ps.pheader.source = rospy.get_name()
        ps.statements = []

        if occupied_confidence == 1: # Publish a filled statement, meaning TRUE
            ps.statements.append(PredicateStatement(predicate='occupied',
                                                confidence=occupied_confidence,
                                                value=PredicateStatement.TRUE,
                                                num_params=1,
                                                params=["occupancy_sensor", "", ""]))
        elif occupied_confidence == -1:
            ps.statements.append(PredicateStatement(predicate='empty',
                                                confidence=occupied_confidence,
                                                value=PredicateStatement.TRUE,
                                                num_params=1,
                                                params=["occupancy_sensor", "", ""]))
        pub_list.publish(ps)

        # Show colored image to reflect if a space is occupied
        if display:
            with image_lock:
                img = np.repeat((im_depth * ~mask)[:, :, None], 3, -1) / im_depth.max()
            img_mask = np.zeros([480, 640, 3])
            if occupied:
                img_mask[:, :, 1] = mask
            else:
                img_mask[:, :, 2] = mask
            cv2.imshow("img", img + img_mask)
            cv2.waitKey(30)

        rate.sleep()
