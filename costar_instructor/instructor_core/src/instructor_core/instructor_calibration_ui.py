#!/usr/bin/env python

import numpy as np

import roslib
roslib.load_manifest('instructor_core')
import rospy
import rospkg

import tf
import tf_conversions as tf_c
import PyKDL as kdl
from semi_static_transform_publisher.srv import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtCore import Qt
import QtCore


class CalibrationUI(Plugin):

    n_cameras = 2
    update_poses = True

    cameras = ['camera_1', 'camera_2']
    camera_marker_frame_names = {}

    cameras_calibrated = []
    calibrated = False

    all_xyz = []
    all_rpy = []

    def __init__(self, context):
        super(CalibrationUI, self).__init__(context)

        self.setObjectName('Instructor Calibration UI')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which is a sibling of this file
        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_core') + '/ui/calibration.ui'

        # Load the ui attributes into the main widget
        loadUi(ui_path, self._widget)
        self._widget.setObjectName('InstructorCalibrationUI')
        self._widget.setWindowTitle('Instructor Calibration UI')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Add widget to the user interface
        context.add_widget(self._widget)
        palette = QPalette ()
        palette.setColor(QPalette.Background, Qt.white)
        self._widget.setPalette(palette)

        # Initialize empty set of available markers for each camera
        for c in range(self.n_cameras):
            self.camera_marker_frame_names[c] = {}
            self.cameras_calibrated += [False]

        # Add listener to detect AR markers
        self.tf_listen = tf.TransformListener()
        self.tf_broadcast = tf.TransformBroadcaster()

        # Add timer
        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL("timeout()"), self.update)
        self.timer.start(100)
        self._widget.done_btn.clicked.connect(self.done_event)
        self._widget.recalibrate_btn.clicked.connect(self.recalibrate_event)

        ### Change Indicator Sample Code --------------------------------------#
        self._widget.done_btn.setStyleSheet('color:#999999')

    def update(self):
        # Return if the calibration is finished
        if not self.update_poses:
            return

        # Get all current TF frames. Add any new ones to AR tag list.
        current_frames = self.tf_listen.getFrameStrings()
        current_frames = [f for f in current_frames if "ar_marker" in f and not ("filtered" in f)]
        # current_frames = [f for f in current_frames if "ar_marker" in f and "filtered" in f]
        for f in current_frames:
            # Detect which camera the marker is sent from
            for c in range(self.n_cameras):
                if self.cameras[c] in f:
                    camera = c
                    break

            # Find tag name, camera frame, and pose
            fid = f[f.find("ar_marker_")+len("ar_marker")+1:len(f)]
            camera_link = "{}_link".format(self.cameras[camera])

            try:
                ret = tf_c.fromTf(self.tf_listen.lookupTransform(camera_link, f, rospy.Time(0)))
            except:
                print "Can't get TF camera->marker", camera_link, f
                continue

            # Check if we've seen this marker before
            if fid not in self.camera_marker_frame_names[camera].keys():
                self.camera_marker_frame_names[camera][fid] = {'name': f, 'count': 1}
                print "New marker in calibration:", camera,  fid, f
            else:
                self.camera_marker_frame_names[camera][fid]['count'] += 1

        # Check for overlapping frames
        marker_1_names = self.camera_marker_frame_names[0].keys()
        marker_2_names = self.camera_marker_frame_names[1].keys()
        overlapping_markers = list(set(marker_1_names).intersection(marker_2_names))
        n_overlapping_markers = len(overlapping_markers)
        print "---1---", self.camera_marker_frame_names[0]
        print "---2---", self.camera_marker_frame_names[1]

        # Check if camera 1 sees any fiducials
        if len(self.camera_marker_frame_names[0]) > 0:
            # if the other camera has seen a fiducial then make sure we're seeing the same one
            if all(self.cameras_calibrated):
                if n_overlapping_markers > 0:
                        self._widget.shoulder_status_label.setText('FOUND FIDUCIAL')
                        self._widget.shoulder_status_label.setStyleSheet('color:#ffffff;background-color:#9AD111')
                        self.cameras_calibrated[0] = True
                else:
                    self._widget.shoulder_status_label.setText("FIDUCIALS DON'T MATCH")
            # Else if the only marker seen then say 'found'
            else:
                self._widget.shoulder_status_label.setText('FOUND FIDUCIAL')
                self._widget.shoulder_status_label.setStyleSheet('color:#ffffff;background-color:#9AD111')
                self.cameras_calibrated[0] = True

        # Check if camera 2 sees any fiducials
        if len(self.camera_marker_frame_names[1]) > 0:
            # if the other camera has seen a fiducial then make sure we're seeing the same one
            if all(self.cameras_calibrated):
                n_overlapping_markers = len(set(marker_1_names).intersection(marker_2_names))
                if n_overlapping_markers > 0:
                    self._widget.wrist_status_label.setText('FOUND FIDUCIAL')
                    self._widget.wrist_status_label.setStyleSheet('color:#ffffff;background-color:#9AD111')
                    self.cameras_calibrated[1] = True
                else:
                    self._widget.wrist_status_label.setText("FIDUCIALS DON'T MATCH")
            # Else if the only marker seen then say 'found'
            else:
                self._widget.wrist_status_label.setText('FOUND FIDUCIAL')
                self._widget.wrist_status_label.setStyleSheet('color:#ffffff;background-color:#9AD111')
                self.cameras_calibrated[1] = True

        # Check if all cameras are calibrated
        if self.cameras_calibrated[0] and self.cameras_calibrated[1]:
            # First verify that the camera links (base to optical frames) are valid
            try:
                ret = self.tf_listen.lookupTransform("/base_link", "camera_1_rgb_optical_frame", rospy.Time(0))
                ret = self.tf_listen.lookupTransform("/base_link", "camera_2_rgb_optical_frame", rospy.Time(0))
            except:
                txt = "ROBOT NOT CONNECTED"
                self._widget.robot_calibration_status_label.setText(txt)
                return

            # Check that each camera has seen the same fiduual
            if n_overlapping_markers == 0:
                self._widget.robot_calibration_status_label.setText("FIDUCIALS DON'T MATCH")
                return

            # Get transform from robot base to camera 2
            """
            Want: World -> Cam2
            Have:
                Cam1->Marker1
                Cam2->Marker2
                Robot->Cam1
            Robot->Cam2 = (Robot->Cam1) * (Cam1->Marker1) * (Cam2->Marker2)^-1
            """

            # Find one of the markers seen by both cameras
            # print overlapping_markers, self.camera_marker_frame_names[0].keys(), self.camera_marker_frame_names[1].keys()
            marker_1_counts = [self.camera_marker_frame_names[0][x]['count'] for x in overlapping_markers]
            marker_2_counts = [self.camera_marker_frame_names[1][x]['count'] for x in overlapping_markers]
            overlap_count = [(marker_1_counts[i]+marker_2_counts[i])/2. for i in range(n_overlapping_markers)]
            ar_idx = overlapping_markers[np.argmax(overlap_count)]

            marker_1 = self.camera_marker_frame_names[0][ar_idx]['name']
            marker_2 = self.camera_marker_frame_names[1][ar_idx]['name']
            # print "Overlapping name:", marker_1, marker_2

            base_frame = '/base_link'
            try:
                T_base_camera_1 = tf_c.fromTf(self.tf_listen.lookupTransform(base_frame,'/camera_1_link',rospy.Time(0)))
            except:
                rospy.logwarn("Can't get TF robot->camera1")
                return

            try:
                T_camera_1_marker_1 = tf_c.fromTf(self.tf_listen.lookupTransform('/camera_1_link', marker_1, rospy.Time(0)))
            except:
                rospy.logwarn("Can't get TF camera1->marker")
                return
            try:
                T_camera_2_marker_2 = tf_c.fromTf(self.tf_listen.lookupTransform('/camera_2_link', marker_2, rospy.Time(0)))
            except:
                rospy.logwarn("Can't get TF camera2->marker")
                return

            T_base_camera_2 = T_base_camera_1*T_camera_1_marker_1*T_camera_2_marker_2.Inverse()

            # Extract position and rotation for the new transformation
            xyz = tf_c.toTf(T_base_camera_2)[0]
            rpy = T_base_camera_2.M.GetRPY()

            # Smooth out output frame to prevent large changes due to noise marker normals
            self.all_xyz += [xyz]
            self.all_rpy += [rpy]

            if len(self.all_xyz) > 2:
                xyz = np.mean(self.all_xyz, 0)
                rpy = np.mean(self.all_rpy, 0)
                # xyz = np.median(self.all_xyz, 0)
                # rpy = np.median(self.all_rpy, 0)
            else:
                return

            # Send the transform to the semi-static transformer
            # First check to see if service exists
            try:
                rospy.wait_for_service('/semi_static_transform_publisher/UpdateTransform')
            except rospy.ROSException as e:
                rospy.logerr('Could not find semi-static transform service')
                return
            # Make servo call to set pose
            try:
                # Setup transform to send to sem-static node
                update_transform_proxy = rospy.ServiceProxy('/semi_static_transform_publisher/UpdateTransform', UpdateTransform)
                msg = UpdateTransformRequest()
                msg.x, msg.y, msg.z = xyz
                msg.roll, msg.pitch, msg.yaw = rpy

                # Call to service
                update_transform_proxy(msg)
            except (rospy.ServiceException), e:
                rospy.logwarn('There was a problem with the service:')
                rospy.logwarn(e)

            # Update GUI
            self._widget.robot_calibration_status_label.setText('ROBOT CALIBRATED')
            self._widget.robot_calibration_status_label.setStyleSheet('color:#ffffff;background-color:#9AD111')
            self._widget.done_btn.setStyleSheet('color:#429611')
            self.calibrated = True

    def done_event(self):
        if not (self.cameras_calibrated[0] and self.cameras_calibrated[1]) or not self.calibrated:
            return
        self.update_poses = False
        self._widget.wrist_status_label.setStyleSheet('color:#ffffff;background-color:#CAF171')
        self._widget.shoulder_status_label.setStyleSheet('color:#ffffff;background-color:#CAF171')
        self._widget.robot_calibration_status_label.setStyleSheet('color:#ffffff;background-color:#CAF171')
        self._widget.done_btn.setStyleSheet('color:#999999')
        self._widget.wrist_camera_status.setStyleSheet('color:#999999;background-color:#dddddd')
        self._widget.shoulder_camera_status.setStyleSheet('color:#999999;background-color:#dddddd')

    def recalibrate_event(self):
        # Reset pose estimates
        self.all_xyz = []
        self.all_rpy = []
        # Reset each fiducial
        self.cameras_calibrated = []
        for c in range(self.n_cameras):
            self.camera_marker_frame_names[c] = {}
            self.cameras_calibrated += [False]
        self.calibrated = False

        # Reset GUI
        self._widget.wrist_status_label.setText('NO FIDUCIAL')
        self._widget.wrist_status_label.setStyleSheet('color:#ffffff;background-color:#F29A27')
        self._widget.shoulder_status_label.setText('NO FIDUCIAL')
        self._widget.shoulder_status_label.setStyleSheet('color:#ffffff;background-color:#F29A27')
        self._widget.robot_calibration_status_label.setText('ROBOT NOT CALIBRATED')
        self._widget.robot_calibration_status_label.setStyleSheet('color:#ffffff;background-color:#FF4D00')
        self._widget.done_btn.setStyleSheet('color:#999999')
        self._widget.wrist_camera_status.setStyleSheet('color:#555555;background-color:#ffffff')
        self._widget.shoulder_camera_status.setStyleSheet('color:#555555;background-color:#ffffff')

        # Stream data again
        self.update_poses = True

    def shutdown_plugin(self):
        # unregister all publishers here
        self.timer.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        pass
