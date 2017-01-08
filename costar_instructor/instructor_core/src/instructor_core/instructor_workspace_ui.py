#!/usr/bin/env python
import roslib
roslib.load_manifest('instructor_core')
import rospy
import rospkg
import rosparam
import tf

from predicator_msgs.msg import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QPalette
from python_qt_binding.QtCore import Qt
import QtCore

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class WorkspaceUI(Plugin):

    use_filtered = False
    marker_frames = {}
    markers_seen = []
    current_unknown_marker = None
    # server = InteractiveMarkerServer("Landmarks")
    pub_markers = rospy.Publisher('visualization_marker_array', MarkerArray)
    markerArray = MarkerArray()

    def __init__(self, context):
        super(WorkspaceUI, self).__init__(context)

        self.setObjectName('Instructor Workspace UI')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which is a sibling of this file
        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_core') + '/ui/workspace.ui'

        # Load the ui attributes into the main widget
        loadUi(ui_path, self._widget)
        self._widget.setObjectName('InstructorWorkspaceUI')
        self._widget.setWindowTitle('Instructor Workspace UI')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            title = self._widget.windowTitle() + (' (%d)' % context.serial_number())
            self._widget.setWindowTitle(title)

        # Add widget to the user interface
        context.add_widget(self._widget)
        palette = QPalette()
        palette.setColor(QPalette.Background, Qt.white)
        self._widget.setPalette(palette)

        # Hide things to start with
        self._widget.fiducial_found_widget.hide()
        self._widget.add_fiducial_widget.hide()

        # Connect things
        self._widget.add_ok_btn.clicked.connect(self.add_ok_event)
        self._widget.add_cancel_btn.clicked.connect(self.add_cancel_event)
        self._widget.fiducial_name_edit.textChanged.connect(self.name_entered_event)

        # Add listener to detect AR markers
        self.tf_listen = tf.TransformListener()

        # Add predicator publisher
        self.pub_predicates = rospy.Publisher('predicator/input', PredicateList)

        # Clean params for landmarks
        rospy.loginfo('Cleaning up previous workspace landmark params')
        params = rospy.get_param_names()
        landmarks = [p for p in params if "instructor_landmark" in p]
        rospy.loginfo(landmarks)
        for marker in landmarks:
            rospy.loginfo('Removed '+str(marker))
            rospy.delete_param(marker)

        # Add timer
        self.timer = QtCore.QTimer()
        self.connect(self.timer, QtCore.SIGNAL("timeout()"), self.update)
        self.timer.start(100)

    # UI Methods ------------------------------------------------------------------#
    def new_marker(self, name, pose):
        """ Add new marker to the scene """

        # Check if previous marker with this name
        if self.get_marker_id(name) is not None:
            return

        # Setup marker
        marker = Marker()
        marker.header.frame_id = "/base_link"
        marker.text = name
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.x = pose[1][0]
        marker.pose.orientation.y = pose[1][1]
        marker.pose.orientation.z = pose[1][2]
        marker.pose.orientation.w = pose[1][3]
        marker.pose.position.x = pose[0][0]
        marker.pose.position.y = pose[0][1]
        marker.pose.position.z = pose[0][2]

        # Add ID number. Use name if it's a number
        try:
            marker.id = int(name)
        except:
            max_id = len(self.markerArray.markers)
            marker.id = max_id

        # Append and publish marker
        self.markerArray.markers.append(marker)
        self.pub_markers.publish(self.markerArray)

    def get_marker_id(self, name):
        # Get all marker names that match "name". Then output its ID
        markers = [i for i, m in enumerate(self.markerArray.markers) if m.text == name]
        if len(markers) == 0:
            return None
        else:
            return markers[0]

    def remove_marker(self, name):
        idx = self.get_marker_id(name)
        if idx is None:
            print "Couldn't remove marker"
            return

        self.markerArray.markers[idx].action = Marker.DELETE
        self.pub_markers.publish(self.markerArray)
        self.markerArray.markers.pop(idx)

    def recolor_marker(self, name, color):
        # Get index of named marker
        idx = self.get_marker_id(name)
        if idx is None:
            print "Couldn't recolor node:", name
            return

        # Recolor
        self.markerArray.markers[idx].color.r = color[0]
        self.markerArray.markers[idx].color.g = color[1]
        self.markerArray.markers[idx].color.b = color[2]
        self.markerArray.markers[idx]

        self.pub_markers.publish(self.markerArray)

    def update_marker_pose(self, name):
        # Get index of named marker
        fid = self.id_from_frame(name)
        idx = self.get_marker_id(fid)
        if idx is None:
            rospy.loginfo("Can't update marker pose #{} [{}]".format(fid, name))
            return

        try:
            # Update the marker and make sure it's displayed
            pose = self.tf_listen.lookupTransform("base_link", name, rospy.Time(0))
            self.markerArray.markers[idx].action = Marker.ADD
        except:
            # Use empty pose and delete the marker
            pose = ((0, 0, 0), (0, 0, 0, 0))
            self.markerArray.markers[idx].action = Marker.DELETE
            rospy.loginfo("Can't update marker [{}] pose".format(name))

        # Update pose
        self.markerArray.markers[idx].pose.position.x = pose[0][0]
        self.markerArray.markers[idx].pose.position.y = pose[0][1]
        self.markerArray.markers[idx].pose.position.z = pose[0][2]
        self.markerArray.markers[idx].pose.orientation.x = pose[1][0]
        self.markerArray.markers[idx].pose.orientation.y = pose[1][1]
        self.markerArray.markers[idx].pose.orientation.z = pose[1][2]
        self.markerArray.markers[idx].pose.orientation.w = pose[1][3]

        # Publish all current markers
        self.pub_markers.publish(self.markerArray)

    def id_from_frame(self, name):
        """ Go from name (e.g. /camera_1/ar_marker_0) to id (e.g. 0) """
        return name[name.find("ar_marker_")+len("ar_marker")+1:len(name)]

    def update(self):
        # Get all current TF frames and filter
        current_frames = self.tf_listen.getFrameStrings()
        # Only get alvar messages...
        current_frames = filter(lambda x: "ar_marker" in x, current_frames)
        # That you haven't seen before...
        current_frames = filter(lambda x: self.id_from_frame(x) not in self.markers_seen, current_frames)
        print "c Frames", current_frames
        # That are (un)filtered
        if self.use_filtered:
            current_frames = filter(lambda x: "filtered" in x, current_frames)
        else:
            current_frames = filter(lambda x: not "filtered" in x, current_frames)

        print "e Frames", current_frames
        # Update all marker poses
        for frame in self.marker_frames.keys():
            self.update_marker_pose(frame)

        # Update the active marker
        if self.current_unknown_marker is not None:
        # Cancel current marker if we don't see the current frames
            if self.current_unknown_marker not in current_frames:
                self.add_cancel_event()
            else:            
                # Otherwise update the pose
                self.update_marker_pose(self.current_unknown_marker)
                # Don't create new active marker if we already have one
                return

        # Get the first frame and add new marker
        if len(current_frames) > 0:
            frame = current_frames[0]
            fid = self.id_from_frame(frame)
            rospy.loginfo("NEW workspace marker #{} [{}]".format(fid, frame))
            self.current_unknown_marker = frame
            self.fiducial_found_event(frame)

    def fiducial_found_event(self, frame):
        fid = self.id_from_frame(frame)

        # Edit text displayed to use the marker id
        show_name_text = "Found fiducial with ID: {}".format(frame)
        self._widget.found_fiducial_id_label_2.setText(show_name_text)

        # Get marker position
        try:
            pose = self.tf_listen.lookupTransform("/base_link", frame, rospy.Time(0))
        except:
            rospy.loginfo("Can't find marker pose [{}]. Canceling current marker".format(frame))
            return

        self.new_marker(fid, pose)

        # GUI: Show the "name" field
        self._widget.fiducial_found_widget.show()

    def name_entered_event(self, val):
        self._widget.add_fiducial_widget.show()

    def add_ok_event(self):
        # Get name for this marker and add to set
        marker_name = self._widget.fiducial_name_edit.text()
        frame = self.current_unknown_marker
        fid = self.id_from_frame(frame)

        self.current_unknown_marker = None
        rospy.loginfo("Adding marker #{} [{}]".format(fid, frame))

        # Add to "seen" list
        self.markers_seen += [fid]
        self.marker_frames[frame] = marker_name

        # Update visualized marker
        self.recolor_marker(fid, [0.0, 1.0, 0.0])

        # Add marker to rosparam
        rosparam_marker_name = "instructor_landmark/{}".format(marker_name)
        rosparam.set_param(rosparam_marker_name, frame)

        # GUI: Append to the list box
        list_txt = "{} ({})".format(marker_name, frame)
        self._widget.fiducial_list.addItem(list_txt)

        # GUI: Reset text field
        self._widget.fiducial_name_edit.setText("")
        self._widget.found_fiducial_id_label_2.setText("Found fiducial with ID: NULL")

        # GUI Remove the "edit name" section of the gui
        self._widget.fiducial_found_widget.hide()
        self._widget.add_fiducial_widget.hide()

    def add_cancel_event(self):
        # Remove viz markers
        frame = self.current_unknown_marker
        fid = self.id_from_frame(frame)
        rospy.loginfo("REMOVING marker #{} [{}]".format(fid, frame))

        # GUI: Reset name/field
        self._widget.fiducial_name_edit.setText("")
        self._widget.found_fiducial_id_label_2.setText("Found fiducial with ID: NULL")

        # GUI: Remove "edit name" section of the gui
        self._widget.fiducial_found_widget.hide()
        self._widget.add_fiducial_widget.hide()

        # Remove rviz marker
        self.remove_marker(fid)

        # Add to "seen" list
        self.current_unknown_marker = None
        self.markers_seen += [fid]

# RQT Plugin Stuff ------------------------------------------------------------#

    def shutdown_plugin(self):
        # unregister all publishers here
        self.pub_predicates.unregister()
        self.pub_markers.unregister()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        pass
