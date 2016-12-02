#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy, sys
from std_msgs.msg import * 
from sensor_msgs.msg import * 
from geometry_msgs.msg import *
from razer_hydra.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
import tf; from tf import *
import tf_conversions as tf_c
import PyKDL
import numpy as np
import rosbag

import interactive_panes
from interactive_panes.msg import *
from interactive_panes import *

class TestVis():

  def __init__(self):
    rospy.init_node('test_vis')
    self.listener_ = tf.TransformListener()
    self.broadcaster_ = tf.TransformBroadcaster()
    rospy.logwarn("TestVis: starting up")
    rospy.logwarn("TestVis: Interfaces Initializing")

    rospy.sleep(.5)

    self.marker_server = InteractiveMarkerServer("adjutant_module_grab_teleop")

    # Panes
    # self.pane_manager = PaneManager()
    # self.menu_button_state = {}
    # self.create_panes()
    self.set_text_marker()
    self.set_endpoint_marker_inactive()

    self.initialize_frames

    ### Main Loop
    r = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
      self.update()
      r.sleep()

  def initialize_frames(self):
    try:
      self.endpoint = tf_c.fromTf(self.listener_.lookupTransform('/world', '/endpoint', rospy.Time(0)))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      rospy.logwarn(str(e))

  def update(self):
    pass

  def create_panes(self):
    q = PyKDL.Rotation.RPY(0,0,0)
    Q = Quaternion(*q.GetQuaternion())

    test_pane = Pane()
    test_pane.name = 'Test'
    test_pane.position = Point(0,0,0)
    test_pane.orientation = Q
    test_pane.scale = Point(.125,0,.125)
    test_pane.attached_frame = '/endpoint'
    test_pane.pane_type = 'BUTTON'
    test_pane.text = 'Test'
    test_pane.style_idle = 'color:#333333;background-color:#FF4640'
    test_pane.style_active = 'color:#333333;background-color:#7C44CC'
    test_pane.font_size = 70

    self.pane_manager.create_group('TestGroup')
    self.pane_manager.add_to_group('TestGroup',test_pane,self.btn_events)
    self.menu_button_state['Test'] = None
    self.pane_manager.apply_changes()
    self.pane_manager.change_group_visibility('TestGroup',True)

  def btn_events(self,name,event):
    self.menu_button_state[name] = event

  def processFeedback(self, feedback):
    pass

  def set_text_marker(self):
    self.marker_server.erase('text_interaction_marker')
    text_marker = InteractiveMarker()
    text_marker.header.frame_id = "/endpoint"
    text_marker.pose = Pose()
    text_marker.scale = 1
    text_marker.name = "text_interaction_marker"
    # text_marker.description = 'some text'
    # create mesh marker for displaying mesh
    self.text_marker = Marker()
    self.text_marker.type = Marker.TEXT_VIEW_FACING
    self.text_marker.color.r = 1.0
    self.text_marker.color.g = 1.0
    self.text_marker.color.b = 1.0
    self.text_marker.color.a = 1.0
    self.text_marker.scale = Point(.03,.03,.03)
    self.text_marker.text = 'POSE'
    self.text_marker.pose = Pose(Point(0,0,.075),Quaternion())
    # create a non-interactive control containing the mesh marker
    text_control = InteractiveMarkerControl()
    text_control.always_visible = True
    text_control.interaction_mode = InteractiveMarkerControl.NONE
    text_control.markers.append( self.text_marker )
    text_marker.controls.append( text_control )
    self.marker_server.insert(text_marker, self.processFeedback)
    self.marker_server.applyChanges()

  def set_endpoint_marker_inactive(self):
    self.marker_server.erase('endpoint_interaction_marker')
    endpoint_i_marker = InteractiveMarker()
    endpoint_i_marker.header.frame_id = "/endpoint"
    endpoint_i_marker.pose = Pose()
    endpoint_i_marker.scale = .1
    endpoint_i_marker.name = "endpoint_interaction_marker"
    # create mesh marker for displaying mesh
    self.endpoint_marker = Marker()
    self.endpoint_marker.type = Marker.SPHERE
    self.endpoint_marker.color.r = 0.0
    self.endpoint_marker.color.g = 0.9
    self.endpoint_marker.color.b = 1.0
    self.endpoint_marker.color.a = 1.0
    self.endpoint_marker.scale = Point(.1,.1,.1)
    # create a non-interactive control containing the mesh marker
    endpoint_control = InteractiveMarkerControl()
    endpoint_control.always_visible = True
    endpoint_control.interaction_mode = InteractiveMarkerControl.NONE
    endpoint_control.markers.append( self.endpoint_marker )
    endpoint_i_marker.controls.append( endpoint_control )
    self.marker_server.insert(endpoint_i_marker, self.processFeedback)
    self.marker_server.applyChanges()

  def hide_text_marker(self):
    self.marker_server.erase('text_interaction_marker')
    self.marker_server.applyChanges()

# MAIN
if __name__ == '__main__':
  nav = TestVis()