#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import *
from threading import Thread
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
from service_node import ServiceNode
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c
# Driver services for ur5
import simple_ur_msgs
from simple_ur_msgs.srv import *

# Node Wrappers -----------------------------------------------------------
class Capability(Object):
    def __init__(self):
        self.functions = {}
        self.inputs = {}
        self.outputs = {}
        self.parameters = {}
        self.interfaces = {}

class WaypointUI(QWidget):
    def __init__(self):
        super(WaypointUI,self).__init__()

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/action_waypoint.ui'

        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.scrollArea = QScrollArea()
        self.scrollArea.setWidget(self.waypoint_ui)
        self.layout_.addWidget(self.scrollArea)

        self.new_waypoint_name = None
        self.waypoint_selected = False
        self.command_waypoint_name = None
        self.command_vel = .75
        self.command_acc = .75
        self.listener_ = tf.TransformListener()

        self.waypoint_ui.waypoint_name_field.textChanged.connect(self.waypoint_name_entered)
        self.waypoint_ui.waypoint_added_btn.clicked.connect(self.add_waypoint)
        self.waypoint_ui.waypoint_remove_btn.clicked.connect(self.remove_waypoint)

        self.update_waypoints()

    def update_waypoints(self):
        rospy.wait_for_service('/instructor_core/GetWaypointList')
        found_waypoints = []
        self.waypoint_ui.waypoint_list.clear()
        try:
            get_waypoints_proxy = rospy.ServiceProxy('/instructor_core/GetWaypointList',GetWaypointList)
            found_waypoints = get_waypoints_proxy('').names
        except rospy.ServiceException, e:
            rospy.logerr(e)
        for w in found_waypoints:
            self.waypoint_ui.waypoint_list.addItem(QListWidgetItem(w.strip('/')))

    def add_waypoint(self):
        if self.new_waypoint_name:
            try:
                F_waypoint = tf_c.fromTf(self.listener_.lookupTransform('/world','/endpoint',rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr('Could not find the tf frame for the robot endpoint')
                return
            try:
                rospy.wait_for_service('/instructor_core/AddWaypoint',2)
            except rospy.ROSException as e:
                rospy.logerr(e)
                return
            try:
                add_waypoint_proxy = rospy.ServiceProxy('/instructor_core/AddWaypoint',AddWaypoint)
                msg = AddWaypointRequest()
                msg.name = '/' + self.new_waypoint_name
                msg.world_pose = tf_c.toMsg(F_waypoint)
                rospy.loginfo(add_waypoint_proxy(msg))
                self.update_waypoints()
            except rospy.ServiceException, e:
                rospy.logerr(e)
        else:
            rospy.logerr('You need to input a name for the waypoint')

    def remove_waypoint(self):
        current_selected_name = self.waypoint_ui.waypoint_list.currentItem().text()
        if current_selected_name:
            try:
                rospy.wait_for_service('/instructor_core/RemoveWaypoint',2)
            except rospy.ROSException as e:
                rospy.logerr(e)
                return
            try:
                remove_waypoint_proxy = rospy.ServiceProxy('/instructor_core/RemoveWaypoint',RemoveWaypoint)
                msg = RemoveWaypointRequest()
                msg.name = str('/' + current_selected_name)
                rospy.loginfo(remove_waypoint_proxy(msg))
                self.update_waypoints()
            except rospy.ServiceException, e:
                rospy.logerr(e)

    def waypoint_name_entered(self,t):
        self.new_waypoint_name = str(t)

# Nodes -------------------------------------------------------------------
class NodeActionWaypoint(ServiceNode):
    def __init__(self,name,label,waypoint_name,vel,acc):
        super(NodeActionWaypoint,self).__init__(name,label,'#26A65B',"Waypoint Service",display_name=waypoint_name)
        self.command_waypoint_name = waypoint_name
        self.command_vel = vel
        self.command_acc = acc
        self.listener_ = tf.TransformListener()

    def make_service_call(self,request,*args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('/simple_ur_msgs/ServoToPose')
        except rospy.ROSException as e:
            rospy.logerr('Could not find servo service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            pose_servo_proxy = rospy.ServiceProxy('/simple_ur_msgs/ServoToPose',ServoToPose)
            
            F_command_world = tf_c.fromTf(self.listener_.lookupTransform('/world', '/'+self.command_waypoint_name, rospy.Time(0)))
            F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
            F_command = F_base_world.Inverse()*F_command_world
                
            msg = simple_ur_msgs.srv.ServoToPoseRequest()
            msg.target = tf_c.toMsg(F_command)
            msg.vel = self.command_vel
            msg.accel = self.command_acc
            # Send Servo Command
            rospy.loginfo('Single Servo Move Started')
            result = pose_servo_proxy(msg)
            if 'FAILURE' in str(result.ack):
                rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.loginfo('Single Servo Move Finished')
                rospy.logwarn('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ServiceException), e:
            rospy.logwarn('There was a problem with the tf lookup or service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return