#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
import rospkg
from beetree import *
import instructor_core
from instructor_core.srv import *
import os,sys, inspect, ast
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
import rospkg
import tf; 
import tf_conversions as tf_c
from instructor_core.instructor_qt import *
# Using roslib.rospack even though it is deprecated
import threading
from roslib import rospack
import yaml
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import time
from copy import deepcopy
import costar_robot_msgs
from costar_robot_msgs.srv import *
from instructor_gui_components import *
from smart_waypoint_manager import SmartWaypointManager
from std_srvs.srv import Empty

### TIMER ###################################################
global tic_time
def tic():
    global tic_time
    tic_time = time.time()
def toc():
    global tic_time
    elapsed = time.time() - tic_time ############## TIMER
    rospy.logerr(elapsed) ################## TIMER

### TIMER ###################################################

color_options = ColorOptions()
colors = color_options.colors

### GUI ELEMENTS --------------------------------------------------------- ### 
class SmartMoveDialog(QWidget):
    def __init__(self, show_hide_fn,parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        # GUI
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/smart_move.ui'
        uic.loadUi(w_path, self)
        self.show_hide = show_hide_fn
        self.selected_object = None
        self.selected_move = None
        self.saved_geom = None
        self.new_move_name = None

        self.manager = SmartWaypointManager(ns="")
        
        self.done_button.clicked.connect(self.done)
        self.name_field.textChanged.connect(self.move_name_cb)
        self.object_list.itemClicked.connect(self.object_selected)
        self.move_list.itemClicked.connect(self.move_selected)

        self.add_move_button.clicked.connect(self.add_move)
        self.delete_move_button.clicked.connect(self.delete_move)

        self.update_objects()
        self.update_moves()

    def update_all(self):
        self.update_objects()
        self.update_moves()

    def done(self):
        self.show_hide()

    def move_name_cb(self,text):
        self.new_move_name = str(text)
        self.update_objects()

    def object_selected(self,item):
        self.selected_object = str(item.text())
        self.object_field.setText(self.selected_object)
        self.update_moves()

    def move_selected(self,item):
        # self.update_objects()
        self.selected_move = str(item.text())

    def update_objects(self):
        self.object_list.clear()
        self.found_objects = self.manager.get_detected_objects()
        # Populate objects in list
        if self.found_objects is not None:
            for m in self.found_objects:
                self.object_list.addItem(QListWidgetItem(m.strip('/')))
            self.object_list.sortItems()
            self.object_list.setCurrentRow(0)

    def update_moves(self):
        if self.selected_object is not None:
            self.move_list.clear()

            # Populate moves in list
            self.manager.load_all()
            self.found_moves = self.manager.get_moves_for_object(self.selected_object)
            if self.found_moves is not None:
                for m in self.found_moves:
                    self.move_list.addItem(QListWidgetItem(m.strip('/')))
                self.move_list.sortItems()
                self.move_list.setCurrentRow(0)

    def add_move(self):
        self.update_objects()
        print self.new_move_name
        if self.new_move_name is not None and self.selected_object is not None:
            # librarian call to add a new move with new_move_name
            self.manager.save_new_waypoint(self.selected_object,self.new_move_name)
            # Update moves
            self.update_moves()
            pass

    def delete_move(self):
        self.update_objects()
        print self.selected_move
        if self.selected_move is not None:
            # librarian call to remove move for selected class
            self.manager.delete(self.selected_move)

            # Update moves
            self.update_moves()
        pass


class JogDialog(QWidget):
    def __init__(self, show_hide_fn,parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        # GUI
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/jog_buttons.ui'
        uic.loadUi(w_path, self)
        # Vars
        self.jog_publisher = rospy.Publisher('/ur_robot/jog',JointState)
        self.show_hide = show_hide_fn 
        self.saved_geom = None
        self.jogging = False
        self.jog_direction = None
        self.step = .01
        self.rot_step = .05
        self.speed = 'low'
        #Signals
        self.btn_ok.clicked.connect(self.ok)

        self.btn_roll_left.pressed.connect(self.start_jog)
        self.btn_roll_right.pressed.connect(self.start_jog)
        self.btn_yaw_left.pressed.connect(self.start_jog)
        self.btn_yaw_right.pressed.connect(self.start_jog)
        self.btn_pitch_up.pressed.connect(self.start_jog)
        self.btn_pitch_down.pressed.connect(self.start_jog)
        self.btn_roll_left.released.connect(self.stop_jog)
        self.btn_roll_right.released.connect(self.stop_jog)
        self.btn_yaw_left.released.connect(self.stop_jog)
        self.btn_yaw_right.released.connect(self.stop_jog)
        self.btn_pitch_up.released.connect(self.stop_jog)
        self.btn_pitch_down.released.connect(self.stop_jog)

        self.btn_plus_x.pressed.connect(self.start_jog)
        self.btn_plus_y.pressed.connect(self.start_jog)
        self.btn_plus_z.pressed.connect(self.start_jog)
        self.btn_minus_x.pressed.connect(self.start_jog)
        self.btn_minus_y.pressed.connect(self.start_jog)
        self.btn_minus_z.pressed.connect(self.start_jog)
        self.btn_plus_x.released.connect(self.stop_jog)
        self.btn_plus_y.released.connect(self.stop_jog)
        self.btn_plus_z.released.connect(self.stop_jog)
        self.btn_minus_x.released.connect(self.stop_jog)
        self.btn_minus_y.released.connect(self.stop_jog)
        self.btn_minus_z.released.connect(self.stop_jog)
        self.btn_speed.clicked.connect(self.toggle_speed)

    def ok(self):
        self.stop_jog()
        self.show_hide()

    def toggle_speed(self):
        if self.speed == 'low':
            self.speed = 'high'
            self.step = .05
            self.rot_step = .1
            self.btn_speed.setText('JOG SPEED: HIGH')
            self.btn_speed.setStyleSheet('''QPushButton#btn_speed { background-color:#F22613;color:#ffffff;border:none }QPushButton#btn_speed:hover:!pressed{ background-color:#F22613;color:#ffffff;border:none }QPushButton#btn_speed:hover { background-color:#F22613;color:#ffffff;border:none }''')
        else:
            self.speed = 'low'
            self.step = .01
            self.rot_step = .05
            self.btn_speed.setText('JOG SPEED: LOW')
            self.btn_speed.setStyleSheet('''QPushButton#btn_speed { background-color:#26A65B;color:#ffffff;border:none }QPushButton#btn_speed:hover:!pressed{ background-color:#26A65B;color:#ffffff;border:none }QPushButton#btn_speed:hover { background-color:#26A65B;color:#ffffff;border:none }''')

    def stop_jog(self):
        sender = str(self.sender().objectName())
        P = JointState()
        P.header.stamp = rospy.get_rostime()
        P.header.frame_id = "jog_offset"
        P.position = [0]*6
        P.velocity = [0]*6
        P.effort = [0]*6
        self.jog_publisher.publish(P)
        rospy.logwarn('Sent '+ str(P.velocity))
        rospy.logwarn('Jog Finished ['+sender+']')
        self.jogging = False

    def start_jog(self):
        if not self.jogging:
            sender = str(self.sender().objectName())

            P = JointState()
            P.header.stamp = rospy.get_rostime()
            P.header.frame_id = "jog_offset"
            P.position = [0]*6
            P.velocity = [0]*6
            P.effort = [0]*6
            if sender == str("btn_plus_x"):
                P.velocity[0] += self.step
            elif sender == str("btn_plus_y"):
                P.velocity[1] += self.step
            elif sender == str("btn_plus_z"):
                P.velocity[2] += self.step
            elif sender == str("btn_minus_x"):
                P.velocity[0] -= self.step
            elif sender == str("btn_minus_y"):
                P.velocity[1] -= self.step
            elif sender == str("btn_minus_z"):
                P.velocity[2] -= self.step
                    
            elif sender == str("btn_pitch_up"):
                P.velocity[4] -= self.rot_step
            elif sender == str("btn_pitch_down"):
                P.velocity[4] += self.rot_step

            elif sender == str("btn_roll_left"):
                P.velocity[3] -= self.rot_step
            elif sender == str("btn_roll_right"):
                P.velocity[3] += self.rot_step
                
            elif sender == str("btn_yaw_left"):
                P.velocity[5] -= self.rot_step
            elif sender == str("btn_yaw_right"):
                P.velocity[5] += self.rot_step
            

            self.jog_publisher.publish(P)
            rospy.logwarn('Jog Started ['+sender+']')
            rospy.logwarn('Sent '+ str(P.velocity))
            self.jogging = True
       

class WaypointManagerDialog(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        # GUI
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_plugins') + '/ui/waypoint_manager_2.ui'
        uic.loadUi(w_path, self)
        self.waypoint_page_widget = QWidget()
        self.relative_page_widget = QWidget()
        waypoint_path = rp.get_path('instructor_plugins') + '/ui/waypoint_inner_frame.ui'
        uic.loadUi(waypoint_path, self.waypoint_page_widget)
        relative_path = rp.get_path('instructor_plugins') + '/ui/relative_inner_frame.ui'
        uic.loadUi(relative_path, self.relative_page_widget)
        self.waypoint_layout.addWidget(self.waypoint_page_widget)
        self.relative_layout.addWidget(self.relative_page_widget)

        self.waypoint_page_btn = InterfaceButton('FIXED WAYPOINTS',colors['blue'])
        self.waypoint_page_btn.clicked.connect(self.show_waypoint_page)
        self.relative_page_btn = InterfaceButton('RELATIVE WAYPOINTS',colors['green'])
        self.relative_page_btn.clicked.connect(self.show_relative_page)
        self.page_button_layout.addWidget(self.waypoint_page_btn)
        self.page_button_layout.addWidget(self.relative_page_btn)

        # BUTTONS #
        self.add_waypoint_btn = InterfaceButton('ADD WAYPOINT',colors['blue'])
        self.button_layout.addWidget(self.add_waypoint_btn)
        self.add_waypoint_btn.clicked.connect(self.add_waypoint)

        self.add_waypoint_seq_btn = InterfaceButton('ADD WAYPOINT SEQUENCE',colors['blue'])
        self.button_layout.addWidget(self.add_waypoint_seq_btn)
        self.add_waypoint_seq_btn.clicked.connect(self.add_waypoint_seq)

        self.add_waypoint_btn = InterfaceButton('REMOVE WAYPOINT',colors['red'])
        self.button_layout.addWidget(self.add_waypoint_btn)
        self.add_waypoint_btn.clicked.connect(self.remove_waypoint)

        self.servo_to_waypoint_btn = InterfaceButton('SERVO TO WAYPOINT',colors['purple'])
        self.button_layout.addWidget(self.servo_to_waypoint_btn)
        self.servo_to_waypoint_btn.clicked.connect(self.servo_to_waypoint)

        self.waypoint_page_widget.name_field.textChanged.connect(self.waypoint_name_cb)
        self.relative_page_widget.name_field.textChanged.connect(self.waypoint_name_cb)

        self.relative_page_widget.landmark_list.itemClicked.connect(self.landmark_selected)
        self.relative_page_widget.landmark_field.setText('NONE')
        self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['gray'].normal+';color:#ffffff')
        
        # Members
        self.landmarks = {}
        self.new_fixed_name = None
        self.new_relative_name = None
        self.selected_landmark = None
        self.waypoint_selected = False
        self.listener_ = tf.TransformListener()
        self.mode = 'FIXED'
        self.found_waypoints = []
        self.found_rel_waypoints = []
        # Initialize
        self.update_waypoints()

    def show_waypoint_page(self):
        self.stack.setCurrentIndex(0)
        self.mode = 'FIXED'
        self.update_waypoints()

    def show_relative_page(self):
        self.stack.setCurrentIndex(1)
        self.mode = 'RELATIVE'
        self.update_waypoints()

    def waypoint_name_cb(self,t):
        if self.mode == 'FIXED':
            self.new_fixed_name = str(t)
        else:
            self.new_relative_name = str(t)

    def landmark_selected(self,item):
        if item == None:
            self.relative_page_widget.landmark_field.setText('NONE')
            self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['gray'].normal+';color:#ffffff')
            self.selected_landmark = None
        else:
            text = str(item.text())
            self.relative_page_widget.landmark_field.setText(text.upper())
            self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['green'].hover+';color:#ffffff')
            self.selected_landmark = text

    def add_waypoint(self):
        if self.mode == 'FIXED':
            if self.new_fixed_name:
                # Check for sequence waypoints
                for w in self.found_waypoints:
                    if '--' in w:
                        if self.new_fixed_name == w.split('--')[0].replace('/',''):
                            rospy.logerr('YOU MUST PICK A NAME THAT IS NOT A SEQUENCE')
                            return
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
                    msg.name = '/' + self.new_fixed_name
                    msg.world_pose = tf_c.toMsg(F_waypoint)
                    rospy.loginfo(add_waypoint_proxy(msg))
                    self.update_waypoints()
                    self.waypoint_page_widget.name_field.setText('')
                except rospy.ServiceException, e:
                    rospy.logwarn(e)
            else:
                rospy.logerr('You need to input a name for the waypoint')
        else:
            rospy.logwarn('Adding Relative Waypoint with landmark ['+str(self.selected_landmark)+'] and waypoint ['+str(self.new_relative_name)+']')
            if self.selected_landmark != None:
                landmark_frame = self.landmarks[str(self.selected_landmark)].strip('/')
                rospy.logwarn(landmark_frame)
                try:
                    F_landmark = tf_c.fromTf(self.listener_.lookupTransform('/world',landmark_frame,rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr('Selected landmark not found')
                    return                
            else:
                rospy.logerr('You did not select a landmark')
                return
            if not self.new_relative_name:
                rospy.logwarn('You did not enter a waypoint name')
                return

            # Get relative pose
            try:
                F_landmark_endpoint = tf_c.fromTf(self.listener_.lookupTransform('/'+landmark_frame,'/endpoint',rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return

            try:
                rospy.wait_for_service('/instructor_core/AddWaypoint',2)
            except rospy.ROSException as e:
                rospy.logerr(e)
                return

            for w in self.found_rel_waypoints:
                if '--' in w:
                    if self.new_relative_name == w.split('--')[0].replace('/',''):
                        rospy.logerr('YOU MUST PICK A NAME THAT IS NOT A SEQUENCE')
                        return
            try:
                add_waypoint_proxy = rospy.ServiceProxy('/instructor_core/AddWaypoint',AddWaypoint)
                msg = AddWaypointRequest()
                msg.name = '/' + self.new_relative_name
                msg.relative_pose = tf_c.toMsg(F_landmark_endpoint)
                msg.relative_frame_name = '/'+landmark_frame
                rospy.loginfo(add_waypoint_proxy(msg))
                self.update_waypoints()
                # Reset
                self.relative_page_widget.name_field.setText('')
                self.relative_page_widget.landmark_field.setText('NONE')
                self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['gray'].normal+';color:#ffffff')
                self.selected_landmark = None
            except rospy.ServiceException, e:
                rospy.logwarn(e)

    def add_waypoint_seq(self):
        if self.mode == 'FIXED':
            if self.new_fixed_name:

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

                # Check for non-sequence waypoints with sequence name
                for w in self.found_waypoints:
                    if '--' not in w:
                        if self.new_fixed_name == w.replace('/',''):
                            rospy.logerr('YOU MUST PICK A UNIQUE NAME FOR A SEQUENCE')
                            return
                            
                # Look for existing sequence and get index if present
                wp = [w.split('--')[0].replace('/','') for w in self.found_waypoints]
                index = 0
                rospy.logwarn('wp: '+str(wp))
                rospy.logwarn('name: '+self.new_fixed_name)
                if self.new_fixed_name in wp:
                    rospy.logwarn('name found... ' + self.new_fixed_name)
                    index = -1
                    for w in self.found_waypoints:
                        if self.new_fixed_name in w:
                            if int(w.split('--')[1]) > index:
                                index = int(w.split('--')[1])
                    index = index + 1

                try:
                    add_waypoint_proxy = rospy.ServiceProxy('/instructor_core/AddWaypoint',AddWaypoint)
                    msg = AddWaypointRequest()
                    msg.name = '/' + self.new_fixed_name + '--' + str(index)
                    msg.world_pose = tf_c.toMsg(F_waypoint)
                    rospy.loginfo(add_waypoint_proxy(msg))
                    self.update_waypoints()
                    # self.waypoint_page_widget.name_field.setText('')
                except rospy.ServiceException, e:
                    rospy.logwarn(e)
            else:
                rospy.logerr('You need to input a name for the waypoint')
        else:
            rospy.logwarn('Adding Relative Waypoint with landmark ['+str(self.selected_landmark)+'] and waypoint ['+str(self.new_relative_name)+']')
            if self.selected_landmark != None:
                landmark_frame = self.landmarks[str(self.selected_landmark)].strip('/')
                rospy.logwarn(landmark_frame)
                try:
                    F_landmark = tf_c.fromTf(self.listener_.lookupTransform('/world',landmark_frame,rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logerr('Selected landmark not found')
                    return                
            else:
                rospy.logerr('You did not select a landmark')
                return
            if not self.new_relative_name:
                rospy.logwarn('You did not enter a waypoint name')
                return

            # Check for non-sequence waypoints with sequence name
            for w in self.found_rel_waypoints:
                if '--' not in w:
                    if self.new_relative_name == w.replace('/',''):
                        rospy.logerr('YOU MUST PICK A UNIQUE NAME FOR A SEQUENCE')
                        return
                                        # Look for existing sequence and get index if present
            wp = [w.split('--')[0].replace('/','') for w in self.found_rel_waypoints]
            index = 0
            if self.new_relative_name in wp:
                index = -1
                for w in self.found_rel_waypoints:
                    if self.new_relative_name in w:
                        if int(w.split('--')[1]) > index:
                            index = int(w.split('--')[1])
                index = index + 1

            # Get relative pose
            try:
                F_landmark_endpoint = tf_c.fromTf(self.listener_.lookupTransform('/'+landmark_frame,'/endpoint',rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr(e)
                return

            try:
                rospy.wait_for_service('/instructor_core/AddWaypoint',2)
            except rospy.ROSException as e:
                rospy.logerr(e)
                return
            try:
                add_waypoint_proxy = rospy.ServiceProxy('/instructor_core/AddWaypoint',AddWaypoint)
                msg = AddWaypointRequest()
                msg.name = '/' + self.new_relative_name + '--' + str(index)
                msg.relative_pose = tf_c.toMsg(F_landmark_endpoint)
                msg.relative_frame_name = '/'+landmark_frame
                rospy.loginfo(add_waypoint_proxy(msg))
                self.update_waypoints()
                # Reset
                # self.relative_page_widget.name_field.setText('')
                # self.relative_page_widget.landmark_field.setText('NONE')
                # self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['gray'].normal+';color:#ffffff')
                # self.selected_landmark = None
            except rospy.ServiceException, e:
                rospy.logwarn(e)

    def remove_waypoint(self):
        if self.mode is 'FIXED':
            current_selected_name = self.waypoint_page_widget.waypoint_list.currentItem().text()
        else:
            current_selected_name = self.relative_page_widget.waypoint_list.currentItem().text()
        
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
                rospy.logwarn(e)

    def update_waypoints(self):
        # Update FIXED Waypoints
        rospy.wait_for_service('/instructor_core/GetWaypointList')
        self.found_waypoints = []
        self.waypoint_page_widget.waypoint_list.clear()
        try:
            get_waypoints_proxy = rospy.ServiceProxy('/instructor_core/GetWaypointList',GetWaypointList)
            self.found_waypoints = get_waypoints_proxy('').names
        except rospy.ServiceException, e:
            rospy.logwarn(e)
        for w in self.found_waypoints:
            self.waypoint_page_widget.waypoint_list.addItem(QListWidgetItem(w.strip('/')))
        self.waypoint_page_widget.waypoint_list.sortItems()
        self.waypoint_page_widget.waypoint_list.setCurrentRow(0)
        # Update RELATIVE Waypoints
        rospy.wait_for_service('/instructor_core/GetRelativeWaypointList')
        self.found_rel_waypoints = []
        self.relative_page_widget.waypoint_list.clear()
        try:
            get_relative_waypoints_proxy = rospy.ServiceProxy('/instructor_core/GetRelativeWaypointList',GetRelativeWaypointList)
            self.found_rel_waypoints = get_relative_waypoints_proxy('').names
        except rospy.ServiceException, e:
            rospy.logwarn(e)
        for w in self.found_rel_waypoints:
            self.relative_page_widget.waypoint_list.addItem(QListWidgetItem(w.strip('/')))
        self.relative_page_widget.waypoint_list.sortItems()
        self.relative_page_widget.waypoint_list.setCurrentRow(0)
        # Update LANDMARKS
        rospy.wait_for_service('/instructor_core/GetLandmarkList')
        landmark_names = []
        landmark_frames = []
        self.landmarks.clear()
        self.relative_page_widget.landmark_list.clear()
        try:
            get_landmarks_proxy = rospy.ServiceProxy('/instructor_core/GetLandmarkList',GetLandmarkList)
            found_landmarks = get_landmarks_proxy('')
            landmark_names = found_landmarks.names
            landmark_frames = found_landmarks.frame_names
        except rospy.ServiceException, e:
            rospy.logwarn(e)
        for name,frame in zip(landmark_names,landmark_frames):
            self.landmarks[name] = frame
        # rospy.logwarn(self.landmarks)
        # Add to UI
        for w in landmark_names:
            self.relative_page_widget.landmark_list.addItem(QListWidgetItem(w.strip('/')))
        self.relative_page_widget.landmark_list.sortItems()
        self.relative_page_widget.landmark_list.setCurrentRow(0)

    def servo_to_waypoint(self):
        if self.mode is 'FIXED':
            current_selected_waypoint = self.waypoint_page_widget.waypoint_list.currentItem().text()
            self.command_waypoint_name = str(current_selected_waypoint)
        else:
            current_selected_waypoint = self.relative_page_widget.waypoint_list.currentItem().text()
            self.command_waypoint_name = str(current_selected_waypoint)

        if self.command_waypoint_name != None:
            try:
                pose_servo_proxy = rospy.ServiceProxy('costar/ServoToPose',costar_robot_msgs.srv.ServoToPose)
                
                F_command_world = tf_c.fromTf(self.listener_.lookupTransform('/world', '/'+self.command_waypoint_name, rospy.Time(0)))
                F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
                F_command = F_base_world.Inverse()*F_command_world
                    
                msg = costar_robot_msgs.srv.ServoToPoseRequest()
                msg.target = tf_c.toMsg(F_command)
                msg.vel = .25
                msg.accel = .25
                # Send Servo Command
                rospy.logwarn('Single Servo Move Started')
                result = pose_servo_proxy(msg)
                if 'FAILURE' in str(result.ack):
                    rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                    return
                else:
                    rospy.logwarn('Single Servo Move Finished')
                    rospy.logwarn('Robot driver reported: '+str(result.ack))
                    return

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ServiceException), e:
                rospy.logwarn('There was a problem with the tf lookup or service:')
                rospy.logwarn(e)
                return


class RobotInterface():
    def __init__(self,status_label,teach_btn,servo_btn,sound_pub,toast):
        self.toast = toast
        self.driver_status_sub = rospy.Subscriber('/costar/DriverStatus',String,self.driver_status_cb)
        self.status_label = status_label
        self.teach_btn = teach_btn
        self.servo_btn = servo_btn
        self.sound_pub = sound_pub
        self.driver_status = 'NOT CONNECTED'
        self.status_label.setText('ROBOT MODE: [NOT CONNECTED]')

    def driver_status_cb(self,msg):
        mode = str(msg.data)
        self.driver_status = mode
        self.status_label.setText('ROBOT MODE: ['+mode.upper()+']')
        # self.update_status()

    def update_status(self):
        if self.driver_status == 'TEACH':
            self.status_label.setStyleSheet('background-color:'+colors['blue'].normal+'; color:#ffffff')
        elif self.driver_status == 'SERVO':
            self.status_label.setStyleSheet('background-color:'+colors['green'].normal+'; color:#ffffff')
        elif self.driver_status == 'IDLE':
            self.status_label.setStyleSheet('background-color:'+colors['gray_light'].normal+'; color:#ffffff')
        elif self.driver_status == 'IDLE - WARN':
            self.status_label.setStyleSheet('background-color:'+colors['orange'].normal+'; color:#ffffff')
        elif self.driver_status == 'NOT CONNECTED':
            self.status_label.setStyleSheet('background-color:'+colors['orange'].normal+'; color:#ffffff')
        elif self.driver_status == 'DISABLED':
            self.status_label.setStyleSheet('background-color:'+colors['red'].normal+'; color:#ffffff')

        # self.status_label.setText('ROBOT MODE: ['+self.driver_status.upper()+']')

    def teach(self):
        if self.driver_status == 'IDLE':
            try:
                rospy.wait_for_service('/costar/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find teach service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/costar/SetTeachMode',SetTeachMode)
                result = teach_mode_service(True)
                # self.sound_pub.publish(String("low_up"))
                rospy.logwarn(result.ack)
                self.teach_btn.set_color(colors['gray_light'])
            except rospy.ServiceException, e:
                print e
        elif self.driver_status == 'TEACH':
            try:
                rospy.wait_for_service('/costar/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find teach service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/costar/SetTeachMode',SetTeachMode)
                result = teach_mode_service(False)
                rospy.logwarn(result.ack)
                # self.sound_pub.publish(String("low_down"))
                self.teach_btn.set_color(colors['gray'])
            except rospy.ServiceException, e:
                print e
        else:
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')
            self.toast('Driver is in ['+self.driver_status+'] mode!')

    def servo(self):
        if self.driver_status == 'IDLE':
            try:
                rospy.wait_for_service('/costar/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
                result = servo_mode_service('SERVO')
                rospy.logwarn(result.ack)
                self.servo_btn.set_color(colors['gray_light'])
            except rospy.ServiceException, e:
                print e

        elif self.driver_status == 'SERVO':
            try:
                rospy.wait_for_service('/costar/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
                result = servo_mode_service('DISABLE')
                rospy.logwarn(result.ack)
                self.servo_btn.set_color(colors['gray'])
            except rospy.ServiceException, e:
                print e
        else:
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')
            self.toast('Driver is in ['+self.driver_status+'] mode!')

    def disable(self):
        try:
            rospy.wait_for_service('/costar/SetServoMode',2)
        except rospy.ROSException as e:
            print 'Could not find SetServoMode service'
            return
        try:
            servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
            result = servo_mode_service('DISABLE')
            rospy.logwarn(result.ack)
        except rospy.ServiceException, e:
            print e

    def teach_disable(self):
        if self.driver_status == 'TEACH':
            try:
                rospy.wait_for_service('/costar/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find teach service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/costar/SetTeachMode',SetTeachMode)
                result = teach_mode_service(False)
                rospy.logwarn(result.ack)
                # self.teach = False
                self._widget.teach_enable_label.setText('DISABLED')
                self._widget.teach_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
                self._widget.msg_label.setText("teach DISABLED")
            except rospy.ServiceException, e:
                print e
        else:
            self._widget.msg_label.setText("DRIVER MUST BE IN TEACH MODE TO DISABLE")
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')
            self.toast('Driver is in ['+self.driver_status+'] mode!')


#############################################################################

def clear_cmd():
    os.system(['clear','cls'][os.name == 'nt'])
    pass

def load_instructor_plugins():
    to_check = rospack.rospack_depends_on_1('beetree')
    rp = rospkg.RosPack()
    # to_check = rospack.get_depends_on('beetree', implicit=False)
    clear_cmd()
    print 'Found packages that have beetree dependency...'
    # print to_check
    plugins = []    
    descriptions = []
    names = []
    types = []
    groups = []
    for pkg in to_check:
        m = rp.get_manifest(pkg)
        p_modules = m.get_export('instructor', 'plugin')
        p_types = m.get_export('instructor', 'type')
        p_descriptions = m.get_export('instructor', 'description')
        p_names = m.get_export('instructor', 'name')
        p_groups = m.get_export('instructor', 'group')
        if not p_modules:
            continue
        if p_modules == []:
            pass

        for p_module, p_description, p_name, p_type, p_group in zip(p_modules,p_descriptions,p_names,p_types,p_groups):
            roslib.load_manifest(pkg)
            package = __import__(pkg)
            sub_mod = p_module.split('.')[1:][0]
            module = getattr(package, sub_mod)
            plugins.append(module)
            descriptions.append(p_description)
            names.append(p_name)               
            types.append(p_type)
            groups.append(p_group)                

    return plugins, descriptions, names, types, groups

class Instructor(QWidget):
    toast_signal = pyqtSignal()
    def __init__(self,app):
        super(Instructor,self).__init__()
        rospy.logwarn('INSTRUCTOR: STARTING UP...')
        self.app_ = app
        self.types__ = ['LOGIC', 'ACTION', 'CONDITION', 'QUERY', 'PROCESS', 'SERVICE','VARIABLE']
        self.colors__ = ['blue', 'green', 'purple', 'orange', 'pink', 'gray','gray']
        self.labels__ = ['BUILDING BLOCKS', 'ROBOT ACTIONS', 'SYSTEM KNOWLEDGE', 'QUERIES', 'PROCESSES','SERVICE','VARIABLES']
        # Load the ui attributes into the main widget
        self.rospack__ = rospkg.RosPack()
        ui_path = self.rospack__.get_path('instructor_core') + '/ui/view.ui'
        uic.loadUi(ui_path, self)
        # Create the Graph Visualization Pane
        self.drawer = Drawer(self)
        self.splitter.setStyleSheet('background-color:#444444')
        self.dot_widget = DotWidget()
        self.instructor_layout.addWidget(self.dot_widget)  
        self.splitter.splitterMoved.connect(self.splitter_moved)
        self.resizeEvent = self.window_resized
        self.setMouseTracking(True)  
        # Finish Up
        self.show()
        # Running the tree
        self.running__ = False
        self.run_timer_ = QTimer(self)
        self.connect(self.run_timer_, QtCore.SIGNAL("timeout()"), self.run)
        # Set up ros_ok watchdog timer to handle termination and ctrl-c
        self.ok_timer_ = QTimer(self)
        self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
        self.ok_timer_.start(100)

        self.toast_timer_ = QTimer(self)
        self.connect(self.toast_timer_, QtCore.SIGNAL("timeout()"), self.toast_reset)
        self.toast_timer_.setSingleShot(True)
        self.toast_signal.connect(self.toast_update)

        # self.refresh_ui_plugins_timer = QTimer(self)
        # self.connect(self.refresh_ui_plugins_timer, QtCore.SIGNAL("timeout()"), self.refresh_available_plugins)
        # self.refresh_ui_plugins_timer.start(1000)

        
        # Load Settings
        self.settings = QSettings('settings.ini', QSettings.IniFormat)
        self.settings.setFallbacksEnabled(False) 
        self.resize( self.settings.value('size', QSize(800, 800), type=QSize) )
        self.move(self.settings.value('pos', QPoint(50, 50), type=QPoint))
        # self.showMaximized()

        self.sound_pub = rospy.Publisher('/audri/sound/sound_player', String)

        ### ROBOT ###
        # Robot Buttons #
        self.teach_btn = InterfaceButton('TEACH',colors['gray'],txtsz=10)
        self.servo_btn = InterfaceButton('SERVO',colors['gray'],txtsz=10)
        self.status_layout.insertWidget(0,self.teach_btn)
        self.status_layout.addWidget(self.servo_btn)
        self.robot_ = RobotInterface(self.status_label,self.teach_btn,self.servo_btn,self.sound_pub,self.toast)
        self.toast_label.hide()
        self.teach_btn.clicked.connect(self.robot_.teach)
        self.servo_btn.clicked.connect(self.robot_.servo)
        self.servo_btn.clicked.connect(self.servo_button_cb)
        #############

        ### CORE STRUCTURES ###
        self.state__ = []
        self.current_tree = {}
        self.current_generators = {}
        self.all_generators = {}
        self.current_node_type = None
        self.selected_node_label = None
        self.active_node_type = None
        self.current_node_info ={}
        self.current_node_generator = None
        self.current_plugin_names = {}
        self.current_node_types = {}
        self.root_node = None
        self.selected_subtree = None
        self.left_selected_node = None
        self.available_plugins = []
        self.core_plugins = []
        self.current_plugins = []
        self.active_plugin_widgets = []
        self.is_set_up = False
        #######################

        # Get known Beetree Builder Node Plugins
        self.parse_plugin_info()
        # Set up the gui with plugin information
        # Set up communication between node view and the rest of the app
        self.connect(self.dot_widget,SIGNAL("clicked"), self.node_leftclick_cb)
        self.connect(self.dot_widget,SIGNAL("right_clicked"), self.node_rightclick_cb)
        self.connect(self.dot_widget,SIGNAL("clicked_location"),self.node_leftclick_location_cb)
        # Set up librarian
        self.lib_set_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.lib_save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.lib_load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.lib_list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.lib_delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)
        rospy.logwarn('INSTRUCTOR: CREATED LIBRARIAN SERVICES')
        self.lib_set_type_service('instructor_node')
        self.lib_set_type_service('instructor_subtree')
        self.set_up_gui()

    def toast(self,m):
        self.toast_message = m
        self.toast_signal.emit()

    def toast_update(self):
        self.toast_label.setStyleSheet('background-color:'+colors['red'].normal+'; color:#ffffff')
        self.toast_label.setText(str(self.toast_message))
        self.status_label.hide()
        self.toast_label.show()
        self.toast_timer_.start(3000)

    def toast_reset(self):
        self.status_label.show()
        self.toast_label.hide()

    def refresh_available_plugins(self):
        if self.is_set_up:
            changed = False
            # rospy.logwarn('INSTRUCTOR: REFRESHING COMPONENTS')
            costar_attachment_namespace = '/costar/component/ui/'
            required_plugins = []
            if rospy.has_param(costar_attachment_namespace):
                ui_list = rospy.get_param(costar_attachment_namespace)
                enable_plugins = disable_plugins = []
                # Check that a plugin is available and add it if not aleeady enabled
                required_plugins = ui_list.values()

                # rospy.logwarn('Available Plugins: ' + str(self.available_plugins))
                # rospy.logwarn('Core Plugins: ' + str(self.core_plugins))

                
                # Check which plugins to enable
                for i in required_plugins:
                    if i not in self.current_plugins:
                        if i in self.available_plugins:
                            rospy.logwarn('['+str(i)+'] is a valid plugin.')
                            self.current_plugins.append(i)
                            rospy.logwarn('Adding: ['+str(i)+']')
                        else:
                            rospy.logwarn('['+str(i)+'] is NOT a valid plugin.')
                        
                # # Check that current plugins are still valid
                # for i in self.current_plugins:
                #     if i not in required_plugins:
                #         if i not in self.core_plugins:
                #             changed = True
                #             rospy.logwarn('Removing: ['+str(i)+']')
                #             self.current_plugins.remove(i)


            ### Update Plugin List 
            # Add new plugins
            for p in self.plugins.itervalues():
                description = p['description']
                if description in self.current_plugins:
                    if description not in self.active_plugin_widgets:
                        rospy.logwarn('Adding widget for plugin: ['+description+']')
                        changed = True
                        self.component_widgets[p['type']].add_item_to_group(p['name'],p['group'])
                        self.active_plugin_widgets.append(description)

            # Remove plugins that are no longer required
            for p in self.plugins.itervalues():
                description = p['description']
                if description in self.current_plugins:
                    if description not in required_plugins:
                        if description not in self.core_plugins:
                            rospy.logwarn('Removing widget for plugin: ['+description+']')
                            changed = True
                            self.current_plugins.remove(description)
                            self.component_widgets[p['type']].remove_item_from_group(p['name'],p['group'])
                            self.active_plugin_widgets.remove(description)

            # for n in self.plugins.itervalues():
            #     item = n['name']
            #     group = n['group']
            #     description = n['description']

            #     if description in self.current_plugins:
            #     self.component_widgets[n['type']].add_item_to_group(item,group)

            for c in self.component_widgets.values():
                if len(c.items) == 0:
                    c.hide()

            if changed is True:
                rospy.logwarn('Current Plugins: ' + str(self.current_plugins))
                rospy.logwarn('Required Plugins: ' + str(required_plugins))
                rospy.logerr('Warning - system changed, you probably need to create a new tree.')

    def parse_plugin_info(self):
        rospy.logwarn('INSTRUCTOR: LOADING PLUGINS')
        self.plugins = {}
        self.node_counter = {}
        plugins, plugin_descriptions, plugin_names, plugin_types, plugin_groups = load_instructor_plugins()
        for plug,desc,name,typ,grp in zip(plugins,plugin_descriptions,plugin_names,plugin_types, plugin_groups):
            self.plugins[name] = {'module':plug, 'type':typ, 'name':name, 'group':grp, 'description':desc, 'generator_type':str(type(plug()))}
        
        self.available_plugins = plugin_descriptions
        # rospy.logwarn(self.available_plugins)
        for p in self.plugins.itervalues():
            # rospy.logwarn(str(p['group']))
            if any(['SYSTEM' in p['group'], 'ROBOT' in p['group']]):
                self.core_plugins.append(p['description'])
                self.current_plugins.append(p['description'])

        rospy.logwarn('INSTRUCTOR: LOADING GENERATORS')
        for name,plugin in self.plugins.items():
            self.all_generators[name] = plugin['module']()

    def splitter_moved(self,pos,index):
        # rospy.logwarn(pos)
        # rospy.logwarn(self.container_widget.geometry())
        self.drawer.resize(360,self.container_widget.geometry().height()-100)
        self.drawer.move(self.container_widget.geometry().left()-360,50)
        self.drawer.hide()

    def window_resized(self,event):
        rospy.logwarn(event.size())
        visible = self.drawer.isVisible()
        self.drawer.resize(360,self.container_widget.geometry().height()-100)
        self.drawer.move(self.container_widget.geometry().left()-360,50)
        if visible:
            self.drawer.show()
        else:
            self.drawer.hide()
    
    def collapse_unused(self,id):
        self.hide_menu()
        self.load_subtree_list()
        # self.load_node_list()
        self.clear_node_info()
        self.load_sub_btn.hide()
        self.remove_sub_btn.hide()
        self.remove_saved_node_btn.hide()
        self.close_drawer()
        for k,v in self.component_widgets.items():
            if k is not id:
                v.contract()
        for k,v in self.load_widgets.items():
            if k is not id:
                v.contract()

    def open_drawer(self):
        self.regenerate_btn.hide()
        if self.drawer_state == 'CLOSED':
            # x,y = self.dot_widget.get_current_pos()
            # self.dot_widget.set_current_pos(x+400,y)
            self.drawer.show()
            self.open_animation = QPropertyAnimation(self.drawer, "geometry")
            self.open_animation.setDuration(200)
            self.drawer.startGeometry = self.drawer_close_geo
            self.drawer.endGeometry = self.drawer_open_geo
            self.open_animation.setStartValue(self.drawer.startGeometry)
            self.open_animation.setEndValue(self.drawer.endGeometry)
            self.open_animation.start()
            self.drawer_state = 'OPEN'

    def close_drawer(self):
        self.selected_node_field.set_color(colors['gray_light'])
        if self.drawer_state == 'OPEN':
            # x,y = self.dot_widget.get_current_pos()
            # self.dot_widget.set_current_pos(x-400,y)
            # self.clear_node_info()
            self.clear_node_info()
            if self.current_node_plugin_name is not None:
                self.all_generators[self.current_node_plugin_name] = self.plugins[self.current_node_plugin_name]['module']()
                self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                self.current_node_type = self.plugins[self.current_node_plugin_name]['type']
                # self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                self.drawer.node_info_layout.addWidget(self.current_node_generator)
                self.right_selected_node = None

            # self.drawer.
            # if self.current_node_generator is not None:
            #     self.drawer.node_info_layout.addWidget(self.current_node_generator)
            # self.clear_node_info()
            # self.drawer.show()
            self.open_animation = QPropertyAnimation(self.drawer, "geometry")
            self.open_animation.setDuration(200)
            self.drawer.startGeometry = self.drawer_open_geo
            self.drawer.endGeometry = self.drawer_close_geo
            self.open_animation.setStartValue(self.drawer.startGeometry)
            self.open_animation.setEndValue(self.drawer.endGeometry)
            self.open_animation.start()
            self.drawer_state = 'CLOSED'
    
    def set_up_gui(self):
        ### DRAWER ###
        self.drawer.resize(360,self.container_widget.geometry().height()-100)
        self.drawer.move(self.container_widget.geometry().left()-360,50)
        self.drawer_open_geo = QRect(self.drawer.geometry())
        self.drawer_close_geo = QRect(self.drawer.geometry().x()+360,self.drawer.geometry().y(),0,self.drawer.geometry().height())
        self.drawer.hide()
        self.drawer_state = 'CLOSED'
        self.add_node_above_btn = InterfaceButton('ADD ABOVE',colors['gray_light'])
        self.add_node_above_btn.clicked.connect(self.add_sibling_before_cb)
        self.add_node_below_btn = InterfaceButton('ADD BELOW',colors['gray_light'])
        self.add_node_below_btn.clicked.connect(self.add_sibling_after_cb)
        self.add_node_child_btn = InterfaceButton('ADD',colors['green_light'])
        self.add_node_child_btn.clicked.connect(self.add_child_cb)
        self.replace_node_btn = InterfaceButton('REPLACE',colors['gray_light'])
        self.replace_node_btn.clicked.connect(self.replace_child_cb)
        self.add_node_root_btn = InterfaceButton('ADD AS ROOT',colors['green_light'])
        self.add_node_root_btn.clicked.connect(self.add_root_cb)
        self.drawer.button_layout.addWidget(self.add_node_root_btn,0,0,1,2)
        self.drawer.button_layout.addWidget(self.add_node_child_btn,1,0)
        self.drawer.button_layout.addWidget(self.replace_node_btn,1,1)
        self.drawer.button_layout.addWidget(self.add_node_above_btn,2,0)
        self.drawer.button_layout.addWidget(self.add_node_below_btn,2,1)

        self.regenerate_btn = InterfaceButton('REGENERATE',colors['green_light'])
        self.regenerate_btn.clicked.connect(self.regenerate_node)
        self.drawer.button_layout.addWidget(self.regenerate_btn,3,0,1,2)
        self.regenerate_btn.hide()
        self.add_node_cancel_btn = InterfaceButton('CANCEL',colors['red'])
        self.add_node_cancel_btn.clicked.connect(self.close_drawer)
        self.drawer.button_layout.addWidget(self.add_node_cancel_btn,4,0,1,2)

        # WAYPOINT DIALOG #
        self.waypoint_dialog = WaypointManagerDialog()
        self.waypoint_dialog.hide()
        self.waypoint_btn.clicked.connect(self.show_waypoint_manager)
        self.waypoint_dialog_saved_geom = None

        # JOG Dialog
        self.jog_dialog = JogDialog(self.show_jog)
        self.jog_dialog.hide()
        self.smartmove_dialog = SmartMoveDialog(self.show_smartmove)
        self.smartmove_button.clicked.connect(self.show_smartmove)

        # CLEAR #
        self.clear_btn = InterfaceButton('CLEAR',colors['dark_red'])
        self.clear_btn.hide()
        self.clear_cancel_btn = InterfaceButton('Cancel',colors['dark_red'])
        self.clear_btn.clicked.connect(self.clear_all)
        self.clear_cancel_btn.clicked.connect(self.clear_all_cancel)
        self.clear_confirm = False

        # SAVE BUTTONS #
        self.load_sub_btn = InterfaceButton('LOAD',colors['green'])
        self.load_sub_btn.hide()
        self.load_sub_btn.clicked.connect(self.load_selected_subtree)
        self.remove_sub_btn = InterfaceButton('REMOVE',colors['red'])
        self.remove_sub_btn.hide()
        self.remove_sub_btn.clicked.connect(self.delete_selected_subtree)
        self.remove_saved_node_btn = InterfaceButton('REMOVE',colors['red'])
        self.remove_saved_node_btn.hide()
        self.remove_saved_node_btn.clicked.connect(self.delete_selected_saved_node)
        self.save_btn = InterfaceButton('SAVE',colors['gray'])
        self.delete_btn = InterfaceButton('DELETE',colors['red'])
        self.save_btn.hide()
        self.delete_btn.hide()
        self.delete_btn.clicked.connect(self.delete_cb)
        self.button_layout.addWidget(self.save_btn)
        self.button_layout.addWidget(self.delete_btn)
        self.button_layout.addWidget(self.clear_btn)
        self.button_layout.addWidget(self.clear_cancel_btn)
        self.clear_cancel_btn.hide()

        # RUN STOP BUTTONS #
        self.run_button.setMinimumHeight(48)
        self.run_button.clicked.connect(self.run_tree)
        self.run_button.show()
        self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}''')

        # Menu
        self.menu_widget.hide()
        self.menu_button.clicked.connect(self.show_menu)
        self.menu_button.setMinimumHeight(60)
        self.menu_visible = False
        self.open_gripper_button.clicked.connect(self.open_gripper_cb)
        self.close_gripper_button.clicked.connect(self.close_gripper_cb)
        self.gripper_combo.activated[QString].connect(self.gripper_mode_cb)
        self.detect_objects_button.clicked.connect(self.detect_objects_cb)
        self.calibrate_button.clicked.connect(self.calibrate_cb)
        self.update_scene_button.clicked.connect(self.update_scene_cb)

        # CONTEXT MENU #
        self.context_popup = Popup(self)
        self.context_inspect_btn = InterfaceButton('INSPECT',colors['pink'],parent=self)
        self.context_inspect_btn.setMinimumWidth(89)
        self.context_inspect_btn.clicked.connect(self.inspect_node)
        self.context_popup.layout.addWidget(self.context_inspect_btn)
        self.context_fold_btn = InterfaceButton('FOLD',colors['sea_green'],parent=self)
        self.context_fold_btn.setMinimumWidth(89)
        self.context_fold_btn.clicked.connect(self.collapse_node)
        self.context_popup.layout.addWidget(self.context_fold_btn)
        self.context_popup.hide()

        # VIEW BUTTONS #
        self.view_fit_btn = InterfaceButton('FIT',colors['gray'],txtsz=10)
        self.view_fit_btn.clicked.connect(self.view_fit)
        self.view_layout.addWidget(self.view_fit_btn)
        self.selected_node_field = InterfaceButton('SELECTED NODE: NONE',colors['gray'],txtsz=10)
        self.view_layout.addWidget(self.selected_node_field)
        self.inspect_btn = InterfaceButton('INSPECT NODE',colors['pink'],txtsz=10)
        self.inspect_btn.clicked.connect(self.inspect_node)
        self.view_layout.addWidget(self.inspect_btn)
        self.inspect_btn.hide()
        self.view_center_btn = InterfaceButton('CENTER ON NODE',colors['gray'],txtsz=10)
        self.view_center_btn.clicked.connect(self.view_center)
        self.view_layout.addWidget(self.view_center_btn)
        self.view_center_btn.hide()

        # COMPONENT WIDGETS
        self.component_widgets = {}
        self.load_widgets = {}
        # Create Expanding Columns for Components
        for t,label,c in zip(self.types__,self.labels__,self.colors__):
            w = ListContainer(t,label,colors[c])
            w.register_callbacks(self.collapse_unused,self.component_selected_callback)
            w.show()
            self.component_widgets[t] = w
            self.component_layout.addWidget(w)

        self.subtree_container = ListContainer('SUBTREES','SUBTREES',colors['green_light'],'large')
        # self.node_container = ListContainer('NODES','NODES',colors['pink'],'large')
        self.subtree_container.show()
        self.subtree_container.register_callbacks(self.collapse_unused,self.subtree_selected_callback)
        self.subtree_container.layout.addWidget(self.load_sub_btn)
        self.subtree_container.layout.addWidget(self.remove_sub_btn)
        # self.node_container.show()
        # self.node_container.register_callbacks(self.collapse_unused,self.node_selected_callback)
        # self.node_container.layout.addWidget(self.remove_saved_node_btn)
        self.load_widgets['SUBTREES'] = self.subtree_container
        # self.load_widgets['NODES'] = self.node_container
        self.component_layout.addWidget(self.subtree_container)
        # self.component_layout.addWidget(self.node_container)

        ### DIALOG ###
        self.save_dialog = Dialog()
        self.save_dialog.move(100,100)
        self.save_dialog.hide()
        self.save_dialog.save_cancel_btn.clicked.connect(self.hide_save_dialog)
        self.save_dialog.save_subtree_btn.clicked.connect(self.save_subtree)
        self.save_dialog.save_node_btn.clicked.connect(self.save_node)
        self.save_dialog.save_name_field.textChanged.connect(self.saved_name_updated_cb)
        self.save_btn.clicked.connect(self.show_save_dialog)

        # ### Update Plugin List 
        for n in self.plugins.itervalues():
            item = n['name']
            group = n['group']
            self.component_widgets[n['type']].add_item_to_group(item,group)

        for c in self.component_widgets.values():
            if len(c.items) == 0:
                c.hide()

        ### LOAD SUBTREE LIST ###
        self.load_subtree_list()
        # self.load_node_list()

        ### FINALIZE ###
        self.is_set_up = True

    def contract_all(self):
        for w in self.component_widgets.values():
            w.contract()
        self.subtree_container.contract()

    def calibrate_cb(self):
        rospy.logwarn('Calibrating...')
        self.toast('Please wait... calibrating...')
        self.calibrate_button.setEnabled(False)
        service_name = '/calibrate'
        self.send_service_command(service_name)
        rospy.logwarn('Calibrating...DONE')
        self.calibrate_button.setEnabled(True)

    def detect_objects_cb(self):
        rospy.logwarn('Detecting objects...')
        self.toast('Please wait... Detecting Objects')
        self.detect_objects_button.setEnabled(False)
        service_name = '/SPServer/SPSegmenter'
        self.send_service_command(service_name)
        rospy.logwarn('Detecting objects...DONE')
        self.detect_objects_button.setEnabled(True)

    def update_scene_cb(self):
        rospy.logwarn('Updating Scene objects...')
        self.toast('Please wait... Updating Scene')
        self.update_scene_button.setEnabled(False)
        service_name = '/planningSceneGenerator/planningSceneGenerator'
        self.send_service_command(service_name)
        rospy.logwarn('Updating Scene...DONE')
        self.update_scene_button.setEnabled(True)

    def gripper_mode_cb(self,text):
        mode = str(text).lower()
        if mode == 'scissor':
            self.send_gripper_command('open')
            self.send_gripper_command('scissor_mode')
        elif mode == 'basic':
            self.send_gripper_command('open')
            self.send_gripper_command('basic_mode')
        elif mode == 'pinch':
            self.send_gripper_command('open')
            self.send_gripper_command('pinch_mode')
        elif mode == 'wide':
            self.send_gripper_command('open')
            self.send_gripper_command('wide_mode')

    def open_gripper_cb(self):
        self.send_gripper_command('open')
        pass

    def close_gripper_cb(self):
        self.send_gripper_command('close')
        pass

    def send_gripper_command(self,option):
        service_name = '/costar/gripper/%s'%(option)
        self.send_service_command(service_name)

    def send_service_command(self,service_name):
        # Check to see if service exists
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find service ['+str(e)+']')
            return
        # Make servo call to robot
        try:
            service_cmd_proxy = rospy.ServiceProxy(service_name,Empty)
            result = service_cmd_proxy()
            return
        except (rospy.ServiceException), e:
            rospy.logwarn('There was a problem with the service ['+str(e)+']')
            return

    def show_menu(self):
        if self.menu_visible == False:
            self.contract_all()
            self.menu_widget.show()
            self.menu_visible = True
        else:
            self.menu_visible = False
            self.menu_widget.hide()

    def hide_menu(self):
        self.menu_visible = False
        self.menu_widget.hide()

    def show_jog(self):
        if self.jog_dialog.isVisible():
            self.jog_dialog.saved_geom = self.jog_dialog.geometry()
            self.jog_dialog.hide()
        else:
            if self.jog_dialog.saved_geom is not None:
                self.jog_dialog.move(self.jog_dialog.saved_geom.x(),self.jog_dialog.saved_geom.y())
            else:
                self.jog_dialog.move(self.geometry().x()+self.geometry().width()/2-self.jog_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.jog_dialog.geometry().height()/2)
            self.jog_dialog.show()

    def show_smartmove(self):
        if self.smartmove_dialog.isVisible():
            self.smartmove_dialog.saved_geom = self.smartmove_dialog.geometry()
            self.smartmove_dialog.hide()
        else:
            if self.smartmove_dialog.saved_geom is not None:
                self.smartmove_dialog.move(self.smartmove_dialog.saved_geom.x(),self.smartmove_dialog.saved_geom.y())
            else:
                self.smartmove_dialog.move(self.geometry().x()+self.geometry().width()/2-self.smartmove_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.smartmove_dialog.geometry().height()/2)
            self.smartmove_dialog.show()
            self.smartmove_dialog.update_all()

    def show_waypoint_manager(self):
        if self.waypoint_dialog.isVisible():
            self.waypoint_dialog_saved_geom = self.waypoint_dialog.geometry()
            self.waypoint_dialog.hide()
        else:
            if self.waypoint_dialog_saved_geom is not None:
                self.waypoint_dialog.move(self.waypoint_dialog_saved_geom.x(),self.waypoint_dialog_saved_geom.y())
            else:
                self.waypoint_dialog.move(self.geometry().x()+self.geometry().width()/2-self.waypoint_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.waypoint_dialog.geometry().height()/2)
            self.waypoint_dialog.show()

    def load_subtree_list(self):
        self.subtree_list = self.lib_list_service('instructor_subtree').entries
        self.subtree_container.remove_all()
        if len(self.subtree_list) > 0: # There are actually nodes to load
            for st in self.subtree_list:
                self.subtree_container.add_item(str(st))

        self.subtree_container.sort()

    # def load_node_list(self):
    #     generator_list = self.lib_list_service('instructor_node').entries
    #     self.node_container.remove_all()
    #     if len(generator_list) > 0: # There are actually nodes to load
    #         self.loadable_nodes = {}
    #         for g in generator_list:
    #             data = yaml.load(self.lib_load_service(id=g,type='instructor_node').text)
    #             self.loadable_nodes[data['name']] = data
    #         for n in self.loadable_nodes.itervalues():
    #             self.node_container.add_item(n['name'])

    def view_fit(self):
        if self.root_node is not None:
            self.dot_widget.zoom_to_fit()

    def view_center_default(self):
        x = 0.0
        y = 0.0
        jump = self.dot_widget.get_jump(x,y)
        if jump is not None:
            self.dot_widget.animate_to(jump.x,jump.y)

    def view_center(self):
        x = self.selected_location[0]
        y = self.selected_location[1]
        jump = self.dot_widget.get_jump(x,y)
        if jump is not None:
            self.dot_widget.animate_to(jump.x,jump.y)

    def show_save_dialog(self):
        self.save_dialog.move(self.geometry().x()+self.geometry().width()/2-self.save_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.save_dialog.geometry().height()/2)
        self.save_dialog.show()
        self.save_dialog.label.setText('To save the selected node, click SAVE NODE.\nTo save the subtree starting with the selected node, click SAVE SUBTREE.')

    def hide_save_dialog(self):
        self.save_dialog.hide()
        pass

    def collapse_node(self):
        if self.right_selected_node != None:
            val = self.right_selected_node
        else:
            val = self.left_selected_node

        node = self.current_tree[val]
        if self.current_node_types[val] == 'LOGIC': 
            if node.collapsed == True:
                node.set_collapsed(False)
                self.regenerate_tree()
                self.context_fold_btn.setText('FOLD')
            elif node.collapsed == False:
                node.set_collapsed(True)
                self.regenerate_tree()
                self.context_fold_btn.setText('UNFOLD')
            self.context_popup.hide()
        
# Show and Hide Info
    def show_info_cb(self,check):
        if check == Qt.Checked:
            for node in self.current_tree.itervalues():
                node.set_alt_view(True)
            self.regenerate_tree()

        if check == Qt.Unchecked:
            for node in self.current_tree.itervalues():
                node.set_alt_view(False)
            self.regenerate_tree()

# Run and Stop -----------------------------------------------------------------
    def servo_button_cb(self):
        if self.running__ == True:
            self.stop_tree()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.toast('ABORTED: Robot is no longer in servo')
            
    def run_tree(self):
         rospy.logwarn(self.robot_.driver_status)
         if 'servo' not in str(self.robot_.driver_status).lower():
             self.toast('Robot is NOT in SERVO MODE')
             return 
         elif self.root_node == None:
             self.toast('There is no root node') 
             return
         else:
            if self.running__ == True:
                self.stop_tree()
                self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
                self.run_button.setText('EXECUTE PLAN')
            else:
                self.root_node.reset()
                self.sound_pub.publish(String("notify_4"))
                self.running__ = True
                self.run_timer_.start(5)
                rospy.logwarn('INSTRUCTOR: Task Tree STARTING')
                self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #F62459;border-radius: 0px;background-color: #F62459;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #F62459;border-radius: 0px;background-color: #F62459;color:#ffffff}''')
                self.run_button.setText('STOP EXECUTION')


    def run(self):
        result = self.root_node.execute()
        rospy.logwarn(result)
        # self.regenerate_tree()
        if result == 'SUCCESS':
            rospy.logwarn('INSTRUCTOR: Task Tree FINISHED WITH SUCCESS')
            self.sound_pub.publish(String("notify_4_succeed"))
            self.run_timer_.stop()
            self.running__ = False
            self.root_node.reset()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.regenerate_tree()
        elif result == 'FAILURE':
            rospy.logerr('INSTRUCTOR: Task Tree FINISHED WITH FAILURE')
            self.run_timer_.stop()
            self.running__ = False
            # self.root_node.reset()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.regenerate_tree()
            rospy.sleep(.5)
            self.sound_pub.publish(String("notify_4_fail"))
        elif result == 'NODE_ERROR':
            rospy.logwarn('INSTRUCTOR: Task Tree ERROR')
            self.run_timer_.stop()
            self.running__ = False
            # self.root_node.reset()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.regenerate_tree()
        elif result == 'RUNNING':
            self.regenerate_tree(True)
            pass

    def stop_tree(self):
        rospy.logwarn('INSTRUCTOR: Task Tree STOPPED')
        self.run_timer_.stop()
        self.running__ = False
        # self.root_node.reset()
        self.regenerate_tree()

# Save and Load Nodes ----------------------------------------------------------
    def save_node(self):
        if self.right_selected_node != None:
            if type(self.current_node_generator.generate()) == str:
                rospy.logerr('The node must be fully defined to save it.')
            else:
                print ''
                generator_to_save = {'node_type':self.current_node_type, 'name':self.save_name, 'plugin_name':self.current_node_plugin_name, 'generator_info':self.current_node_generator.save()}
                D = yaml.dump(generator_to_save)
                print D
                print self.lib_save_service(id=self.save_name,type='instructor_node',text=D)
                self.hide_save_dialog()

        elif self.current_node_type != None:
            if type(self.current_node_generator.generate()) == str:
                rospy.logerr('The node must be fully defined to save it.')
            else:
                print ''
                generator_to_save = {'node_type':self.current_node_type, 'name':self.save_name, 'plugin_name':self.current_node_plugin_name, 'generator_info':self.current_node_generator.save()}
                D = yaml.dump(generator_to_save)
                print D
                print self.lib_save_service(id=self.save_name,type='instructor_node',text=D)
                self.hide_save_dialog()

    def load_node_info(self):
        print 'Attempting to load node'
        try:
            self.open_drawer()
            node_data = self.loadable_nodes[self.selected_load_node]
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                print 'Node to load matches known nodes ['+ node_plugin_name +']'
                self.clear_node_info()
                self.current_node_generator = self.plugins[node_plugin_name]['module']()
                self.current_node_type = self.plugins[node_plugin_name]['type']
                self.current_node_plugin_name = node_plugin_name
                rospy.logwarn(self.current_node_generator.get_name())
                self.drawer.node_info_layout.addWidget(self.current_node_generator)
                # Add in parameters from saved file
                self.current_node_generator.load(node_data['generator_info'])
                self.current_node_generator.name.set_field(self.increment_node_name(node_plugin_name))
                # Close gui

        except KeyError as e:
            print "Not a valid node"

    def node_selected_callback(self,val):
        self.remove_saved_node_btn.show()
        self.selected_load_node = val
        self.load_node_info()

    def delete_selected_saved_node(self):
        if self.selected_load_node != None:
            self.lib_delete_service(id=self.selected_load_node,type='instructor_node')
            # self.load_node_list()

# Save and Load Subtrees -------------------------------------------------------

    def saved_name_updated_cb(self,t):
        self.save_name = str(t)
        pass

    def save_subtree(self):
        if self.left_selected_node == None:
            rospy.logerr('There is no head node selected to save')
        else:
            if self.save_name != None:
                tree = self.walk_tree(self.current_tree[self.left_selected_node]) # this should start from the selected node
                # tree = self.walk_tree(self.root_node) #this will always start from root
                D = yaml.dump({'name':self.save_name,'tree':tree})
                rospy.logwarn('SAVING SUBTREE')
                rospy.logwarn(D)
                print self.lib_save_service(id=self.save_name,type='instructor_subtree',text=D)
                # Hide on successful save    
                #self.subtree_save_widget.hide()
                self.hide_save_dialog()
                self.load_subtree_list()
            else:
                rospy.logerr('You must enter a name to save the subtree')

    def walk_tree(self,node):
        t = [self.walk_tree(C) for C in node.children_]
        # Generate Info
        generator = self.all_generators[self.current_plugin_names[node.name_]]
        generator.load(self.current_node_info[node.name_])
        # generator = self.current_generators[node.name_]
        node_type = self.current_node_types[node.name_]
        plugin_name = self.current_plugin_names[node.name_]
        generator_to_save = {'node_type':node_type, 'name':node.name_, 'plugin_name':plugin_name, 'generator_info':generator.save()}

        return {'name':node.name_, 'save_info':generator_to_save, 'children':t}

    def subtree_selected_callback(self,val):
        # self.selected_subtree = str(self.subtree_load_model.itemFromIndex(val).text())
        self.selected_subtree = val
        self.load_sub_btn.show()
        self.remove_sub_btn.show()
        self.load_sub_btn.setText('LOAD [ '+val.upper()+' ]')
        self.remove_sub_btn.setText('REMOVE [ '+val.upper()+' ]')

        self.selected_subtree_data = yaml.load(self.lib_load_service(id=self.selected_subtree,type='instructor_subtree').text)
        selected_subtree_root_name = self.selected_subtree_data['tree']['save_info']['plugin_name']
        print selected_subtree_root_name
        if 'root' in selected_subtree_root_name.lower():
            print 'You have selected a tree with a root node.  If a node is currently selected, this subtree will be added as a child tree of the selected node.  If the graph is empty, this subtree will be added as the entire tree.'
        else:
            print 'The selected subtree has no root node.  If a node is currently selected, this subtree will be added as a child tree.  If no node is selected, the subtree will be added along with a root node.'

    def delete_selected_subtree(self):
        if self.selected_subtree != None:
            self.lib_delete_service(id=self.selected_subtree,type='instructor_subtree')
            self.load_subtree_list()

    def load_selected_subtree(self):
        if self.selected_subtree != None:
            clear_cmd()
            print 'loading subtree'
            if self.root_node != None: # There is an existing tree, with a root node
                rospy.logwarn('Loading as subtree...')
                if self.left_selected_node == None:
                    rospy.logwarn('You must select a node as a parent for the subtree')
                else:
                    # Test to see if the subtree has a root...
                    node_plugin_name = self.selected_subtree_data['tree']['save_info']['plugin_name']
                    if node_plugin_name == 'Root':
                        rospy.logwarn('Found root in subtree... removing')
                        start_point = self.selected_subtree_data['tree']['children'][0]
                    else:
                        rospy.logwarn('No root, starting with '+node_plugin_name)
                        start_point = self.selected_subtree_data['tree']
                    try:
                        self.recursive_add_nodes(start_point,self.current_tree[self.left_selected_node])
                        self.regenerate_tree(center=True)
                    except Exception as e:
                        rospy.logerr('There was a problem loading the tree, most likely a node that didnt generate properly')
                        rospy.logerr(e)

            else: # The tree is empty
                rospy.logwarn('Loading as full tree...')
                try:
                    # Test to see if the subtree has a root...
                    node_plugin_name = self.selected_subtree_data['tree']['save_info']['plugin_name']
                    if node_plugin_name != 'Root':
                        # No root, generate one...
                        self.generate_root()
                        self.recursive_add_nodes(self.selected_subtree_data['tree'], self.root_node)
                        self.regenerate_tree(center=True)
                    else:
                        self.recursive_add_nodes(self.selected_subtree_data['tree'],None)
                        self.regenerate_tree(center=True)
                except Exception as e:
                    rospy.logerr('There was a problem loading the tree, most likely a node that didnt generate properly')
                    rospy.logerr(e)

            # Finish
            self.load_sub_btn.hide()
            self.remove_sub_btn.hide()
            self.selected_subtree = None
            self.view_fit()
        else:
            rospy.logwarn('You must select a subtree to load')

    def recursive_add_nodes(self,info,parent):
        if parent == None:
            node_data = info['save_info']
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                node_type = self.plugins[node_plugin_name]['type']
                plugin_name = node_plugin_name
                # Add in parameters from saved file
                self.all_generators[node_plugin_name].load(node_data['generator_info'])
                # Generate Node
                node_to_add = self.all_generators[node_plugin_name].generate()
                current_name = self.all_generators[node_plugin_name].get_name()
                # Add node
                self.current_tree[current_name] = node_to_add
                self.current_node_info[current_name] = self.all_generators[node_plugin_name].save()
                self.current_plugin_names[current_name] = plugin_name
                self.current_node_types[current_name] = node_type
                self.root_node = node_to_add
            else:
                rospy.logwarn('node plugin undefined')
            # recusively call child nodes
            for C in info['children']:
                self.recursive_add_nodes(C,node_to_add)
        else: 
            node_data = info['save_info']
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                node_type = self.plugins[node_plugin_name]['type']
                plugin_name = node_plugin_name
                self.all_generators[node_plugin_name].load(node_data['generator_info'])
                # Generate Child
                self.all_generators[node_plugin_name].name.set_field(self.increment_node_name(node_plugin_name))
                node_to_add = self.all_generators[node_plugin_name].generate()
                current_name = self.all_generators[node_plugin_name].get_name()
                self.current_tree[current_name] = node_to_add
                self.current_tree[parent.name_].add_child(node_to_add)
                self.current_node_info[current_name] = self.all_generators[node_plugin_name].save()
                self.current_plugin_names[current_name] = plugin_name
                self.current_node_types[current_name] = node_type
            else:
                rospy.logwarn(node_plugin_name)
                rospy.logwarn('node plugin undefined')
            # recusively call child nodes
            for C in info['children']:
                self.recursive_add_nodes(C,node_to_add)

# Callbacks --------------------------------------------------------------------
    def increment_node_name(self,name):
        # rospy.logwarn('incrementing ['+name+']')
        if 'root' not in name.lower():
            if self.node_counter.has_key(name):
                self.node_counter[name]+=1
                num = self.node_counter[name]
            else:
                num = 0
                self.node_counter[name] = num
            auto_name = str(str(name)+'_'+str(num)).replace(' ','_').lower()
            return auto_name
        else:
            return 'root'
    
    def component_selected_callback(self,name):
        rospy.logwarn('Selected node with type [' + str(self.plugins[name]['type']) + '] from list.')
        self.open_drawer()
        # if self.plugins[name]['type'] == 'LOGIC':
        #     self.add_node_root_btn.show()
        # else:
        #     self.add_node_root_btn.hide()

        if self.plugins[name]['name'] == 'Root':
            self.add_node_root_btn.show()
            self.add_node_child_btn.hide()
            self.replace_node_btn.hide()
            self.add_node_above_btn.hide()
            self.add_node_below_btn.hide()
        else:
            self.add_node_root_btn.hide()
            self.add_node_child_btn.show()
            self.replace_node_btn.show()
            self.add_node_above_btn.show()
            self.add_node_below_btn.show()

        if self.plugins.has_key(name):
            self.clear_node_info()
            self.all_generators[name] = self.plugins[name]['module']()
            self.current_node_generator = self.all_generators[name]
            self.current_node_type = self.plugins[name]['type']
            self.current_node_plugin_name = name
            self.current_node_generator.name.set_field(self.increment_node_name(name))
            self.drawer.node_info_layout.addWidget(self.current_node_generator)
            self.right_selected_node = None
        else:
            self.clear_node_info()

    def node_leftclick_location_cb(self,event):
        self.selected_location = event
        pass

    def node_leftclick_cb(self,event):
        # rospy.logwarn(event)
        if event == 'none':
            # Update Context Menu
            self.context_popup.hide()
            self.root_node.reset()
            # Update Selected Node
            self.left_selected_node = None
            self.selected_node_field.setText('SELECTED NODE: NONE')
            self.selected_node_field.set_color(colors['gray_light'])
            self.inspect_btn.hide()
            self.view_center_btn.hide()
            # self.run_button.hide()
            self.load_sub_btn.hide()
            self.remove_sub_btn.hide()
            self.remove_saved_node_btn.hide()
            self.selected_subtree = None
            self.save_btn.hide()
            self.delete_btn.hide()
            self.clear_btn.hide()
            # Remove all highlights
            if len(self.current_tree.keys()) != 0:
                self.remove_node_highlights()
                self.regenerate_tree()
            self.clear_all_cancel()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}''')
        else:
            if event in self.current_tree:
                self.left_selected_node = event
                # Update Context Menu
                self.context_popup.hide()
                # Update Selected Node Field
                self.inspect_btn.show()
                self.view_center_btn.show()
                self.save_btn.show()
                self.delete_btn.show()
                self.selected_node_field.setText(str(event).upper())
                self.selected_node_field.set_color(colors['green'])
                if self.current_node_types[event] == 'KNOLWEDGE':
                    self.selected_node_field.set_color(colors['purple'])
                if self.current_node_types[event] == 'LOGIC':
                    self.selected_node_field.set_color(colors['blue'])
                    if 'root' in event.lower():
                        self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;
border-radius: 0px;
background-color: #3FC380;
color:#ffffff}
QPushButton#run_button:pressed{
border: 2px solid #3FC380;
border-radius: 0px;
background-color: #3FC380;
color:#ffffff}''')
                        self.clear_btn.show()
                        # self.run_button.show()
                        pass
                    else:
                        # self.run_button.hide()
                        pass
                else:
                    self.save_btn.hide()

                # Remove all highlights
                self.remove_node_highlights()
                # highlight the selected node
                self.current_tree[str(event)].set_flag(True)
                self.regenerate_tree()
                self.right_selected_node = None
            else:
                print '[' + event + '] NOT found in current tree...'

    def remove_node_highlights(self):
        for n in self.current_tree.itervalues():
            if n.flag_ == True:
                n.set_flag(False)

    def node_rightclick_cb(self,event):
        # Get Click Interface Pose
        # experimental
        self.left_selected_node = None
        # experimental
        currentPos = self.mapFromGlobal(QCursor.pos())
        rospy.logwarn(currentPos)
        self.context_popup.move(currentPos.x()-90,currentPos.y()-70)

        if event == 'none':
            self.right_selected_node = None
            self.selected_node_field.setText('SELECTED NODE: NONE')
            self.selected_node_field.set_color(colors['gray_light'])
            self.inspect_btn.hide()
            self.view_center_btn.hide()
            self.save_btn.hide()
            self.delete_btn.hide()
            # Update Context Menu
            self.context_popup.hide()
            self.close_drawer()
        else:
            if event in self.current_tree:
                # Update Context Menu
                self.context_popup.show()
                if self.current_node_types[event] == 'LOGIC':
                    self.context_fold_btn.show()
                    if self.current_tree[str(event)].collapsed == False:
                        self.context_fold_btn.setText('FOLD')
                    else:
                        self.context_fold_btn.setText('UNFOLD')
                else:
                    self.context_fold_btn.hide()
                # Update Inspected Node Field
                self.selected_node_field.setText(str(event).upper())
                self.selected_node_field.set_color(colors['highlight_red'])
                self.inspect_btn.show()
                self.right_selected_node = event
                
            else:
                print '[' + event + '] NOT found in current tree...'

    def inspect_node(self):
        self.selected_node_field.set_color(colors['pink'])
        self.clear_node_info()
        self.open_drawer()
        if self.left_selected_node:
            self.current_node_generator = self.all_generators[self.current_plugin_names[self.left_selected_node]]
        else:
            self.current_node_generator = self.all_generators[self.current_plugin_names[self.right_selected_node]]
        self.current_node_generator.name.set_read_only(True)
        if self.left_selected_node:
            self.current_node_generator.load(self.current_node_info[self.left_selected_node])
            self.current_node_type = self.current_node_types[self.left_selected_node]
            self.current_node_plugin_name = self.current_plugin_names[self.left_selected_node]
        else:    
            self.current_node_generator.load(self.current_node_info[self.right_selected_node])
            self.current_node_type = self.current_node_types[self.right_selected_node]
            self.current_node_plugin_name = self.current_plugin_names[self.right_selected_node]
        self.drawer.node_info_layout.addWidget(self.current_node_generator)
        self.regenerate_btn.show()
        self.context_popup.hide()
        # self.save_state()
        pass

    def regenerate_node(self):
        if self.left_selected_node:
            current_name = self.left_selected_node
        else:
            current_name = self.right_selected_node

        if current_name:
            # current_name = self.right_selected_node
            replacement_node = self.current_node_generator.generate()
            if type(replacement_node) == str:
                rospy.logerr(str(replacement_node))
            else:
                current_child = self.current_tree[current_name]
                current_parent = self.current_tree[current_name].get_parent()
                if current_parent.replace_child(current_child, replacement_node):
                    self.current_tree[current_name] = replacement_node
                    self.current_node_generator.name.set_read_only(False)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.regenerate_tree()
                    self.regenerate_btn.hide()
                    self.close_drawer()
                    self.inspect_btn.hide()
                    self.view_center_btn.hide()
                    # self.save_state()
                else:
                    rospy.logwarn('Child to replace was not a member of the parent. Something went wrong.')
        
# Add and delete nodes ---------------------------------------------------------
    # This is a little brittle because it assumes that the name of the root plugin 
    # will not change... which I think is ok considering it is a core component
    def generate_root(self):
        rospy.logwarn('Generating new ROOT node')
        self.all_generators['Root'] = self.plugins['Root']['module']()
        self.current_node_generator = self.all_generators['Root']
        self.current_node_type = self.plugins['Root']['type']
        self.current_node_plugin_name = 'Root'
        node_to_add = self.current_node_generator.generate()
        if type(node_to_add) == str:
            rospy.logerr(str(node_to_add))
        else:
            current_name = self.current_node_generator.get_name()
            self.current_tree[current_name] = node_to_add
            self.current_node_info[current_name] = self.current_node_generator.save()
            self.current_plugin_names[current_name] = self.current_node_plugin_name
            self.current_node_types[current_name] = self.current_node_type
            self.root_node = node_to_add
            self.regenerate_tree()

    def add_root_cb(self):
        rospy.logwarn('adding ROOT node')
        if self.current_node_type != None:
            if self.root_node == None:
                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.root_node = node_to_add
                    self.regenerate_tree()
                    self.close_drawer()
                    # self.save_state()
                    self.node_leftclick_cb(current_name)
            else:
                rospy.logwarn('THERE IS ALREADY A ROOT NODE')
        else:
            rospy.logwarn('NO NODE SELECTED')


    
    def add_child_cb(self):
        if self.current_node_type != None:
            rospy.logwarn('adding node of type ' + str(self.current_node_type))
            if self.left_selected_node == None:
                rospy.logerr('There is no parent node selected')
            else:

                if not self.current_node_types[self.left_selected_node] == 'LOGIC':
                    rospy.logwarn('Parent must be a logic node')
                    return

                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    self.current_tree[self.left_selected_node].add_child(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    ## debugging
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.regenerate_tree()
                    # Prime GUI for another node of the same type
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                    self.drawer.node_info_layout.addWidget(self.current_node_generator)
                    # self.save_state()
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

    def add_sibling_before_cb(self):
        print 'adding sibling node of type ' + self.current_node_type
        if self.current_node_type != None:
            if self.left_selected_node == None:
              rospy.logerr('There is no left sibling node selected')
            else:
                
                if not self.current_node_types[self.left_selected_node] == 'LOGIC':
                    rospy.logwarn('Parent must be a logic node')
                    return

                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    sibling_node = self.current_tree[self.left_selected_node]
                    sibling_node.add_sibling_before(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type 
                    self.regenerate_tree()
                    # Prime GUI for another node of the same type
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                    self.drawer.node_info_layout.addWidget(self.current_node_generator)
                    # self.save_state()
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

    def replace_child_cb(self):
        if self.left_selected_node != None:
            current_name = self.left_selected_node
            rospy.logwarn(current_name)
            new_name = self.current_node_generator.get_name()
            rospy.logwarn(new_name)
            replacement_node = self.current_node_generator.generate()
            # current_name = self.current_node_generator.get_name()
            current_child = self.current_tree[current_name]
            current_parent = self.current_tree[current_name].get_parent()
            if current_parent.replace_child(current_child, replacement_node):
                self.current_tree[new_name] = replacement_node
                self.current_node_info[new_name] = self.current_node_generator.save()
                self.current_plugin_names[new_name] = self.current_node_plugin_name
                self.current_node_types[new_name] = self.current_node_type
                if self.current_node_types[new_name] == 'LOGIC':
                    self.delete_node(current_name,keep_children=True)
                else:
                    self.delete_node(current_name,keep_children=False)
                self.regenerate_tree()
                self.close_drawer()
                # self.save_state()
                if self.current_node_types[current_name] == 'LOGIC':
                    self.node_leftclick_cb(current_name)
            else:
                rospy.logerr('Replacing node failed: '+ current_name + ' , '+ new_name)

    def add_sibling_after_cb(self):
        print 'adding sibling node of type ' + self.current_node_type
        if self.current_node_type != None:
            if self.left_selected_node == None:
              rospy.logerr('There is no left sibling node selected')
            else:
                
                if not self.current_node_types[self.left_selected_node] == 'LOGIC':
                    rospy.logwarn('Parent must be a logic node')
                    return

                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    sibling_node = self.current_tree[self.left_selected_node]
                    sibling_node.add_sibling_after(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type 
                    self.regenerate_tree()
                    # Prime GUI for another node of the same type
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                    self.drawer.node_info_layout.addWidget(self.current_node_generator)
                    # self.save_state()
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

    def clear_all(self):
        if self.clear_confirm == False:
            self.clear_btn.setText('CONFIRM CLEAR ?')
            self.clear_cancel_btn.show()
            self.clear_confirm = True
            self.clear_btn.set_color(colors['orange'])
        else:
            self.clear_btn.setText('CLEAR')
            self.clear_confirm = False 
            self.clear_btn.set_color(colors['dark_red'])
            self.clear_cancel_btn.hide()   
            # Stop Execution if running
            if self.root_node is not None:
                self.dot_widget.zoom_image(1, center=True)
                self.stop_tree()

                # self.root_node = None

                self.dot_widget.set_dotcode('digraph behavior_tree {}')
                self.clear_tree()
                # Update Context Menu
                self.context_popup.hide()
                # Update Selected Node
                self.left_selected_node = None
                self.selected_node_field.setText('SELECTED NODE: NONE')
                self.selected_node_field.set_color(colors['gray_light'])
                self.inspect_btn.hide()
                self.view_center_btn.hide()
                # self.run_button.hide()
                self.load_sub_btn.hide()
                self.remove_sub_btn.hide()
                self.remove_saved_node_btn.hide()
                self.selected_subtree = None

    def clear_all_cancel(self):
        self.clear_cancel_btn.hide()
        self.clear_btn.setText('CLEAR')
        self.clear_confirm = False 
        self.clear_btn.set_color(colors['dark_red']) 


    def clear_tree(self):
        self.current_tree.clear()
        self.current_generators.clear()
        self.current_node_types.clear()
        self.current_plugin_names.clear()       
        self.root_node = None
        # self.save_state()

    def clear_node(self,name):
        self.current_tree.pop(name)
        self.current_node_info.pop(name)
        self.current_node_types.pop(name)
        self.current_plugin_names.pop(name)
        # self.save_state()

    def delete_cb(self):
        if self.root_node == None:
            # self.clear_tree()
            # self.dot_widget.set_dotcode('digraph behavior_tree {}') 
            pass
        else:
            if self.left_selected_node == None:
                rospy.logerr('There is no node selected in the gui')
                return
            else:
                self.delete_node(self.left_selected_node)
            self.left_selected_node = None

    def delete_node(self,name,keep_children=False):
        node = self.current_tree[name]
        if keep_children == False:
            for child in node.get_child_names():
                self.delete_node(child,keep_children)
        if node == self.root_node:
            # self.root_node = None
            self.dot_widget.set_dotcode('digraph behavior_tree {}')
            self.clear_tree()
        else:
            node.remove_self()
            self.clear_node(name)
            self.regenerate_tree()
        rospy.logwarn('Removed node ['+name+']')

    def regenerate_tree(self,runtime=False,center=False):
        if self.root_node == None:
            self.dot_widget.set_dotcode('digraph behavior_tree {}')
        else:
            #rospy.logwarn(self.root_node.generate_dot(runtime))
            self.dot_widget.set_dotcode(self.root_node.generate_dot(runtime),False)
        # rospy.logwarn(self.current_tree)

# UI Specific Functions --------------------------------------------------------
    def clear_node_info(self):
        for i in reversed(range(self.drawer.node_info_layout.count())): 
            item = self.drawer.node_info_layout.takeAt(i)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

    def keyPressEvent(self, event):
        if type(event) == QtGui.QKeyEvent:
            if event.key() == 16777220:
                if self.selected_subtree != None:
                    self.load_selected_subtree()
                elif 'root' in self.current_node_plugin_name.lower():
                    self.add_root_cb()
                else:
                    self.add_child_cb()
            elif event.key() == 16777223:
                self.delete_cb()
            elif event.key() == 16777219:
                self.delete_cb()
            elif event.key() == 32: # space
                self.view_fit()
            elif event.key() == 70: # "f"
                # self.view_center()
                self.collapse_node()
            elif event.key() == 87: # "w"
                self.show_waypoint_manager()
            elif event.key() == 16777216: # "esc"
                self.close_drawer()
            else:
                # rospy.logwarn(event.key())
                pass
            event.accept()
        else:
            event.ignore()
    
# App Specific Functions --------------------------------------------------------------
    def closeEvent(self, event):
        print 'saving info'
        self.settings.setValue('size', self.size())
        self.settings.setValue('pos', self.pos())
        self.settings.sync()
        event.accept()

    def check_ok(self):
        self.update()
        self.robot_.update_status()
        if rospy.is_shutdown():
          self.close()
          self.app_.exit()

# MAIN #######################################################
if __name__ == '__main__':
  rospy.init_node('beetree',anonymous=True)
  app = QApplication(sys.argv)
  wrapper = Instructor(app)
  # Running
  app.exec_()
  # Done








