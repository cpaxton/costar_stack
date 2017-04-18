
import rospy
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
import rospkg
from beetree import *

import costar_robot_msgs
from costar_robot_msgs.srv import *
from instructor_gui_components import *
from instructor_core.instructor_qt import *
from std_srvs.srv import Empty

from instructor_gui_components import *

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

        # INFO #
        self.info_textbox = TextEdit(self,'INFO_TEXTBOX','',color=colors['gray_light'])
        self.info_textbox.setReadOnly(True)
        self.info_textbox.hide()
        self.button_layout.addWidget(self.info_textbox)

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
                    rospy.logerr('Could not find the tf frame for the robot endpoint: %s'%str(e))
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
                    self.info_textbox.notify(add_waypoint_proxy(msg))
                    self.update_waypoints()
                    self.waypoint_page_widget.name_field.setText('')
                except rospy.ServiceException, e:
                    self.info_textbox.notify(e,'warn')
            else:
                self.info_textbox.notify('You need to input a name for the waypoint','warn')
        else:
            self.info_textbox.notify('Adding Relative Waypoint with landmark ['+str(self.selected_landmark)+'] and waypoint ['+str(self.new_relative_name)+']')
            if self.selected_landmark != None:
                landmark_frame = self.landmarks[str(self.selected_landmark)].strip('/')
                self.info_textbox.notify(landmark_frame)
                try:
                    F_landmark = tf_c.fromTf(self.listener_.lookupTransform('/world',landmark_frame,rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    self.info_textbox.notify('Selected landmark not found','error')
                    return                
            else:
                self.info_textbox.notify('You did not select a landmark','error')
                return
            if not self.new_relative_name:
                self.info_textbox.notify('You did not enter a waypoint name')
                return

            # Get relative pose
            try:
                F_landmark_endpoint = tf_c.fromTf(self.listener_.lookupTransform('/'+landmark_frame,'/endpoint',rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                self.info_textbox.notify(e,'error')
                return

            try:
                rospy.wait_for_service('/instructor_core/AddWaypoint',2)
            except rospy.ROSException as e:
                self.info_textbox.notify(e,'error')
                return

            for w in self.found_rel_waypoints:
                if '--' in w:
                    if self.new_relative_name == w.split('--')[0].replace('/',''):
                        self.info_textbox.notify('You must pick a name that is not a sequence')
                        return
            try:
                add_waypoint_proxy = rospy.ServiceProxy('/instructor_core/AddWaypoint',AddWaypoint)
                msg = AddWaypointRequest()
                msg.name = '/' + self.new_relative_name
                msg.relative_pose = tf_c.toMsg(F_landmark_endpoint)
                msg.relative_frame_name = '/'+landmark_frame
                self.info_textbox.notify(add_waypoint_proxy(msg))
                self.update_waypoints()
                # Reset
                self.relative_page_widget.name_field.setText('')
                self.relative_page_widget.landmark_field.setText('NONE')
                self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['gray'].normal+';color:#ffffff')
                self.selected_landmark = None
            except rospy.ServiceException, e:
                self.info_textbox.notify(e,'warn')

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
                self.info_textbox.notify('wp: '+str(wp) + '\nname: '+self.new_fixed_name)
                if self.new_fixed_name in wp:
                    self.info_textbox.notify('name found... ' + self.new_fixed_name)
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
                    self.info_textbox.notify(add_waypoint_proxy(msg))
                    self.update_waypoints()
                    # self.waypoint_page_widget.name_field.setText('')
                except rospy.ServiceException, e:
                    self.info_textbox.notify(e)
            else:
                self.info_textbox.notify('You need to input a name for the waypoint')
        else:
            message = ('Adding Relative Waypoint with landmark ['+str(self.selected_landmark)+'] and waypoint ['+str(self.new_relative_name)+']')
            if self.selected_landmark != None:
                landmark_frame = self.landmarks[str(self.selected_landmark)].strip('/')
                rospy.logwarn(message + landmark_frame)
                try:
                    F_landmark = tf_c.fromTf(self.listener_.lookupTransform('/world',landmark_frame,rospy.Time(0)))
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    self.info_textbox.notify(message+'\nSelected landmark not found','error')
                    return                
            else:
                self.info_textbox.notify('You did not select a landmark')
                return
            if not self.new_relative_name:
                self.info_textbox.notify('You did not enter a waypoint name')
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
                self.info_textbox.notify(add_waypoint_proxy(msg))
                self.update_waypoints()
                # Reset
                # self.relative_page_widget.name_field.setText('')
                # self.relative_page_widget.landmark_field.setText('NONE')
                # self.relative_page_widget.landmark_field.setStyleSheet('background-color:'+colors['gray'].normal+';color:#ffffff')
                # self.selected_landmark = None
            except rospy.ServiceException, e:
                self.info_textbox.notify(e)

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
                self.info_textbox.notify(remove_waypoint_proxy(msg))
                self.update_waypoints()
            except rospy.ServiceException, e:
                self.info_textbox.notify(e,'error')

    def update_waypoints(self):
        # Update FIXED Waypoints
        rospy.wait_for_service('/instructor_core/GetWaypointList')
        self.found_waypoints = []
        self.waypoint_page_widget.waypoint_list.clear()
        try:
            get_waypoints_proxy = rospy.ServiceProxy('/instructor_core/GetWaypointList',GetWaypointList)
            self.found_waypoints = get_waypoints_proxy('').names
        except rospy.ServiceException, e:
            self.info_textbox.notify(e,'error')
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
            self.info_textbox.notify(e,'error')
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
            self.info_textbox.notify(e,'error')
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
                self.info_textbox.notify('Single Servo Move Started')
                result = pose_servo_proxy(msg)
                if 'FAILURE' in str(result.ack):
                    self.info_textbox.notify('Servo failed with reply: '+ str(result.ack))
                    return
                else:
                    self.info_textbox.notify('Single Servo Move Finished' + 'Robot driver reported: '+str(result.ack))
                    return

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ServiceException), e:
                self.info_textbox.notify('There was a problem with the tf lookup or service:\n'+e,'error')
                return


