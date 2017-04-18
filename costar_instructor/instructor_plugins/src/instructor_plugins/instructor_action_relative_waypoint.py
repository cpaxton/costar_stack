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
from instructor_core.instructor_qt import NamedField, ColorOptions
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c
# Driver services for ur5
import costar_robot_msgs
from costar_robot_msgs.srv import *

colors = ColorOptions().colors

# Node Wrappers -----------------------------------------------------------
class NodeActionRelativeWaypointGUI(NodeGUI):
    def __init__(self):
        super(NodeActionRelativeWaypointGUI,self).__init__('green')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/action_waypoint.ui'

        self.title.setText('MOVE TO WAYPOINT ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.setStyleSheet('background-color:'+colors['green'].normal+' ; color:#ffffff')

        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.layout_.addWidget(self.waypoint_ui)

        self.new_waypoint_name = None
        self.waypoint_selected = False
        self.command_waypoint_name = None
        self.command_vel = .5
        self.command_acc = .5
        self.listener_ = tf.TransformListener()

        self.waypoint_ui.waypoint_list.itemClicked.connect(self.waypoint_selected_from_list)
        self.waypoint_ui.acc_slider.valueChanged.connect(self.acc_changed)
        self.waypoint_ui.vel_slider.valueChanged.connect(self.vel_changed)
        self.waypoint_ui.refresh_btn.clicked.connect(self.update_relative_waypoints)

        self.update_relative_waypoints()

    def update_relative_waypoints(self):
        rospy.wait_for_service('/instructor_core/GetRelativeWaypointList')
        found_waypoints = []
        self.waypoint_ui.waypoint_list.clear()
        try:
            get_waypoints_proxy = rospy.ServiceProxy('/instructor_core/GetRelativeWaypointList',GetRelativeWaypointList)
            found_waypoints = get_waypoints_proxy('').names
        except rospy.ServiceException, e:
            rospy.logerr(e)
        for w in found_waypoints:
            self.waypoint_ui.waypoint_list.addItem(QListWidgetItem(w.strip('/')))
        self.waypoint_ui.waypoint_label.setText('NONE')
        self.waypoint_ui.waypoint_label.setStyleSheet('background-color:#223F35 ; color:#6C897A')

    def vel_changed(self,t):
        self.waypoint_ui.vel_field.setText(str(float(t)))
        self.command_vel = float(t)/100

    def acc_changed(self,t):
        self.waypoint_ui.acc_field.setText(str(float(t)))
        self.command_acc = float(t)/100

    def waypoint_selected_from_list(self,item):
        self.set_command_waypoint(str(item.text()))

    def set_command_waypoint(self,waypoint_name):
        # rospy.logwarn('Setting Command Waypoint')
        self.waypoint_ui.waypoint_label.setText(waypoint_name)
        self.waypoint_ui.waypoint_label.setStyleSheet('background-color:'+colors['green'].hover+' ; color:#ffffff')
        self.waypoint_selected = True
        self.command_waypoint_name = waypoint_name

    def save_data(self,data):
        data['waypoint_name'] = {'value':self.command_waypoint_name}
        data['vel'] = {'value':self.command_vel}
        data['acc'] = {'value':self.command_acc}
        return data

    def load_data(self,data):
        if data.has_key('waypoint_name'):
            if data['waypoint_name']['value']!=None:
                self.set_command_waypoint(data['waypoint_name']['value'])
        if data.has_key('vel'):
            if data['vel']['value']!=None:
                self.command_vel = data['vel']['value']
                self.waypoint_ui.vel_field.setText(str(float(self.command_vel)*100))
                self.waypoint_ui.vel_slider.setSliderPosition(int(float(self.command_vel)*100))
        if data.has_key('acc'):
            if data['acc']['value']!=None:
                self.command_acc = data['acc']['value']
                self.waypoint_ui.acc_field.setText(str(float(self.command_acc)*100))
                self.waypoint_ui.acc_slider.setSliderPosition(int(float(self.command_acc)*100))
        self.update_relative_waypoints()

    def generate(self):
        if all([self.name.full(), self.command_waypoint_name]):
            # # rospy.loginfo('Generating Move with acc='+str(self.command_acc)+' and vel='+str(self.command_vel))
            return NodeActionRelativeWaypoint(self.get_name(),self.get_label(),self.command_waypoint_name,self.command_vel,self.command_acc,self.listener_)
        else:
            rospy.logerr('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

    def refresh_data(self):
        self.update_relative_waypoints()


# Nodes -------------------------------------------------------------------
class NodeActionRelativeWaypoint(ServiceNode):
    def __init__(self,name,label,waypoint_name,vel,acc,tfl):
        #L = 'MOVE RELATIVE TO\\n ['+waypoint_name.upper()+'] \nVelocity: %d%%\nAcceleration: %d%%'%(int(vel*100),int(acc*100))
        L = 'MOVE TO ['+waypoint_name.upper()+']\n(relative to object)\nVel: %d%%, Acc: %d%%'%(int(vel*100),int(acc*100))
        
        super(NodeActionRelativeWaypoint,self).__init__(name,L,'#26A65B',"Relative Waypoint Service")
        self.command_waypoint_name = waypoint_name
        self.command_acc = acc
        self.command_vel = vel
        self.listener_ = tfl

    def make_service_call(self,request,*args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('/costar/ServoToPose')
        except rospy.ROSException as e:
            rospy.logerr('Could not find servo service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            pose_servo_proxy = rospy.ServiceProxy('/costar/ServoToPose',ServoToPose)
            
            F_command_world = tf_c.fromTf(self.listener_.lookupTransform('/world', '/'+self.command_waypoint_name, rospy.Time(0)))
            F_base_world = tf_c.fromTf(self.listener_.lookupTransform('/world','/base_link',rospy.Time(0)))
            F_command = F_base_world.Inverse()*F_command_world
                
            msg = ServoToPoseRequest()
            msg.target = tf_c.toMsg(F_command)
            msg.accel = self.command_acc
            msg.vel = self.command_vel
            # Send Servo Command
            rospy.loginfo('Single Servo Move Started')
            result = pose_servo_proxy(msg)
            if 'FAILED' in str(result.ack):
                rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.loginfo('Single Servo Move Finished')
                rospy.loginfo('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ServiceException), e:
            rospy.logerr('There was a problem with the tf lookup or service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return
