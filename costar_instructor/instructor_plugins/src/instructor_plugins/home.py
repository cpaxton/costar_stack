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
#import simple_ur_msgs
#from simple_ur_msgs.srv import *
import costar_robot_msgs
from costar_robot_msgs.srv import *

colors = ColorOptions().colors

# Node Wrappers -----------------------------------------------------------
class NodeHomeGUI(NodeGUI):
    def __init__(self,plan=False):
        super(NodeHomeGUI,self).__init__('green')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/home.ui'

        if plan:
            title = "PLAN"
        else:
            title = "MOVE"
        self.title.setText('%s TO HOME ACTION'%title)
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.setStyleSheet('background-color:'+colors['green'].normal+' ; color:#ffffff')


        self.plan = plan
        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.layout_.addWidget(self.waypoint_ui)

        self.command_waypoint_name = None
        self.command_vel = .5
        self.command_acc = .5
        self.listener_ = tf.TransformListener()

        self.waypoint_ui.acc_slider.valueChanged.connect(self.acc_changed)
        self.waypoint_ui.vel_slider.valueChanged.connect(self.vel_changed)

    def vel_changed(self,t):
        self.waypoint_ui.vel_field.setText(str(float(t)))
        self.command_vel = float(t)/100

    def acc_changed(self,t):
        self.waypoint_ui.acc_field.setText(str(float(t)))
        self.command_acc = float(t)/100

    def save_data(self,data):
        data['vel'] = {'value':self.command_vel}
        data['acc'] = {'value':self.command_acc}
        return data

    def load_data(self,data):
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

    def generate(self):
        if all([self.name.full()]):
            # rospy.logwarn('Generating Move with acc='+str(self.command_acc)+' and vel='+str(self.command_vel))
            return NodeHome(self.get_name(),self.get_label(),self.command_vel,self.command_acc,self.plan)
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeHome(ServiceNode):
    def __init__(self,name,label,vel,acc,plan):
        if plan:
            L = 'PLAN TO HOME'
            service_description = "Plan to home Service"
        else:
            L = 'MOVE TO HOME\nVel: %d%%, Acc: %d%%'%(int(vel*100),int(acc*100))
            service_description = "Move to home Service"
        super(NodeHome,self).__init__(name,L,'#26A65B',"%s Service"%service_description)
        self.command_vel = vel
        self.command_acc = acc

        if plan:
            self.srv_name = 'Plan'
        else:
            self.srv_name = 'Servo'

    def make_service_call(self,request,*args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('costar/%sToHome'%self.srv_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find servo service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            pose_servo_proxy = rospy.ServiceProxy('costar/%sToHome'%self.srv_name,ServoToPose)
            
            msg = costar_robot_msgs.srv.ServoToPoseRequest()
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
                rospy.loginfo('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ServiceException), e:
            rospy.logerr('There was a problem with the tf lookup or service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return

class PlanToHomeGUI(NodeHomeGUI):
    def __init__(self):
        super(PlanToHomeGUI, self).__init__(plan=True)

class MoveToHomeGUI(NodeHomeGUI):
    def __init__(self):
        super(MoveToHomeGUI, self).__init__(plan=False)
