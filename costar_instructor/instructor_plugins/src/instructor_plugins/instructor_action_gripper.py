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
from instructor_core.instructor_qt import NamedField, NoteField, ColorOptions
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c
# Driver services for ur5
#from robotiq_c_model_control.srv import *
from c_model_srvs import *

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodeActionGripperOpenGUI(NodeGUI):
    def __init__(self):
        super(NodeActionGripperOpenGUI,self).__init__(color='green')
        self.title.setText('GRIPPER OPEN ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionGripper(self.get_name(),self.get_label(),True,int(self.wait_finish.get()))
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeActionGripperCloseGUI(NodeGUI):
    def __init__(self):
        super(NodeActionGripperCloseGUI,self).__init__(color='green')
        self.title.setText('GRIPPER CLOSE ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionGripper(self.get_name(),self.get_label(),False,int(self.wait_finish.get()))
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionGripper(ServiceNode):
    def __init__(self,name,label,open_gripper,wait_finish):
        self.open_gripper = open_gripper
        if self.open_gripper == True:
            self.type = 'OPEN'
        else:
            self.type = 'CLOSE'
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True
        # L = 'Action\\n'+label+'\\n WAIT ['+str(self.wait_finish)+']'
        L = self.type+' GRIPPER'
        super(NodeActionGripper,self).__init__(name,L,'#26A65B',"Gripper %s"%self.type)

    def make_service_call(self,request,*args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('/robotiq_c_model_control/Open')
        except rospy.ROSException as e:
            rospy.logerr('Could not find gripper service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            gripper_open_proxy = rospy.ServiceProxy('/robotiq_c_model_control/Open',Open)                
            # Send Open Command
            rospy.loginfo('Gripper '+self.type+' Started')
            msg = OpenRequest()
            msg.state = self.open_gripper
            msg.wait = self.wait_finish
            result = gripper_open_proxy(msg)
            if 'FAILURE' in str(result.ack):
                rospy.logwarn('Gripper '+self.type+' failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.logwarn('Gripper '+self.type+' Finished')
                self.finished_with_success = True
                return

        except (rospy.ServiceException), e:
            rospy.logerr('There was a problem with the service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return
