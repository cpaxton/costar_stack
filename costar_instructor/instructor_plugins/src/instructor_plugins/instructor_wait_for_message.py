#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import Empty
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

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodeWaitForMessageGUI(NodeGUI):
    def __init__(self):
        super(NodeWaitForMessageGUI,self).__init__(color='purple')
        self.title.setText('Wait For Message')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','purple')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','purple')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    
    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionWaitForMessage(self.get_name(),self.get_label(),int(self.wait_finish.get()))
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionWaitForMessage(ServiceNode):
    def __init__(self,name,label,wait_finish):
        L = "Waiting message from:\n" + label
        super(NodeActionWaitForMessage,self).__init__(name,L,colors['purple'].normal,"Message Collector")

        self.message_source = str(label) 
        rospy.Subscriber(self.message_source, Empty, self.callback)
        self.message_received = False
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True

    def make_service_call(self,request,*args):
    	self.message_received = False
        while not self.message_received:
            rospy.loginfo('Waiting for empty message from: %s'%self.message_source)
            rospy.sleep(0.5)
        self.finished_with_success = True
        return

    def callback(self,msg):
        self.message_received = True
        return