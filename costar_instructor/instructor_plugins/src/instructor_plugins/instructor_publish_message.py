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
from std_srvs.srv import Empty

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodePublishMessageGUI(NodeGUI):
    def __init__(self):
        super(NodePublishMessageGUI,self).__init__(color='purple')
        self.title.setText('Publish Message')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','purple')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','purple')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionPublishMessage(self.get_name(),self.get_label(),int(self.wait_finish.get()))
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionPublishMessage(ServiceNode):
    def __init__(self,name,label,wait_finish):
        L = "Publish Message: " + label
        super(NodeActionPublishMessage,self).__init__(name,L,colors['purple'].normal,"Message Server")
        self.message_pub = rospy.Publisher('/instructor/message_server', String,queue_size=100)
        self.message_to_publish = label
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True

    def make_service_call(self,request,*args):
    	self.message_pub.publish(self.message_to_publish)
        self.finished_with_success = True
        return