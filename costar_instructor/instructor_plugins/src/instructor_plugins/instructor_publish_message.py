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
# from std_srvs.srv import Empty
import os

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodePublishMessageGUI(NodeGUI):
    def __init__(self):
        super(NodePublishMessageGUI,self).__init__(color='purple')
        self.title.setText('Publish Message')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','purple')
        self.wait_finish.set_field('1')

        self.message_contents = NamedField('message','', 'purple')
        self.message_topic = NamedField('Rostopic','', 'purple')
        self.message_topic.set_field('info')

        self.note = NoteField('(1 = true, 0 = false)\nrostopic has prefix: /costar/messages/','purple')
        self.layout_.addWidget(self.message_contents)
        self.layout_.addWidget(self.message_topic)
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    
    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            full_topic_name = os.path.join('/costar/messages/',str(self.message_topic.get()))
            return NodeActionPublishMessage(self.get_name(),str(self.message_contents.get()), full_topic_name, int(self.wait_finish.get()))
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionPublishMessage(ServiceNode):
    def __init__(self,name,message_contents,topic_name,wait_finish):
        L = "Publish Message: " + message_contents + "\nto " + topic_name 
        super(NodeActionPublishMessage,self).__init__(name,L,colors['purple'].normal,"Message Server")
        # if topic_name == '':
        #     self.message_pub = rospy.Publisher('/instructor/message_server', String,queue_size=100)
        # else:
        self.message_pub = rospy.Publisher(topic_name, String,queue_size=100)
        self.message_to_publish = message_contents
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True

    def make_service_call(self,request,*args):
    	self.message_pub.publish(self.message_to_publish)
        self.finished_with_success = True
        return