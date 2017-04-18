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
class NodeActionRecordDataGUI(NodeGUI):
    def __init__(self):
        super(NodeActionRecordDataGUI,self).__init__(color='green')
        self.title.setText('RECORD DATA')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionRecordData(self.get_name(),self.get_label(),int(self.wait_finish.get()))
        else:
            rospy.logerr('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeActionRecordData(ServiceNode):
    def __init__(self,name,label,wait_finish):
        L = "RECORD DATA"
        super(NodeActionRecordData,self).__init__(name,L,'#26A65B',"Recorder")
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True

    def make_service_call(self,request,*args):
        # Check to see if service exists
        service_name = '/record_camera'
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find recorder service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            recorder_open_proxy = rospy.ServiceProxy(service_name,Empty)
            # Send Open Command
            rospy.loginfo('Recorder Started')
            result = recorder_open_proxy()
            self.finished_with_success = True
            return

        except (rospy.ServiceException), e:
            rospy.logerr('There was a problem with the service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return
