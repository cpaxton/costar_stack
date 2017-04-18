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
# Driver services for force sensor
from robotiq_force_torque_sensor.msg import *
import robotiq_force_torque_sensor.srv as ft_srv

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodeActionForceZeroGUI(NodeGUI):
    def __init__(self):
        super(NodeActionForceZeroGUI,self).__init__(color='green')
        self.title.setText('ZERO FORCE SENSOR')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.note = NoteField('This will zero the force sensor','green')
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full()]):
            return NodeActionForceZero(self.get_name(),self.get_label())
        else:
            rospy.logerr('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeActionForceZero(ServiceNode):
    def __init__(self,name,label):
        L = 'ZERO FORCE'
        super(NodeActionForceZero,self).__init__(name,L,'#26A65B', "Force Zero")

    def make_service_call(self,request,*args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('/robotiq_force_torque_sensor_acc')
        except rospy.ROSException as e:
            rospy.logerr('Could not find force_zero service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            force_zero_open_proxy = rospy.ServiceProxy('/robotiq_force_torque_sensor_acc',ft_srv.sensor_accessor)
            # Send Open Command
            rospy.loginfo('Zero Sensor Started')
            msg = ft_srv.sensor_accessorRequest()
            msg.command = "SET ZRO"
            result = force_zero_open_proxy(msg)
            if 'Done' not in str(result):
                rospy.logwarn('Zero Sensor failed with reply: '+ str(result))
                self.finished_with_success = False
                return
            else:
                rospy.logwarn('Zero Sensor Finished')
                self.finished_with_success = True
                return

        except (rospy.ServiceException), e:
            rospy.logerr('There was a problem with the service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return