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
import beetree; from beetree import Node, NodeAction
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
class NodeActionForceZero(Node):
    def __init__(self,name,label):
        super(NodeActionForceZero,self).__init__(name,label)
        # L = 'Action\\n'+label+'\\n WAIT ['+str(self.wait_finish)+']'
        L = 'ZERO FORCE'
        super(NodeActionForceZero,self).__init__(name,L,'#26A65B')
        # Reset params
        self.force_zero_service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Force Zero [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.force_zero_service_thread.start()
                        rospy.loginfo('Zero Sensor [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('Zero Sensor [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.force_zero_service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('Zero Sensor [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('Zero Sensor [' + self.name_ + '] failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')

    def reset_self(self):
        self.force_zero_service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False

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