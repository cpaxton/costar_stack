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
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, NoteField, ColorOptions
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c
# Driver services for ur5
from c_model_srvs.srv import *

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodeActionPedalGUI(NodeGUI):
    def __init__(self):
        super(NodeActionPedalGUI,self).__init__(color='green')
        self.title.setText('FOOT PEDAL ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.duration = NamedField('Duration', '','green')
        self.note = NoteField('(Time (s) to press pedal)','green')
        self.layout_.addWidget(self.duration)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.duration.full()]):
            return NodeActionPedal(self.get_name(),self.get_label(),int(self.duration.get()))
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionPedal(Node):
    def __init__(self,name,label,duration):
        self.duration = duration
        L = 'PRESS PEDAL '+str(self.duration)+'s'
        super(NodeActionPedal,self).__init__(name,L,'#26A65B')
        # Reset params
        self.pedal_pub_ = rospy.Publisher('/foot_pedal_actuator/toggle_led', UInt16)
        self.sleep_thread = Thread(target=self.sleep,  args=(self.duration + .25, 1))
        self.running = False
        self.needs_reset = False
    def get_node_type(self):
        return 'ACTION'
    def get_node_name(self):
        return 'Action'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Pedal [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                try:
                    self.pedal_pub_.publish(UInt16(self.duration))
                    rospy.loginfo('PEDAL PRESS ['+self.name_+']: STARTED')
                    self.sleep_thread.start()
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception,  errtxt:
                    rospy.logwarn('PEDAL ENABLE ['+self.name_+']: FAILED TO START THREAD')
                    self.running = False
                    self.needs_reset = True
                    return self.set_status('FAILURE')
            else:
                # If thread is running
                if self.sleep_thread.is_alive():
                    rospy.logwarn('PEDAL ['+self.name_+']: SLEEPING')
                    return self.set_status('RUNNING')
                else:
                    rospy.logwarn('PEDAL ['+self.name_+']: FINISHED')
                    self.running = False
                    self.needs_reset = True
                    return self.set_status('SUCCESS')

    def sleep(self, request, *args):
        rospy.sleep(request)

    def reset_self(self):
        self.sleep_thread = Thread(target=self.sleep,  args=(.25, 1))
        self.running = False
        self.needs_reset = False

















    # def execute(self):
    #     if self.needs_reset:
    #         rospy.loginfo('Pedal [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
    #         return self.get_status()
    #     else:
    #         if self.running == False:
    #             if self.type == 'ON':
    #                 self.pedal_pub_.publish(UInt16(3))
    #                 rospy.sleep(.5)
    #                 rospy.logwarn('PEDAL ENABLE ['+self.name_+']: FINISHED')
    #             elif self.type == 'OFF':
    #                 self.pedal_pub_.publish(UInt16(0))
    #                 rospy.sleep(.5)
    #                 rospy.logwarn('PEDAL DISABLE ['+self.name_+']: FINISHED')

    #             self.running = True
    #             return self.set_status('RUNNING')
    #         else:
    #             self.running = False
    #             self.needs_reset = True
    #             return self.set_status('SUCCESS')
            
    # def reset_self(self):
    #     self.running = False
    #     self.needs_reset = False
