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
class NodeActionStopGUI(NodeGUI):
    def __init__(self):
        super(NodeActionStopGUI, self).__init__('green')
        self.title.setText('STOP ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.stop_time = NamedField('Stop Time', '','green')
        self.layout_.addWidget(self.stop_time)

    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
        
    def generate(self):
        if all([self.name.full(),  self.stop_time.full()]):
            return NodeActionStop(self.get_name(),  self.get_label(),  float(self.stop_time.get()))
        else:
            return 'ERROR: STOP action check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionStop(Node):
    def __init__(self, name, label, wait_time):
        L = 'STOP IN '+str(wait_time)+'s'
        super(NodeActionStop, self).__init__(name, L, '#26A65B')
        self.wait_time = wait_time
        # Reset params
        self.stop_thread = Thread(target=self.stop,  args=(self.wait_time, 1))
        self.running = False
        self.needs_reset = False

    def get_node_type(self):
        return 'STOP_ACTION'

    def get_node_name(self):
        return 'Stop_Action'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Stop Service [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                try:
                    self.stop_thread.start()
                    rospy.loginfo('STOP ACTION ['+self.name_+']: STARTED')
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception,  errtxt:
                    rospy.logwarn('STOP ACTION ['+self.name_+']: FAILED TO START THREAD')
                    self.running = False
                    self.needs_reset = True
                    self.set_color(colors['gray'].normal)
                    return self.set_status('FAILURE')
            else:
                # If thread is running
                if self.stop_thread.is_alive():
                    rospy.logwarn('STOP ACTION ['+self.name_+']: SLEEPING')
                    return self.set_status('RUNNING')
                else:
                    rospy.logwarn('STOP ACTION ['+self.name_+']: FINISHED')
                    self.running = False
                    self.needs_reset = True
                    self.set_color(colors['gray'].normal)
                    return self.set_status('SUCCESS')

    def stop(self, request, *args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('/simple_ur_msgs/SetStop')
        except rospy.ROSException as e:
            rospy.logerr('Could not find stop service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            stop_proxy = rospy.ServiceProxy('/simple_ur_msgs/SetStop',SetStop)
            
            msg = simple_ur_msgs.srv.SetStopRequest()
            # Send Servo Command
            result = stop_proxy(msg)
            rospy.logwarn('Stop Move Finished')
            rospy.logwarn('Robot driver reported: '+str(result.ack))
            self.finished_with_success = True
            return

        except (rospy.ServiceException), e:
            rospy.logerr('There was a problem with the service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return

    def reset_self(self):
        self.stop_thread = Thread(target=self.stop,  args=(self.wait_time, 1))
        self.running = False
        self.needs_reset = False
        #self.set_color('#26A65B')
        self.set_color(colors['green'].normal)

