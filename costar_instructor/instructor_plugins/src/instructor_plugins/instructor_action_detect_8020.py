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

colors = ColorOptions().colors

# Node Wrappers -----------------------------------------------------------
class NodeActionDetect8020GUI(NodeGUI):
    def __init__(self):
        super(NodeActionDetect8020GUI,self).__init__()
        self.title.setText('8020 DETECT ONCE')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full(),self.label.full()]):
            return NodeActionDetect8020(self.get_name(),self.get_label())
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

# Nodes -------------------------------------------------------------------
class NodeActionDetect8020(Node):
    def __init__(self,name,label):
        L = 'DETECT_8020\\n[\\"'+label.upper()+'\\"]'#+'\\n['+self.type+']'
        super(NodeActionDetect8020,self).__init__(name,L,'#26A65B')
        # Reset params
        self.detect_pub = rospy.Publisher('plates/detect', Empty)
        self.sleep_thread = Thread(target=self.sleep,  args=(.25, 1))
        self.running = False
        self.needs_reset = False
        self.runs = 0
    def get_node_type(self):
        return 'ACTION'
    def get_node_name(self):
        return 'Action'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Detect [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                try:
                    # self.detect_pub.publish(Empty())
                    rospy.loginfo('DETECTION 8020 ['+self.name_+']: STARTED')
                    self.sleep_thread.start()
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception,  errtxt:
                    rospy.logwarn(' ['+self.name_+']: FAILED TO START THREAD')
                    rospy.logwarn(errtxt)
                    self.running = False
                    self.needs_reset = True
                    return self.set_status('FAILURE')
            else:
                # If thread is running
                if self.sleep_thread.is_alive():
                    rospy.logwarn('DETECTION 8020 ['+self.name_+']: SLEEPING')
                    return self.set_status('RUNNING')
                else:
                    rospy.logwarn('DETECTION 8020 ['+self.name_+']: FINISHED')
                    self.running = False
                    self.needs_reset = True
                    return self.set_status('SUCCESS')

    def sleep(self, request, *args):
        rospy.sleep(request)

    def reset_self(self):
        self.sleep_thread = Thread(target=self.sleep,  args=(.25, 1))
        self.running = False
        self.needs_reset = False
