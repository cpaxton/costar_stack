#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField
# For testing the service node
from instructor_plugins.srv import *

# Sample Node Wrappers -----------------------------------------------------------
'''
class NodeActionSampleGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSampleGUI,self).__init__()

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return beetree.NodeAction(parent,self.get_name(),self.get_label())
        else:
            return 'ERROR: node not properly defined'

class NodeServiceSampleGUI(NodeGUI):
    def __init__(self):
        super(NodeServiceSampleGUI,self).__init__()

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return NodeServiceSample(parent,self.get_name(),self.get_label())
        else:
            return 'ERROR: node not properly defined'
'''

class NodeQueryClosestObjectGUI(NodeGUI):
    def __init__(self):
        super(NodeQueryClosestObjectGUI,self).__init__()

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return NodeQueryClosestObject(parent,self.get_name(),self.get_label())
        else:
            return 'ERROR: node not properly defined'

# Sample Nodes -------------------------------------------------------------------
class NodeQueryClosestObject(Node):
    def __init__(self,parent,name,label):
        color='#5B8EEB'
        super(NodeQueryClosestObject,self).__init__(False,parent,name,label,color)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        # TODO
        print 'Executing Service: ' + self.name_
        self.node_status_ = 'SUCCESS'
        print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_

'''
from threading import Thread
class NodeServiceSample(Node):
    def __init__(self,parent,name,label):
        super(NodeServiceSample,self).__init__(False,parent,name,label,'#92D665')
        self.service_thread = Thread(target=self.make_service_call, args=('request',1))
        self.running = False
        self.finished_with_success = None
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        if not self.running: # Thread is not running
            if self.finished_with_success == None: # Service was never called
                try:
                    self.service_thread.start()
                    return set_status('RUNNING')
                    self.running = True
                except Exception, errtxt:
                    return set_status('FAILURE')
                    self.running = False
        else:
            # If thread is running
            if self.service_thread.is_alive():
                return set_status('RUNNING')
            else:
                if self.finished_with_success == True:
                    return set_status('SUCCESS')
                    self.running = False
                else:
                    return set_status('FAILURE')
                    self.running = False

    def make_service_call(self,request,*args):
        rospy.wait_for_service('instructor_plugins/test_service')
        try:
            test_service = rospy.ServiceProxy('instructor_plugins/test_service', AddTwoInts)
            self.result = test_service(request)
            self.finished_with_success = True
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False

'''
