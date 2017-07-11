#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node, NodeQuery
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, NamedComboBox, ColorOptions
# For testing the service node
import instructor_plugins
from instructor_plugins.srv import *
from threading import Thread

colors = ColorOptions().colors

# Sample Node Wrappers -----------------------------------------------------------
class NodeActionSampleGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSampleGUI,self).__init__()

    def generate(self):
        if all([self.name.full(),self.label.full()]):
            return beetree.NodeAction(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeServiceSampleGUI(NodeGUI):
    def __init__(self):
        super(NodeServiceSampleGUI,self).__init__()

    def generate(self):
        if all([self.name.full(),self.label.full()]):
            return NodeServiceSample(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeQuerySampleGUI(NodeGUI):
    def __init__(self):
        super(NodeQuerySampleGUI,self).__init__()

    def generate(self):
        if all([self.name.full(),self.label.full()]):
            return NodeQuerySample(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeVariableSampleGUI(NodeGUI):
    def __init__(self):
        super(NodeVariableSampleGUI,self).__init__()

    def generate(self):
        if all([self.name.full(),self.label.full()]):
            return NodeVariableSample(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

# Sample Nodes -------------------------------------------------------------------
class NodeQuerySample(NodeQuery):
    def __init__(self,name,label):
        super(NodeQuerySample,self).__init__(name,label)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        # TODO
        self.node_status_ = 'SUCCESS'
        return self.node_status_

class NodeVariableSample(Node):
    def __init__(self,name,label):
        L = 'VARIABLE\\n[' + label + ']'
        L_alt = '{VARIABLE | ' + label.lower()+'}'
        color = '#BC83DE'
        super(NodeVariableSample,self).__init__(name,L,color,alt_label=L_alt,attach=False)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        # TODO
        self.node_status_ = 'SUCCESS'
        return self.node_status_

class NodeServiceSample(Node):
    def __init__(self,name,label):
        super(NodeServiceSample,self).__init__(name,label,'#26A65B')
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
                    rospy.loginfo('Test Service running')
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception, errtxt:
                    rospy.loginfo('Test Service thread failed')
                    self.running = False
                    return self.set_status('FAILURE')
        else:
            # If thread is running
            if self.service_thread.is_alive():
                return self.set_status('RUNNING')
            else:
                if self.finished_with_success == True:
                    rospy.loginfo('Test Service succeeded')
                    self.running = False
                    return self.set_status('SUCCESS')
                else:
                    rospy.loginfo('Test Service failed')
                    return self.set_status('FAILURE')
                    self.running = False

    def make_service_call(self,request,*args):
        #TODO check for timeout
        rospy.wait_for_service('instructor_plugins/TestService')
        try:
            test_service_proxy = rospy.ServiceProxy('instructor_plugins/TestService', instructor_plugins.srv.TestService)
            self.result = test_service_proxy(request)
            rospy.loginfo('Test Service finished successfully')
            self.finished_with_success = True
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False
