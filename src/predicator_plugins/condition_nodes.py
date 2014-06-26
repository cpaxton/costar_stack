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
from instructor import NodeGUI
from instructor.instructor_qt import NamedField

# Node Wrappers -----------------------------------------------------------
class NodeConditionTestPredicateGUI(NodeGUI):
    def __init__(self):
        super(NodeParamConditionGUI,self).__init__()
        self.param = NamedField('Parameter','')
        self.value = NamedField('Check Value','')
        self.layout_.addWidget(self.param)
        self.layout_.addWidget(self.value)

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full(),self.param.full(),self.value.full()]):
            return NodeConditionTestPredicate(parent,self.get_name(),self.get_label(),self.param.get(),self.value.get())
        else:
            return 'ERROR: node not properly defined'


# Nodes -------------------------------------------------------------------
class NodeConditionTestPredicate(Node):
    def __init__(self,parent,name,label,param_name=None,desired_value=None):
        L = '( condition )\\n' + label.upper()
        color = '#FAE364'
        super(NodeParamCondition,self).__init__(False,parent,name,L,color,'ellipse')
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'
    def execute(self):
        print 'Executing Condition: (' + self.name_ + ')'
        # Check for value on parameter server
        if not rospy.has_param(self.param_name_):
            self.node_status_ = 'FAILURE'
            print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
            return self.node_status_
        # Get value
        value = rospy.get_param(self.param_name_)
        # Check if value matches
        if value == self.desired_value_:
            self.node_status_ = 'SUCCESS'
            print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
            return self.node_status_
        else:
            self.node_status_ = 'FAILURE'
            print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
            return self.node_status_
