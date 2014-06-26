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
        super(NodeConditionTestPredicateGUI,self).__init__()
        self.predicate = NamedField('Predicate','')
        self.param1 = NamedField('Parameter 1','')
        self.param2 = NamedField('Parameter 2','')
        self.param3 = NamedField('Parameter 3','')
        self.layout_.addWidget(self.predicate)
        self.layout_.addWidget(self.param1)
        self.layout_.addWidget(self.param2)
        self.layout_.addWidget(self.param3)

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full(),self.predicate.full()]):
            return NodeConditionTestPredicate(parent,self.get_name(),self.get_label(),self.predicate.get(),
                    self.param1.get(),
                    self.param2.get(),
                    self.param3.get())
        else:
            return 'ERROR: node not properly defined'


# Nodes -------------------------------------------------------------------
class NodeConditionTestPredicate(Node):
    def __init__(self,parent,name,label,predicate_name=None,param1=None,param2=None,param3=None):
        L = '( condition )\\n' + label.upper()
        color = '#FAE364'
        super(NodeConditionTestPredicate,self).__init__(False,parent,name,L,color,'ellipse')
        self.predicate_ = predicate_name
        self.param1_ = param1
        self.param2_ = param2
        self.param3_ = param3
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'
    def execute(self):
        print 'Executing Condition: (' + self.name_ + ')'
        # Check for value on parameter server
        '''
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
        '''
        return self.node_status_
