#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy, rospkg 
from std_msgs.msg import *
from beetree import *
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from instructor_core.instructor_qt import NamedField, NamedComboBox
from instructor_core.instructor_qt import NamedField, NoteField, ColorOptions

from instructor_core import NodeGUI
from service_node import ServiceNode
colors = ColorOptions().colors

import inspect

class NodeDecoratorRepeat(Node):
    ''' Decorator Repeat Node
    Executes child node N times. If N = -1, will repeat forever, ignoring failures
    '''
    def __init__(self,name,label):
        L = 'Repeat \n Until'
        
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        
        super(NodeDecoratorRepeat,self).__init__(name,L,color,shape='invhouse',alt_label=L_alt,size=12)
        self.runs_ = 0
        self.num_runs_ = 0
    def get_node_type(self):
        return 'DECORATOR_While'
    def get_node_name(self):
        return 'Decorator While'
    def execute(self):
        if self.runs_ == -1: # run forever
            self.child_status_ = self.children_[0].execute()
            if self.child_status_[:7] == 'SUCCESS':
                rospy.logwarn('REPEAT DECORATOR ['+self.name_+']: SUCCEEDED, RESET')
                self.children_[0].reset()
                return self.set_status('RUNNING')
            elif self.child_status_[:7] == 'RUNNING':
                return self.set_status('RUNNING')
            elif self.child_status_[:7] == 'FAILURE':
                return self.set_status('FAILURE')

    def reset(self):
        super(NodeDecoratorRepeat, self).reset()
        if not self.runs_ == -1:
            self.num_runs_ = 0


class NodeLearningSequence(Node):
    """ Learning Sequence Node
    The sequence node executes its children in order or insertion.  If a child
    fails, the node will return FAILURE. If a child succeeds, the sequence will
    then execute the next child until all children are executed, then return
    SUCCESS.
    """
    def __init__(self,name,label):
        L = 'Learn ->'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#6897bb'
        super(NodeLearningSequence,self).__init__(name,L,color,alt_label=L_alt,shape='diamond')
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):

        if len(self.children_) == 0:
            rospy.logwarn("sequence ["+self.name_+"] has no children, returning success.")
            return self.set_status('SUCCESS')

        for child in self.children_:
            status = child.execute()

            if status is None:
                return self.set_status('FAILURE')
            elif status[:7] != 'SUCCESS':
                return self.set_status(status)

        return self.set_status('SUCCESS')

class NodeLearnedSequence(Node):
    """ Learning Sequence Node
    The sequence node executes its children in order or insertion.  If a child
    fails, the node will return FAILURE. If a child succeeds, the sequence will
    then execute the next child until all children are executed, then return
    SUCCESS.
    """
    def __init__(self,name,label):
        L = 'Learned ' + label
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#FF7373'
        super(NodeLearnedSequence,self).__init__(name,L,color,alt_label=L_alt,shape='diamond')
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):

        if len(self.children_) == 0:
            rospy.logwarn("sequence ["+self.name_+"] has no children, returning success.")
            return self.set_status('SUCCESS')

        for child in self.children_:
            status = child.execute()

            if status is None:
                return self.set_status('FAILURE')
            elif status[:7] != 'SUCCESS':
                return self.set_status(status)

        return self.set_status('SUCCESS')

class NodeRepeatUntilGUI(NodeGUI):
    def __init__(self):
        super(NodeRepeatUntilGUI,self).__init__()
        self.title.setText('Repeat Until LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def save_data(self,data):
        return data

    def load_data(self,data):
        # no data to load
        pass
        
    def generate(self):
        if all([self.name.full()]):
            return NodeDecoratorRepeat(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'


class NodeLearningSequenceGUI(NodeGUI):
    def __init__(self):
        super(NodeLearningSequenceGUI,self).__init__()
        self.title.setText('Learning Sequence LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def save_data(self,data):
        return data

    def load_data(self,data):
        # no data to load
        pass
        
    def generate(self):
        if all([self.name.full()]):
            return NodeLearningSequence(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeLearnedSequenceGUI(NodeGUI):
    def __init__(self):
        super(NodeLearnedSequenceGUI,self).__init__()
        self.title.setText('Learned Sequence LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def save_data(self,data):
        return data

    def load_data(self,data):
        # no data to load
        pass
        
    def generate(self):
        if all([self.name.full()]):
            return NodeLearnedSequence(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'


class NodeDummyConditionalGUI(NodeGUI):
    def __init__(self):
        super(NodeDummyConditionalGUI,self).__init__(color='purple')
        self.title.setText('Dummy Conditional:')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')

    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full()]):
            return NodeDummyConditional(self.get_name(),self.get_label())
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeDummyRobotActionGUI(NodeGUI):
    def __init__(self):
        super(NodeDummyRobotActionGUI,self).__init__(color='sea_green')
        self.title.setText('Dummy Conditional:')
        self.title.setStyleSheet('background-color:'+colors['sea_green'].normal+';color:#ffffff')
    
    def save_data(self,data):
        return data
    def load_data(self,data):
        pass
    def generate(self):
        if all([self.name.full()]):
            return NodeDummyRobotAction(self.get_name(),self.get_label())
        else:
            rospy.logerr('check that all menu items are properly selected for this node')
            return 'ERROR: check that all menu items are properly selected for this node'


class NodeDummyConditional(ServiceNode):
    def __init__(self,name,label):
        L = label
        super(NodeDummyConditional,self).__init__(name,L,colors['purple'].normal,"Dummy Condition")

    def make_service_call(self,request,*args):
        self.finished_with_success = True
        return

class NodeDummyRobotAction(ServiceNode):
    def __init__(self,name,label):
        L = label
        super(NodeDummyRobotAction,self).__init__(name,L,colors['orange'].normal,"Dummy Action")

    def make_service_call(self,request,*args):
        # Check to see if service exists
        self.finished_with_success = True
        return


