#!/usr/bin/env python
import roslib; roslib.load_manifest('predicator_core')
import rospy 
from std_msgs.msg import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node, NodeCondition
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField
# Thread for service calls
from threading import Thread

from predicator_msgs.srv import *

# Node Wrappers -----------------------------------------------------------
class NodeConditionTestPredicateGUI(NodeGUI):
    def __init__(self):
        super(NodeConditionTestPredicateGUI,self).__init__()
        self.title.setText('PREDICATE CONDITION')
        self.title.setStyleSheet('background-color:#FADA07;color:#ffffff')
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
            return NodeConditionTestPredicate(self.get_name(),self.get_label(),self.predicate.get(),
                    self.param1.get(),
                    self.param2.get(),
                    self.param3.get())
        else:
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeConditionTestPredicate(NodeCondition):
    def __init__(self,name,label,predicate_name=None,param1=None,param2=None,param3=None):
        L = name.upper()+'\\n[\\"'+label.upper()+'\\"]'#+'\\n['+self.type+']'
        super(NodeConditionTestPredicate,self).__init__(name,L)
        self.predicate_ = predicate_name
        self.param1_ = param1
        self.param2_ = param2
        self.param3_ = param3
        # Reset params
        self.service_thread = Thread(target=self.make_service_call)
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'

    def execute(self):
        rospy.logwarn('TEST PREDICATE ['+self.name_+']: EXECUTING')
        if self.needs_reset:
            rospy.loginfo('Sleep Service [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                try:
                    self.service_thread.start()
                    rospy.logwarn('TEST PREDICATE CONDITION ['+self.name_+']: STARTED')
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception,  errtxt:
                    rospy.logwarn('TEST PREDICATE CONDITION ['+self.name_+']: FAILED TO START THREAD')
                    self.running = False
                    self.needs_reset = True
                    return self.set_status('FAILURE')
            else:
                # If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    self.running = False
                    self.needs_reset = True
                    if self.finished_with_success == True:
                        rospy.logwarn('TEST PREDICATE CONDITION ['+self.name_+']: FINISHED with SUCCESS')
                        return self.set_status('SUCCESS')
                    else:
                        rospy.logwarn('TEST PREDICATE CONDITION ['+self.name_+']: FINISHED with FAILURE')
                        return self.set_status('FAILURE')
                    

    def make_service_call(self,*args):
        req = TestPredicateRequest()
        req.statement.predicate = self.predicate_
        req.statement.params[0] = self.param1_
        req.statement.params[1] = self.param2_
        req.statement.params[2] = self.param3_
        try:
            rospy.wait_for_service('/predicator/test_predicate',2)
        except:
            rospy.logerr('TEST PREDICATE CONDITION ['+self.name_+']: Could not find service')
            self.finished_with_success = False
        try:
            rospy.logwarn('TEST PREDICATE CONDITION ['+self.name_+']: TESTING')
            test_service = rospy.ServiceProxy('predicator/test_predicate', TestPredicate)
            self.result = test_service(req)
            self.finished_with_success = self.result.found
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False

    def reset_self(self):
        self.service_thread = Thread(target=self.make_service_call)
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False



