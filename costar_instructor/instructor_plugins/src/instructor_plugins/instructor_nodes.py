#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy
import rospkg
from std_msgs.msg import *
from threading import Thread
# Qt 
from PyQt4 import QtGui,  QtCore,  uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, ColorOptions
# Predicator
from predicator_msgs.msg import *
from predicator_msgs import srv

colors = ColorOptions().colors

# Node Wrappers -----------------------------------------------------------
class NodeParamConditionGUI(NodeGUI):
    def __init__(self):
        super(NodeParamConditionGUI, self).__init__('pink')
        self.title.setText('PARAMETER CONDITION')
        self.title.setStyleSheet('background-color:'+colors['pink'].normal+';color:#ffffff')
        self.param = NamedField('Parameter', '','pink')
        self.value = NamedField('Check Value', '','pink')
        self.layout_.addWidget(self.param)
        self.layout_.addWidget(self.value)

    def generate(self):
        if all([self.name.full(), self.label.full(), self.param.full(), self.value.full()]):
            return NodeParamCondition(self.get_name(), self.get_label(), self.param.get(), self.value.get())
        else:
            return 'ERROR: node not properly defined'


class NodeActionTestGUI(NodeGUI):
    def __init__(self):
        super(NodeActionTestGUI, self).__init__()
        self.wait = NamedField('Wait', '')
        self.layout_.addWidget(self.wait)
        self.simulate_success = NamedField('Simulate Success', 'True')
        self.layout_.addWidget(self.simulate_success)

    def generate(self):
        print self.wait.get()
        if all([self.name.full(), self.label.full(), self.wait.full(), self.simulate_success.full()]):
            return NodeActionTest(self.get_name(), self.get_label(), int(self.wait.get()),  self.simulate_success.get())
        else:
            return 'ERROR: node not properly defined'


class NodeActionSleepGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSleepGUI, self).__init__('green')
        self.title.setText('SLEEP ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait = NamedField('Sleep Time', '','green')
        self.layout_.addWidget(self.wait)

    def generate(self):
        if all([self.name.full(),  self.wait.full()]):
            return NodeActionSleep(self.get_name(),  self.get_label(),  float(self.wait.get()))
        else:
            return 'ERROR: sleep action node not properly defined'


# Nodes -------------------------------------------------------------------
class NodeParamCondition(Node):
    def __init__(self, name, label, param_name=None, desired_value=None):
        L = '( condition )\\n' + label.upper()
        color = '#E87E04'
        super(NodeParamCondition, self).__init__(name, L, color, 'ellipse')
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

class NodeActionTest(Node):
    def __init__(self, name, label, wait, simulate_success):
        L = '( action )\\n' + label.upper()
        super(NodeActionTest, self).__init__(name, L, '#26A65B')
        self.wait_ = wait
        self.wait_thread_ = Thread(target=self.sleep_fn,  args=(self.wait_, 1))
        self.simulate_success_ = simulate_success
        self.running_ = False
    def get_node_type(self):
        return 'ACTION'
    def get_node_name(self):
        return 'Action'
    def reset(self):
        print "Node " + self.name_ + " resetting."
        self.running_ = False
        self.wait_thread_ = Thread(target=self.sleep_fn,  args=(self.wait_, 1))
    def execute(self):
        print 'Executing Action: (' + self.name_ + ')'
        if not self.running_: # Thread is not running
            try:
                self.wait_thread_.start()
                self.running_ = True
                return self.set_status('RUNNING')
            except Exception,  errtxt:
                print 'could not create thread'
                self.running_ = False
                return self.set_status('FAILURE')
        else:
            if self.wait_thread_.is_alive():
                # If thread is running
                return self.set_status('RUNNING')
            else:
                # Finished
                # print self.simulate_success_
                if self.simulate_success_.upper() == 'TRUE':
                    # print '>>> action returned success'
                    return self.set_status('SUCCESS')
                else:
                    # print '>>> action returned failure'
                    return self.set_status('FAILURE')

    def sleep_fn(self, wait, *args):
        print 'action '+ self.name_+ ' is SLEEPING.'
        rospy.sleep(wait)
        print 'action '+ self.name_+ ' is DONE.'

class NodeActionSleep(Node):
    def __init__(self, name, label, wait_time):
        L = 'SLEEP FOR '+str(wait_time)+'s'
        super(NodeActionSleep, self).__init__(name, L, '#26A65B')
        self.wait_time = wait_time
        # Reset params
        self.sleep_thread = Thread(target=self.sleep,  args=(self.wait_time, 1))
        self.running = False
        self.needs_reset = False

    def get_node_type(self):
        return 'SLEEP_ACTION'

    def get_node_name(self):
        return 'Sleep_Action'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Sleep Service [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                try:
                    self.sleep_thread.start()
                    rospy.loginfo('SLEEP ACTION ['+self.name_+']: STARTED')
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception,  errtxt:
                    rospy.logwarn('SLEEP ACTION ['+self.name_+']: FAILED TO START THREAD')
                    self.running = False
                    self.needs_reset = True
                    self.set_color(colors['gray'].normal)
                    return self.set_status('FAILURE')
            else:
                # If thread is running
                if self.sleep_thread.is_alive():
                    rospy.logwarn('SLEEP ACTION ['+self.name_+']: SLEEPING')
                    return self.set_status('RUNNING')
                else:
                    rospy.logwarn('SLEEP ACTION ['+self.name_+']: FINISHED')
                    self.running = False
                    self.needs_reset = True
                    self.set_color(colors['gray'].normal)
                    return self.set_status('SUCCESS')

    def sleep(self, request, *args):
        rospy.logwarn('SLEEP ACTION ['+self.name_+']: sleeping ...')
        rospy.sleep(request)
        rospy.logwarn('SLEEP ACTION ['+self.name_+']: done ...')

    def reset_self(self):
        self.sleep_thread = Thread(target=self.sleep,  args=(self.wait_time, 1))
        self.running = False
        self.needs_reset = False
        self.set_color('#26A65B')
