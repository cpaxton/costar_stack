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
import beetree; from beetree import Node, NodeAction
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, NoteField, ColorOptions
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c
# Driver services for ur5
#from robotiq_c_model_control.srv import *
from std_srvs.srv import Empty

colors = ColorOptions().colors
# Node Wrappers -----------------------------------------------------------
class NodeActionSGripperOpenGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSGripperOpenGUI,self).__init__(color='green')
        self.title.setText('GRIPPER OPEN ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionSGripper(self.get_name(),self.get_label(),'open',int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

class NodeActionSGripperCloseGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSGripperCloseGUI,self).__init__(color='green')
        self.title.setText('GRIPPER CLOSE ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionSGripper(self.get_name(),self.get_label(),'close',int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

class NodeActionSGripperBasicModeGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSGripperBasicModeGUI,self).__init__(color='green')
        self.title.setText('GRIPPER CLOSE ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionSGripper(self.get_name(),self.get_label(),'basic_mode',int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

class NodeActionSGripperPinchModeGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSGripperPinchModeGUI,self).__init__(color='green')
        self.title.setText('GRIPPER CLOSE ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionSGripper(self.get_name(),self.get_label(),'pinch_mode',int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

class NodeActionSGripperWideModeGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSGripperWideModeGUI,self).__init__(color='green')
        self.title.setText('GRIPPER CLOSE ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionSGripper(self.get_name(),self.get_label(),'wide_mode',int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

class NodeActionSGripperScissorModeGUI(NodeGUI):
    def __init__(self):
        super(NodeActionSGripperScissorModeGUI,self).__init__(color='green')
        self.title.setText('GRIPPER CLOSE ACTION')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionSGripper(self.get_name(),self.get_label(),'scissor_mode',int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeActionSGripper(Node):
    def __init__(self,name,label,gripper_command,wait_finish):
        super(NodeActionSGripper,self).__init__(name,label)
        self.gripper_command = gripper_command
        if self.gripper_command == 'open':
            self.type = 'OPEN S GRIPPER'
        elif self.gripper_command == 'close':
            self.type = 'CLOSE S GRIPPER'
        elif self.gripper_command == 'basic_mode':
            self.type = 'SET BASIC GRASP'
        elif self.gripper_command == 'scissor_mode':
            self.type = 'SET SCISSOR GRASP'
        elif self.gripper_command == 'pinch_mode':
            self.type = 'SET PINCH GRASP'
        elif self.gripper_command == 'wide_mode':
            self.type = 'SET WIDE GRASP'
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True
        # L = 'Action\\n'+label+'\\n WAIT ['+str(self.wait_finish)+']'
        L = self.type
        super(NodeActionSGripper,self).__init__(name,L,'#26A65B')
        # Reset params
        self.gripper_service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Gripper '+self.type+' [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.gripper_service_thread.start()
                        rospy.loginfo('Gripper '+self.type+' [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('Gripper '+self.type+' [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.gripper_service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('Gripper '+self.type+' [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('Gripper '+self.type+' [' + self.name_ + '] failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')

    def reset_self(self):
        self.gripper_service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
        self.set_color('#26A65B')

    def make_service_call(self,request,*args):
        # Check to see if service exists
        service_name = 'costar/gripper/%s'%(self.gripper_command)
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find gripper service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            gripper_open_proxy = rospy.ServiceProxy(service_name,Empty)
            # Send Open Command
            rospy.logwarn('Gripper '+self.type+' Started')
            # msg.state = self.open_gripper
            # msg.wait = self.wait_finish
            result = gripper_open_proxy()
            #if 'FAILURE' in str(result.ack):
            #    rospy.logwarn('Gripper '+self.type+' failed with reply: '+ str(result.ack))
            #    self.finished_with_success = False
            #    return
            #else:
            #    rospy.logwarn('Gripper '+self.type+' Finished')
            self.finished_with_success = True
            return

        except (rospy.ServiceException), e:
            rospy.logwarn('There was a problem with the service:')
            rospy.logwarn(e)
            self.finished_with_success = False
            return
