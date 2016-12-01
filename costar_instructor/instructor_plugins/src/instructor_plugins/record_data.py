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
class NodeActionRecordDataGUI(NodeGUI):
    def __init__(self):
        super(NodeActionRecordDataGUI,self).__init__(color='green')
        self.title.setText('RECORD DATA')
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','green')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','green')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)
    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionRecordData(self.get_name(),self.get_label(),int(self.wait_finish.get()))
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeActionRecordData(Node):
    def __init__(self,name,label,wait_finish):
        super(NodeActionRecordData,self).__init__(name,label)
        self.type = "RECORD DATA"
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True
        # L = 'Action\\n'+label+'\\n WAIT ['+str(self.wait_finish)+']'
        L = self.type
        super(NodeActionRecordData,self).__init__(name,L,'#26A65B')
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
            rospy.loginfo('Recorder '+self.type+' [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.gripper_service_thread.start()
                        rospy.loginfo('SP Server '+self.type+' [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('SP Server '+self.type+' [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.gripper_service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('SP Server '+self.type+' [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('SP Server '+self.type+' [' + self.name_ + '] failed')
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
        service_name = '/record_camera'
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find object segmentation/detection service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            gripper_open_proxy = rospy.ServiceProxy(service_name,Empty)
            # Send Open Command
            rospy.logwarn('Recorder '+self.type+' Started')
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
