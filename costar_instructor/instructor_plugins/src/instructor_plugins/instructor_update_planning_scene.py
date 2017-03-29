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
class NodeActionUpdatePlanningSceneGUI(NodeGUI):
    def __init__(self):
        super(NodeActionUpdatePlanningSceneGUI,self).__init__(color='purple')
        self.title.setText('UPDATE PLANNING SCENE')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
        self.wait_finish = NamedField('Wait', '','purple')
        self.wait_finish.set_field('1')
        self.note = NoteField('(1 = true, 0 = false)','purple')
        self.layout_.addWidget(self.wait_finish)
        self.layout_.addWidget(self.note)

    def generate(self):
        if all([self.name.full(),self.wait_finish.full()]):
            return NodeActionUpdatePlanningScene(self.get_name(),self.get_label(),int(self.wait_finish.get()))
        else:
            rospy.logerr('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeActionUpdatePlanningScene(Node):
    def __init__(self,name,label,wait_finish):
        color = colors['purple'].normal
        L = 'UPDATE PLANNING SCENE'
        if wait_finish == 0:
            self.wait_finish = False
        else:
            self.wait_finish = True
        # L = 'Action\\n'+label+'\\n WAIT ['+str(self.wait_finish)+']'
        super(NodeActionUpdatePlanningScene,self).__init__(name,L,color)
        # Reset params
        self.planning_scene_service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('MoveIt Planning Scene Server '+self.get_node_type()+' [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.planning_scene_service_thread.start()
                        rospy.loginfo('MoveIt Planning Scene Server '+self.get_node_type()+' [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('MoveIt Planning Scene Server '+self.get_node_type()+' [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.planning_scene_service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('MoveIt Planning Scene Server '+self.get_node_type()+' [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('MoveIt Planning Scene Server '+self.get_node_type()+' [' + self.name_ + '] failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')

    def reset_self(self):
        self.planning_scene_service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
        color = colors['purple'].normal
        self.set_color(color)

    def make_service_call(self,request,*args):
        # Check to see if service exists
        service_name = '/planningSceneGenerator/planningSceneGenerator'
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find planning scene generation service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            planning_scene_open_proxy = rospy.ServiceProxy(service_name,Empty)
            # Send Open Command
            rospy.loginfo('MoveIt Planning Scene Server '+self.get_node_type()+' Started')
            result = planning_scene_open_proxy()
            self.finished_with_success = True
            return

        except (rospy.ServiceException), e:
            rospy.logerr('There was a problem with the service:')
            rospy.logerr(e)
            self.finished_with_success = False
            return
