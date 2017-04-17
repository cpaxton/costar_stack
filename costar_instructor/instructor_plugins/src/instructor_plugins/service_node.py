
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import *
from threading import Thread

# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor

import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, ColorOptions
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c

# Driver services for ur5
import costar_robot_msgs
from costar_robot_msgs.srv import *

class ServiceNode(Node):

    def __init__(self, name, label, color,service_description):
        super(ServiceNode,self).__init__(name,label,color)
        self.status_msg = ''
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
        self.service_description = service_description

    def get_node_type(self):
        return 'SERVICE'

    def get_node_name(self):
        return 'Service'

    def reset_self(self):
        self.service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
        self.set_color(self.color)

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Waypoint Service [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.service_thread.start()
                        rospy.loginfo('%s [' + self.name_ + '] running'%self.service_description)
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        self.status_msg = '%s [' + self.name_ + '] thread failed'%self.service_description
                        rospy.logwarn(self.status_msg)
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE -- %s'%self.status_msg)
                        
            else:# If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('%s [' + self.name_ + '] succeeded'%self.service_description)
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('SUCCESS')
                    else:
                        self.status_msg = '%s [' + self.name_ + '] failed'%self.service_description
                        rospy.logwarn(self.status_msg)
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE -- %s'%self.status_msg)

    def make_service_call(self,request,*args):
        raise NotImplementedError('make_service_call must be implemented in child class')

