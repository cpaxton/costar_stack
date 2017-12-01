
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

colors = ColorOptions().colors

class ServiceNode(Node):

    def __init__(self, name, label, color,service_description, display_name = None):
        super(ServiceNode,self).__init__(name,label,color)
        self.ready_color = color
        self.service_description = service_description
        self.display_name = display_name
        self.service_thread = None
        self.reset_self()

    def get_node_type(self):
        return 'SERVICE'

    def get_node_name(self):
        return 'Service'

    def reset_self(self):
        if self.service_thread is not None and self.service_thread.is_alive():
            rospy.logwarn('joining' + str(self) + self.name_)
            #self.service_thread.join()
            #rospy.logwarn(' done joining' + str(self) + self.name_)
        self.service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
        self.set_color(self.ready_color)
        self.status_msg = ''

    def execute(self):
        if self.display_name is not None:
            running_service = '%s [%s]'%(self.service_description,self.display_name)
        else:
            running_service = '%s [%s]'%(self.service_description,self.name_)

        if self.needs_reset:
            rospy.loginfo('%s already [%s], needs reset'%(running_service,self.get_status()))
            return self.get_status()
        else:
            rospy.logwarn("running? " + str(self.running) + "; " + str(self.finished_with_success))
            if not self.running: # Thread is not running
                if self.finished_with_success is None: # Service was never called
                    try:
                        self.service_thread.start()
                        rospy.loginfo('%s running'%running_service)
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        self.status_msg = '%s thread failed'%running_service
                        rospy.logwarn(self.status_msg)
                        self.running = False
                        self.needs_reset = True
                        rospy.logerr('failed -- %s'%self.status_msg)
                        return self.set_status('FAILURE -- %s'%self.status_msg)
                        
            else:# If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('%s succeeded'%running_service)
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('SUCCESS')
                    else:
                        self.status_msg = '%s failed'%running_service
                        rospy.logwarn(self.status_msg)
                        self.running = False
                        self.needs_reset = True
                        rospy.logerr('failed -- %s'%self.status_msg)
                        return self.set_status('FAILURE -- %s'%self.status_msg)

    #def make_service_call(self,request,*args):
    #    raise NotImplementedError('make_service_call must be implemented in child class')

