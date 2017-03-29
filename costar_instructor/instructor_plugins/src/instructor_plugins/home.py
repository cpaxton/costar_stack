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
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, ColorOptions
import rospkg
from instructor_core.srv import *
import tf; 
import tf_conversions as tf_c
# Driver services for ur5
#import simple_ur_msgs
#from simple_ur_msgs.srv import *
import costar_robot_msgs
from costar_robot_msgs.srv import *

colors = ColorOptions().colors

# Node Wrappers -----------------------------------------------------------
class NodeHomeGUI(NodeGUI):
    def __init__(self,plan=False):
        super(NodeHomeGUI,self).__init__('green')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/home.ui'

        if plan:
            title = "PLAN"
        else:
            title = "MOVE"
        self.title.setText('%s TO HOME ACTION'%title)
        self.title.setStyleSheet('background-color:'+colors['green'].normal+';color:#ffffff')
        self.setStyleSheet('background-color:'+colors['green'].normal+' ; color:#ffffff')


        self.plan = plan
        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.layout_.addWidget(self.waypoint_ui)

        self.command_waypoint_name = None
        self.command_vel = .5
        self.command_acc = .5
        self.listener_ = tf.TransformListener()

        self.waypoint_ui.acc_slider.valueChanged.connect(self.acc_changed)
        self.waypoint_ui.vel_slider.valueChanged.connect(self.vel_changed)

    def vel_changed(self,t):
        self.waypoint_ui.vel_field.setText(str(float(t)))
        self.command_vel = float(t)/100

    def acc_changed(self,t):
        self.waypoint_ui.acc_field.setText(str(float(t)))
        self.command_acc = float(t)/100

    def save_data(self,data):
        data['vel'] = {'value':self.command_vel}
        data['acc'] = {'value':self.command_acc}
        return data

    def load_data(self,data):
        if data.has_key('vel'):
            if data['vel']['value']!=None:
                self.command_vel = data['vel']['value']
                self.waypoint_ui.vel_field.setText(str(float(self.command_vel)*100))
                self.waypoint_ui.vel_slider.setSliderPosition(int(float(self.command_vel)*100))
        if data.has_key('acc'):
            if data['acc']['value']!=None:
                self.command_acc = data['acc']['value']
                self.waypoint_ui.acc_field.setText(str(float(self.command_acc)*100))
                self.waypoint_ui.acc_slider.setSliderPosition(int(float(self.command_acc)*100))

    def generate(self):
        if all([self.name.full()]):
            # rospy.logwarn('Generating Move with acc='+str(self.command_acc)+' and vel='+str(self.command_vel))
            return NodeHome(self.get_name(),self.get_label(),self.command_vel,self.command_acc,self.plan)
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

# Nodes -------------------------------------------------------------------
class NodeHome(Node):
    def __init__(self,name,label,vel,acc,plan):
        if plan:
            L = 'PLAN TO HOME'
        else:
            L = 'MOVE TO HOME'
        super(NodeHome,self).__init__(name,L,'#26A65B')
        self.command_vel = vel
        self.command_acc = acc
        # Reset params
        self.service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False

        if plan:
            self.srv_name = 'Plan'
        else:
            self.srv_name = 'Servo'

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Waypoint Service [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.service_thread.start()
                        rospy.loginfo('Waypoint Service [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('Waypoint Service [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('Waypoint Service [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('Waypoint Service [' + self.name_ + '] failed')
                        self.running = False
                        self.needs_reset = True
                        self.set_color(colors['gray'].normal)
                        return self.set_status('FAILURE')

    def reset_self(self):
        self.service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False
        self.set_color('#26A65B')

    def make_service_call(self,request,*args):
        # Check to see if service exists
        try:
            rospy.wait_for_service('costar/%sToHome'%self.srv_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find servo service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            pose_servo_proxy = rospy.ServiceProxy('costar/%sToHome'%self.srv_name,ServoToPose)
            
            msg = costar_robot_msgs.srv.ServoToPoseRequest()
            msg.vel = self.command_vel
            msg.accel = self.command_acc
            # Send Servo Command
            rospy.logwarn('Single Servo Move Started')
            result = pose_servo_proxy(msg)
            if 'FAILURE' in str(result.ack):
                rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.logwarn('Single Servo Move Finished')
                rospy.logwarn('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, rospy.ServiceException), e:
            rospy.logwarn('There was a problem with the tf lookup or service:')
            rospy.logwarn(e)
            self.finished_with_success = False
            return

class PlanToHomeGUI(NodeHomeGUI):
    def __init__(self):
        super(PlanToHomeGUI, self).__init__(plan=True)

class MoveToHomeGUI(NodeHomeGUI):
    def __init__(self):
        super(MoveToHomeGUI, self).__init__(plan=False)
