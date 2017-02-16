
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from instructor_qt import *

import rospy

import os,sys, inspect, ast
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
import rospkg
import tf; 
import tf_conversions as tf_c

# Using roslib.rospack even though it is deprecated
import threading
from roslib import rospack
import yaml
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import time
from copy import deepcopy

from costar_robot_msgs.srv import *

color_options = ColorOptions()
colors = color_options.colors

class RobotInterface():
    def __init__(self,status_label,teach_btn,servo_btn,sound_pub,toast):
        self.toast = toast
        self.driver_status_sub = rospy.Subscriber('/costar/DriverStatus',String,self.driver_status_cb)
        self.status_label = status_label
        self.teach_btn = teach_btn
        self.servo_btn = servo_btn
        self.sound_pub = sound_pub
        self.driver_status = 'NOT CONNECTED'
        self.status_label.setText('ROBOT MODE: [NOT CONNECTED]')

    def driver_status_cb(self,msg):
        mode = str(msg.data)
        self.driver_status = mode
        self.status_label.setText('ROBOT MODE: ['+mode.upper()+']')
        # self.update_status()

    def update_status(self):
        if self.driver_status == 'TEACH':
            self.status_label.setStyleSheet('background-color:'+colors['blue'].normal+'; color:#ffffff')
        elif self.driver_status == 'SERVO':
            self.status_label.setStyleSheet('background-color:'+colors['green'].normal+'; color:#ffffff')
        elif self.driver_status == 'IDLE':
            self.status_label.setStyleSheet('background-color:'+colors['gray_light'].normal+'; color:#ffffff')
        elif self.driver_status == 'IDLE - WARN':
            self.status_label.setStyleSheet('background-color:'+colors['orange'].normal+'; color:#ffffff')
        elif self.driver_status == 'NOT CONNECTED':
            self.status_label.setStyleSheet('background-color:'+colors['orange'].normal+'; color:#ffffff')
        elif self.driver_status == 'DISABLED':
            self.status_label.setStyleSheet('background-color:'+colors['red'].normal+'; color:#ffffff')

        # self.status_label.setText('ROBOT MODE: ['+self.driver_status.upper()+']')

    def teach(self):
        if self.driver_status == 'IDLE':
            try:
                rospy.wait_for_service('/costar/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find teach service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/costar/SetTeachMode',SetTeachMode)
                result = teach_mode_service(True)
                # self.sound_pub.publish(String("low_up"))
                rospy.logwarn(result.ack)
                self.teach_btn.set_color(colors['gray_light'])
            except rospy.ServiceException, e:
                print e
        elif self.driver_status == 'TEACH':
            try:
                rospy.wait_for_service('/costar/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find teach service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/costar/SetTeachMode',SetTeachMode)
                result = teach_mode_service(False)
                rospy.logwarn(result.ack)
                # self.sound_pub.publish(String("low_down"))
                self.teach_btn.set_color(colors['gray'])
            except rospy.ServiceException, e:
                print e
        else:
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')
            self.toast('Driver is in ['+self.driver_status+'] mode!')

    def servo(self):
        if self.driver_status == 'IDLE':
            try:
                rospy.wait_for_service('/costar/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
                result = servo_mode_service('SERVO')
                rospy.logwarn(result.ack)
                self.servo_btn.set_color(colors['gray_light'])
            except rospy.ServiceException, e:
                print e

        elif self.driver_status == 'SERVO':
            try:
                rospy.wait_for_service('/costar/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
                result = servo_mode_service('DISABLE')
                rospy.logwarn(result.ack)
                self.servo_btn.set_color(colors['gray'])
            except rospy.ServiceException, e:
                print e
        else:
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')
            self.toast('Driver is in ['+self.driver_status+'] mode!')

    def stop_servo(self):
        if self.driver_status == 'SERVO':
            try:
                rospy.wait_for_service('/costar/SetServoMode',2)
            except rospy.ROSException as e:
                print 'Could not find SetServoMode service'
                return
            try:
                servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
                result = servo_mode_service('DISABLE')
                rospy.logwarn(result.ack)
                self.servo_btn.set_color(colors['gray'])
            except rospy.ServiceException, e:
                print e

    def disable(self):
        try:
            rospy.wait_for_service('/costar/SetServoMode',2)
        except rospy.ROSException as e:
            print 'Could not find SetServoMode service'
            return
        try:
            servo_mode_service = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
            result = servo_mode_service('DISABLE')
            rospy.logwarn(result.ack)
        except rospy.ServiceException, e:
            print e

    def teach_disable(self):
        if self.driver_status == 'TEACH':
            try:
                rospy.wait_for_service('/costar/SetTeachMode',2)
            except rospy.ROSException as e:
                print 'Could not find teach service'
                return
            try:
                teach_mode_service = rospy.ServiceProxy('/costar/SetTeachMode',SetTeachMode)
                result = teach_mode_service(False)
                rospy.logwarn(result.ack)
                # self.teach = False
                self._widget.teach_enable_label.setText('DISABLED')
                self._widget.teach_enable_label.setStyleSheet('color:#ffffff;background-color:#FF9100')
                self._widget.msg_label.setText("teach DISABLED")
            except rospy.ServiceException, e:
                print e
        else:
            self._widget.msg_label.setText("DRIVER MUST BE IN TEACH MODE TO DISABLE")
            rospy.logwarn('FAILED, driver is in ['+self.driver_status+'] mode.')
            self.toast('Driver is in ['+self.driver_status+'] mode!')


