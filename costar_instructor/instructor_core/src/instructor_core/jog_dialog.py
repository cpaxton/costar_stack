import rospy
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
import rospkg
from beetree import *



class JogDialog(QWidget):
    def __init__(self, show_hide_fn,parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        # GUI
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/jog_buttons.ui'
        uic.loadUi(w_path, self)
        # Vars
        self.jog_publisher = rospy.Publisher('/ur_robot/jog',JointState)
        self.show_hide = show_hide_fn 
        self.saved_geom = None
        self.jogging = False
        self.jog_direction = None
        self.step = .01
        self.rot_step = .05
        self.speed = 'low'
        #Signals
        self.btn_ok.clicked.connect(self.ok)

        self.btn_roll_left.pressed.connect(self.start_jog)
        self.btn_roll_right.pressed.connect(self.start_jog)
        self.btn_yaw_left.pressed.connect(self.start_jog)
        self.btn_yaw_right.pressed.connect(self.start_jog)
        self.btn_pitch_up.pressed.connect(self.start_jog)
        self.btn_pitch_down.pressed.connect(self.start_jog)
        self.btn_roll_left.released.connect(self.stop_jog)
        self.btn_roll_right.released.connect(self.stop_jog)
        self.btn_yaw_left.released.connect(self.stop_jog)
        self.btn_yaw_right.released.connect(self.stop_jog)
        self.btn_pitch_up.released.connect(self.stop_jog)
        self.btn_pitch_down.released.connect(self.stop_jog)

        self.btn_plus_x.pressed.connect(self.start_jog)
        self.btn_plus_y.pressed.connect(self.start_jog)
        self.btn_plus_z.pressed.connect(self.start_jog)
        self.btn_minus_x.pressed.connect(self.start_jog)
        self.btn_minus_y.pressed.connect(self.start_jog)
        self.btn_minus_z.pressed.connect(self.start_jog)
        self.btn_plus_x.released.connect(self.stop_jog)
        self.btn_plus_y.released.connect(self.stop_jog)
        self.btn_plus_z.released.connect(self.stop_jog)
        self.btn_minus_x.released.connect(self.stop_jog)
        self.btn_minus_y.released.connect(self.stop_jog)
        self.btn_minus_z.released.connect(self.stop_jog)
        self.btn_speed.clicked.connect(self.toggle_speed)

    def ok(self):
        self.stop_jog()
        self.show_hide()

    def toggle_speed(self):
        if self.speed == 'low':
            self.speed = 'high'
            self.step = .05
            self.rot_step = .1
            self.btn_speed.setText('JOG SPEED: HIGH')
            self.btn_speed.setStyleSheet('''QPushButton#btn_speed { background-color:#F22613;color:#ffffff;border:none }QPushButton#btn_speed:hover:!pressed{ background-color:#F22613;color:#ffffff;border:none }QPushButton#btn_speed:hover { background-color:#F22613;color:#ffffff;border:none }''')
        else:
            self.speed = 'low'
            self.step = .01
            self.rot_step = .05
            self.btn_speed.setText('JOG SPEED: LOW')
            self.btn_speed.setStyleSheet('''QPushButton#btn_speed { background-color:#26A65B;color:#ffffff;border:none }QPushButton#btn_speed:hover:!pressed{ background-color:#26A65B;color:#ffffff;border:none }QPushButton#btn_speed:hover { background-color:#26A65B;color:#ffffff;border:none }''')

    def stop_jog(self):
        sender = str(self.sender().objectName())
        P = JointState()
        P.header.stamp = rospy.get_rostime()
        P.header.frame_id = "jog_offset"
        P.position = [0]*6
        P.velocity = [0]*6
        P.effort = [0]*6
        self.jog_publisher.publish(P)
        rospy.logwarn('Sent '+ str(P.velocity))
        rospy.logwarn('Jog Finished ['+sender+']')
        self.jogging = False

    def start_jog(self):
        if not self.jogging:
            sender = str(self.sender().objectName())

            P = JointState()
            P.header.stamp = rospy.get_rostime()
            P.header.frame_id = "jog_offset"
            P.position = [0]*6
            P.velocity = [0]*6
            P.effort = [0]*6
            if sender == str("btn_plus_x"):
                P.velocity[0] += self.step
            elif sender == str("btn_plus_y"):
                P.velocity[1] += self.step
            elif sender == str("btn_plus_z"):
                P.velocity[2] += self.step
            elif sender == str("btn_minus_x"):
                P.velocity[0] -= self.step
            elif sender == str("btn_minus_y"):
                P.velocity[1] -= self.step
            elif sender == str("btn_minus_z"):
                P.velocity[2] -= self.step
                    
            elif sender == str("btn_pitch_up"):
                P.velocity[4] -= self.rot_step
            elif sender == str("btn_pitch_down"):
                P.velocity[4] += self.rot_step

            elif sender == str("btn_roll_left"):
                P.velocity[3] -= self.rot_step
            elif sender == str("btn_roll_right"):
                P.velocity[3] += self.rot_step
                
            elif sender == str("btn_yaw_left"):
                P.velocity[5] -= self.rot_step
            elif sender == str("btn_yaw_right"):
                P.velocity[5] += self.rot_step
            

            self.jog_publisher.publish(P)
            rospy.logwarn('Jog Started ['+sender+']')
            rospy.logwarn('Sent '+ str(P.velocity))
            self.jogging = True
 
