#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy

from PyQt4 import QtGui, QtCore
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
from beetree import *
import sys
from std_msgs.msg import *

class BTWidget(QWidget):
  def __init__(self,app):
    super(BTWidget,self).__init__()
    self.app_ = app
    self.setMinimumWidth(1080)
    self.setMinimumHeight(540)
    
    # Setup layout
    self.layout_ = QGridLayout()
    self.setLayout(self.layout_)
    self.dot_widget1 = DotWidget()
    self.dot_widget2 = DotWidget()
    self.dot_widget3 = DotWidget()
    self.dot_widget4 = DotWidget()
    self.layout_.addWidget(self.dot_widget1,0,0)
    self.layout_.addWidget(self.dot_widget2,0,1)
    self.layout_.addWidget(self.dot_widget3,1,0)
    self.layout_.addWidget(self.dot_widget4,1,1)
    self.setStyleSheet('background-color:#ffffff')
    self.show()

    # Set up ros_ok watchdog timer to handle termination and ctrl-c
    self.ok_timer_ = QTimer(self)
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(1000)

    ### TREE -------------------------------------------------------------------

    self.root = NodeRoot('root','start')
    para_look_and_grab = NodeParallelAll('para_look_and_grab','detect_obj_move_to_bin')
    act_detect_object = NodeAction('act_detect_object','detect_obj')
    sec_pick_move_to_bin = NodeSequence('sec_pick_move_to_bin','pick_move_to_bin')
    cond_found_obj = NodeCondition('cond_found_obj','object_found','test','test')
    sec_pick_up = NodeSequence('sec_pick_up','pick_up')
    act_move_to_obj = NodeAction('act_move_to_obj','move_to_object')
    act_grab = NodeAction('act_grab','grab')
    act_lift = NodeAction('act_lift','lift')
    act_move = NodeAction('act_move','move_above_bin')
    sec_place = NodeSequence('sec_place','place')
    srv_reset = NodeService('srv_reset','reset')
    act_move_to_bin = NodeAction('act_move_to_bin','move_to_bin')
    act_release = NodeAction('act_release','release')
    act_lift.set_flag(True)

    self.root.add_child(para_look_and_grab)
    para_look_and_grab.add_child(act_detect_object)
    para_look_and_grab.add_child(sec_pick_move_to_bin)
    sec_pick_move_to_bin.add_child(cond_found_obj)
    sec_pick_move_to_bin.add_child(sec_pick_up)
    sec_pick_up.add_child(act_move_to_obj)
    sec_pick_up.add_child(act_grab)
    sec_pick_up.add_child(act_lift)
    sec_pick_move_to_bin.add_child(act_move)
    sec_pick_move_to_bin.add_child(sec_place)
    sec_pick_move_to_bin.add_child(srv_reset)
    sec_place.add_child(act_move_to_bin)    
    sec_place.add_child(act_release)
    #---------------------------------------------------------------------------

    # self.pub_ = rospy.Publisher('/beetree/dot',String)
    # print self.root.generate_dot()

    # CODE FOR TESTING DELETE FUNCTIONALITY ####################################
    self.dot_widget1.set_dotcode(self.root.generate_dot())
    self.dot_widget1.zoom_to_fit()

    act_release.remove_self()
    act_grab.remove_self()

    self.dot_widget2.set_dotcode(self.root.generate_dot())
    self.dot_widget2.zoom_to_fit()

    # sec_pick_up.remove_all_children()
    act_release = NodeAction('act_release','release')
    sec_place.add_child(act_release)
    act_grab = NodeAction('act_grab','grab')
    sec_pick_up.add_child(act_grab)

    self.dot_widget3.set_dotcode(self.root.generate_dot())
    self.dot_widget3.zoom_to_fit()

    # sec_pick_move_to_bin.remove_child('sec_place')
    act_wait = NodeAction('act_wait','wait')
    act_stop = NodeAction('act_stop','stop')
    act_grab.add_sibling_after(act_wait)
    act_grab.add_sibling_before(act_stop)

    self.dot_widget4.set_dotcode(self.root.generate_dot())
    self.dot_widget4.zoom_to_fit()

    ############################################################################
    # self.dot_widget1.set_dotcode(self.root.generate_dot())
    # self.dot_widget1.zoom_to_fit()

    # act_stop = NodeAction('act_stop','stop')
    # sec_pick_up.add_child(act_stop)

    # self.dot_widget2.set_dotcode(self.root.generate_dot())
    # self.dot_widget2.zoom_to_fit()

    # act_lift.remove_self()

    # self.dot_widget3.set_dotcode(self.root.generate_dot())
    # self.dot_widget3.zoom_to_fit()

    # self.dot_widget4.set_dotcode(self.root.generate_dot())
    # self.dot_widget4.zoom_to_fit()


    # print 'Connecting'
    # self.connect(self.dot_widget,SIGNAL("clicked"), self.clicked)

  def closeEvent(self, event):
    # self.settings.setValue('size', self.size())
    # self.settings.setValue('pos', self.pos())
    event.accept()

  def clicked(self,event):
    print 'You clicked ' + event

  def clean_up(self):
    pass

  def check_ok(self):
    # self.pub_.publish(String(self.root.generate_dot()))

    if rospy.is_shutdown():
      self.clean_up()
      self.close()
      self.app_.exit()


# MAIN #########################################################################
if __name__ == '__main__':
  rospy.init_node('beetree',anonymous=True)
  app = QApplication(sys.argv)
  wrapper = BTWidget(app)
  # Running
  app.exec_()
  # Done

        


















