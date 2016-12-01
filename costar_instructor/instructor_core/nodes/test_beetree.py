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
    # self.layout_.addWidget(self.dot_widget2,0,1)
    # self.layout_.addWidget(self.dot_widget3,1,0)
    # self.layout_.addWidget(self.dot_widget4,1,1)
    self.setStyleSheet('background-color:#ffffff')
    self.show()

    # Set up ros_ok watchdog timer to handle termination and ctrl-c
    self.ok_timer_ = QTimer(self)
    self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
    self.ok_timer_.start(1000)

    self.root = NodeRoot('root','start')
    para_look_and_grab = NodeParallelAll('para_look_and_grab','detect_obj_move_to_bin')
    act_detect_object_cup = NodeAction('act_detect_object_c','DETECT_OBJ\\n[\\"CUP\\"]')
    act_detect_object_bin = NodeAction('act_detect_object_b','DETECT_OBJ\\n[\\"BIN\\"]')
    sec_find_objs = NodeSequence('sec_find_objs','find_objs')
    sec_pick_move_to_bin = NodeSequence('sec_pick_move_to_bin','pick_move_to_bin')
    cond_found_obj = NodeCondition('cond_found_obj','OBJECT_FOUND\\n[\\"CUP\\"]','test','test')
    sec_pick_up = NodeSequence('sec_pick_up','pick_up')
    query_obj = NodeQuery('query_obj','GET_POSE\\n[\\"CUP\\", CUP_POSE]')
    query_bin = NodeQuery('query_bin','GET_POSE\\n[\\"BIN\\", BIN_POSE]')
    act_move_to_obj = NodeAction('act_move_to_obj','MOVE_TO\\n[CUP_POSE]')
    act_grab = NodeAction('act_grab','grab')
    act_lift = NodeAction('act_lift','lift')
    act_move = NodeAction('act_move','move_above_bin')
    sec_place = NodeSequence('sec_place','place')
    act_move_to_bin = NodeAction('act_move_to_bin','MOVE_TO\\n[BIN_POSE]')    
    act_close_gripper = NodeAction('act_close_gripper','CLOSE\\nGRIPPER')
    act_open_gripper = NodeAction('act_open_gripper','OPEN\\nGRIPPER')
    act_wait = NodeAction('act_wait','WAIT\\n[5.0s]')
    sel_check = NodeSelector('sel_check','sel_check')
    sec_check = NodeSequence('sec_check','sec_check')
    var_cup = NodeVariable('var_cup','[CUP_POSE]',attach=False)
    var_bin = NodeVariable('var_bin','[BIN_POSE]',attach=False)

    _1_para_look_and_grab = NodeParallelAll('_1_para_look_and_grab','detect_obj_move_to_bin')
    _1_act_detect_object_cup = NodeAction('_1_act_detect_object_c','DETECT_OBJ\\n[\\"CUP\\"]')
    _1_act_detect_object_bin = NodeAction('_1_act_detect_object_b','DETECT_OBJ\\n[\\"BIN\\"]')
    _1_sec_find_objs = NodeSequence('_1_sec_find_objs','find_objs')
    _1_sec_pick_move_to_bin = NodeSequence('_1_sec_pick_move_to_bin','pick_move_to_bin')
    _1_cond_found_obj = NodeCondition('_1_cond_found_obj','OBJECT_FOUND\\n[\\"CUP\\"]','test','test')
    _1_sec_pick_up = NodeSequence('_1_sec_pick_up','pick_up')
    _1_query_obj = NodeQuery('_1_query_obj','GET_POSE\\n[\\"CUP\\", CUP_POSE]')
    _1_query_bin = NodeQuery('_1_query_bin','GET_POSE\\n[\\"BIN\\", BIN_POSE]')
    _1_act_move_to_obj = NodeAction('_1_act_move_to_obj','MOVE_TO\\n[CUP_POSE]')
    _1_act_grab = NodeAction('_1_act_grab','grab')
    _1_act_lift = NodeAction('_1_act_lift','lift')
    _1_act_move = NodeAction('_1_act_move','move_above_bin')
    _1_sec_place = NodeSequence('_1_sec_place','place')
    _1_act_move_to_bin = NodeAction('_1_act_move_to_bin','MOVE_TO\\n[BIN_POSE]')    
    _1_act_close_gripper = NodeAction('_1_act_close_gripper','CLOSE\\nGRIPPER')
    _1_act_open_gripper = NodeAction('_1_act_open_gripper','OPEN\\nGRIPPER')
    _1_act_wait = NodeAction('_1_act_wait','WAIT\\n[5.0s]')
    _1_sel_check = NodeSelector('_1_sel_check','sel_check')
    _1_sec_check = NodeSequence('_1_sec_check','sec_check')
    _1_var_cup = NodeVariable('_1_var_cup','[CUP_POSE]',attach=False)
    _1_var_bin = NodeVariable('_1_var_bin','[BIN_POSE]',attach=False)

    _2_para_look_and_grab = NodeParallelAll('_2_para_look_and_grab','detect_obj_move_to_bin')
    _2_act_detect_object_cup = NodeAction('_2_act_detect_object_c','DETECT_OBJ\\n[\\"CUP\\"]')
    _2_act_detect_object_bin = NodeAction('_2_act_detect_object_b','DETECT_OBJ\\n[\\"BIN\\"]')
    _2_sec_find_objs = NodeSequence('_2_sec_find_objs','find_objs')
    _2_sec_pick_move_to_bin = NodeSequence('_2_sec_pick_move_to_bin','pick_move_to_bin')
    _2_cond_found_obj = NodeCondition('_2_cond_found_obj','OBJECT_FOUND\\n[\\"CUP\\"]','test','test')
    _2_sec_pick_up = NodeSequence('_2_sec_pick_up','pick_up')
    _2_query_obj = NodeQuery('_2_query_obj','GET_POSE\\n[\\"CUP\\", CUP_POSE]')
    _2_query_bin = NodeQuery('_2_query_bin','GET_POSE\\n[\\"BIN\\", BIN_POSE]')
    _2_act_move_to_obj = NodeAction('_2_act_move_to_obj','MOVE_TO\\n[CUP_POSE]')
    _2_act_grab = NodeAction('_2_act_grab','grab')
    _2_act_lift = NodeAction('_2_act_lift','lift')
    _2_act_move = NodeAction('_2_act_move','move_above_bin')
    _2_sec_place = NodeSequence('_2_sec_place','place')
    _2_act_move_to_bin = NodeAction('_2_act_move_to_bin','MOVE_TO\\n[BIN_POSE]')    
    _2_act_close_gripper = NodeAction('_2_act_close_gripper','CLOSE\\nGRIPPER')
    _2_act_open_gripper = NodeAction('_2_act_open_gripper','OPEN\\nGRIPPER')
    _2_act_wait = NodeAction('_2_act_wait','WAIT\\n[5.0s]')
    _2_sel_check = NodeSelector('_2_sel_check','sel_check')
    _2_sec_check = NodeSequence('_2_sec_check','sec_check')
    _2_var_cup = NodeVariable('_2_var_cup','[CUP_POSE]',attach=False)
    _2_var_bin = NodeVariable('_2_var_bin','[BIN_POSE]',attach=False)

    _3_para_look_and_grab = NodeParallelAll('_3_para_look_and_grab','detect_obj_move_to_bin')
    _3_act_detect_object_cup = NodeAction('_3_act_detect_object_c','DETECT_OBJ\\n[\\"CUP\\"]')
    _3_act_detect_object_bin = NodeAction('_3_act_detect_object_b','DETECT_OBJ\\n[\\"BIN\\"]')
    _3_sec_find_objs = NodeSequence('_3_sec_find_objs','find_objs')
    _3_sec_pick_move_to_bin = NodeSequence('_3_sec_pick_move_to_bin','pick_move_to_bin')
    _3_cond_found_obj = NodeCondition('_3_cond_found_obj','OBJECT_FOUND\\n[\\"CUP\\"]','test','test')
    _3_sec_pick_up = NodeSequence('_3_sec_pick_up','pick_up')
    _3_query_obj = NodeQuery('_3_query_obj','GET_POSE\\n[\\"CUP\\", CUP_POSE]')
    _3_query_bin = NodeQuery('_3_query_bin','GET_POSE\\n[\\"BIN\\", BIN_POSE]')
    _3_act_move_to_obj = NodeAction('_3_act_move_to_obj','MOVE_TO\\n[CUP_POSE]')
    _3_act_grab = NodeAction('_3_act_grab','grab')
    _3_act_lift = NodeAction('_3_act_lift','lift')
    _3_act_move = NodeAction('_3_act_move','move_above_bin')
    _3_sec_place = NodeSequence('_3_sec_place','place')
    _3_act_move_to_bin = NodeAction('_3_act_move_to_bin','MOVE_TO\\n[BIN_POSE]')    
    _3_act_close_gripper = NodeAction('_3_act_close_gripper','CLOSE\\nGRIPPER')
    _3_act_open_gripper = NodeAction('_3_act_open_gripper','OPEN\\nGRIPPER')
    _3_act_wait = NodeAction('_3_act_wait','WAIT\\n[5.0s]')
    _3_sel_check = NodeSelector('_3_sel_check','sel_check')
    _3_sec_check = NodeSequence('_3_sec_check','sec_check')
    _3_var_cup = NodeVariable('_3_var_cup','[CUP_POSE]',attach=False)
    _3_var_bin = NodeVariable('_3_var_bin','[BIN_POSE]',attach=False)

    _4_para_look_and_grab = NodeParallelAll('_4_para_look_and_grab','detect_obj_move_to_bin')
    _4_act_detect_object_cup = NodeAction('_4_act_detect_object_c','DETECT_OBJ\\n[\\"CUP\\"]')
    _4_act_detect_object_bin = NodeAction('_4_act_detect_object_b','DETECT_OBJ\\n[\\"BIN\\"]')
    _4_sec_find_objs = NodeSequence('_4_sec_find_objs','find_objs')
    _4_sec_pick_move_to_bin = NodeSequence('_4_sec_pick_move_to_bin','pick_move_to_bin')
    _4_cond_found_obj = NodeCondition('_4_cond_found_obj','OBJECT_FOUND\\n[\\"CUP\\"]','test','test')
    _4_sec_pick_up = NodeSequence('_4_sec_pick_up','pick_up')
    _4_query_obj = NodeQuery('_4_query_obj','GET_POSE\\n[\\"CUP\\", CUP_POSE]')
    _4_query_bin = NodeQuery('_4_query_bin','GET_POSE\\n[\\"BIN\\", BIN_POSE]')
    _4_act_move_to_obj = NodeAction('_4_act_move_to_obj','MOVE_TO\\n[CUP_POSE]')
    _4_act_grab = NodeAction('_4_act_grab','grab')
    _4_act_lift = NodeAction('_4_act_lift','lift')
    _4_act_move = NodeAction('_4_act_move','move_above_bin')
    _4_sec_place = NodeSequence('_4_sec_place','place')
    _4_act_move_to_bin = NodeAction('_4_act_move_to_bin','MOVE_TO\\n[BIN_POSE]')    
    _4_act_close_gripper = NodeAction('_4_act_close_gripper','CLOSE\\nGRIPPER')
    _4_act_open_gripper = NodeAction('_4_act_open_gripper','OPEN\\nGRIPPER')
    _4_act_wait = NodeAction('_4_act_wait','WAIT\\n[5.0s]')
    _4_sel_check = NodeSelector('_4_sel_check','sel_check')
    _4_sec_check = NodeSequence('_4_sec_check','sec_check')
    _4_var_cup = NodeVariable('_4_var_cup','[CUP_POSE]',attach=False)
    _4_var_bin = NodeVariable('_4_var_bin','[BIN_POSE]',attach=False)


    _5_para_look_and_grab = NodeParallelAll('_5_para_look_and_grab','detect_obj_move_to_bin')
    _5_act_detect_object_cup = NodeAction('_5_act_detect_object_c','DETECT_OBJ\\n[\\"CUP\\"]')
    _5_act_detect_object_bin = NodeAction('_5_act_detect_object_b','DETECT_OBJ\\n[\\"BIN\\"]')
    _5_sec_find_objs = NodeSequence('_5_sec_find_objs','find_objs')
    _5_sec_pick_move_to_bin = NodeSequence('_5_sec_pick_move_to_bin','pick_move_to_bin')
    _5_cond_found_obj = NodeCondition('_5_cond_found_obj','OBJECT_FOUND\\n[\\"CUP\\"]','test','test')
    _5_sec_pick_up = NodeSequence('_5_sec_pick_up','pick_up')
    _5_query_obj = NodeQuery('_5_query_obj','GET_POSE\\n[\\"CUP\\", CUP_POSE]')
    _5_query_bin = NodeQuery('_5_query_bin','GET_POSE\\n[\\"BIN\\", BIN_POSE]')
    _5_act_move_to_obj = NodeAction('_5_act_move_to_obj','MOVE_TO\\n[CUP_POSE]')
    _5_act_grab = NodeAction('_5_act_grab','grab')
    _5_act_lift = NodeAction('_5_act_lift','lift')
    _5_act_move = NodeAction('_5_act_move','move_above_bin')
    _5_sec_place = NodeSequence('_5_sec_place','place')
    _5_act_move_to_bin = NodeAction('_5_act_move_to_bin','MOVE_TO\\n[BIN_POSE]')    
    _5_act_close_gripper = NodeAction('_5_act_close_gripper','CLOSE\\nGRIPPER')
    _5_act_open_gripper = NodeAction('_5_act_open_gripper','OPEN\\nGRIPPER')
    _5_act_wait = NodeAction('_5_act_wait','WAIT\\n[5.0s]')
    _5_sel_check = NodeSelector('_5_sel_check','sel_check')
    _5_sec_check = NodeSequence('_5_sec_check','sec_check')
    _5_var_cup = NodeVariable('_5_var_cup','[CUP_POSE]',attach=False)
    _5_var_bin = NodeVariable('_5_var_bin','[BIN_POSE]',attach=False)

    # # act_lift.set_flag(True)
    # self.root.add_child(para_look_and_grab)
    # para_look_and_grab.add_child(act_detect_object_cup)
    # para_look_and_grab.add_child(act_detect_object_bin)
    # para_look_and_grab.add_child(sec_find_objs)
    # sec_find_objs.add_child(cond_found_obj)
    # sec_find_objs.add_child(query_obj)
    # sec_find_objs.add_child(query_bin)
    # sec_find_objs.add_child(sec_pick_move_to_bin)
    # sec_pick_move_to_bin.add_child(act_move_to_obj)
    # sec_pick_move_to_bin.add_child(act_close_gripper)
    # sec_pick_move_to_bin.add_child(act_move_to_bin)
    # sec_pick_move_to_bin.add_child(act_open_gripper)

    self.root.add_child(para_look_and_grab)
    self.root.add_child(var_cup)
    self.root.add_child(var_bin)
    para_look_and_grab.add_child(act_detect_object_cup)
    para_look_and_grab.add_child(act_detect_object_bin)
    para_look_and_grab.add_child(sel_check)
    sel_check.add_child(sec_check)
    sel_check.add_child(act_wait)
    sec_check.add_child(cond_found_obj)
    sec_check.add_child(query_obj)
    sec_check.add_child(query_bin)
    sec_check.add_child(sec_pick_move_to_bin)
    sec_pick_move_to_bin.add_child(act_move_to_obj)
    sec_pick_move_to_bin.add_child(act_close_gripper)
    sec_pick_move_to_bin.add_child(act_move_to_bin)
    sec_pick_move_to_bin.add_child(act_open_gripper)

    _1_para_look_and_grab.add_child(_1_act_detect_object_cup)
    _1_para_look_and_grab.add_child(_1_act_detect_object_bin)
    _1_para_look_and_grab.add_child(_1_sel_check)
    _1_sel_check.add_child(_1_sec_check)
    _1_sel_check.add_child(_1_act_wait)
    _1_sec_check.add_child(_1_cond_found_obj)
    _1_sec_check.add_child(_1_query_obj)
    _1_sec_check.add_child(_1_query_bin)
    _1_sec_check.add_child(_1_sec_pick_move_to_bin)
    _1_sec_pick_move_to_bin.add_child(_1_act_move_to_obj)
    _1_sec_pick_move_to_bin.add_child(_1_act_close_gripper)
    _1_sec_pick_move_to_bin.add_child(_1_act_move_to_bin)
    _1_sec_pick_move_to_bin.add_child(_1_act_open_gripper)

    _3_para_look_and_grab.add_child(_3_act_detect_object_cup)
    _3_para_look_and_grab.add_child(_3_act_detect_object_bin)
    _3_para_look_and_grab.add_child(_3_sel_check)
    _3_sel_check.add_child(_3_sec_check)
    _3_sel_check.add_child(_3_act_wait)
    _3_sec_check.add_child(_3_cond_found_obj)
    _3_sec_check.add_child(_3_query_obj)
    _3_sec_check.add_child(_3_query_bin)
    _3_sec_check.add_child(_3_sec_pick_move_to_bin)
    _3_sec_pick_move_to_bin.add_child(_3_act_move_to_obj)
    _3_sec_pick_move_to_bin.add_child(_3_act_close_gripper)
    _3_sec_pick_move_to_bin.add_child(_3_act_move_to_bin)
    _3_sec_pick_move_to_bin.add_child(_3_act_open_gripper)

    _2_para_look_and_grab.add_child(_2_act_detect_object_cup)
    _2_para_look_and_grab.add_child(_2_act_detect_object_bin)
    _2_para_look_and_grab.add_child(_2_sel_check)
    _2_sel_check.add_child(_2_sec_check)
    _2_sel_check.add_child(_2_act_wait)
    _2_sec_check.add_child(_2_cond_found_obj)
    _2_sec_check.add_child(_2_query_obj)
    _2_sec_check.add_child(_2_query_bin)
    _2_sec_check.add_child(_2_sec_pick_move_to_bin)
    _2_sec_pick_move_to_bin.add_child(_2_act_move_to_obj)
    _2_sec_pick_move_to_bin.add_child(_2_act_close_gripper)
    _2_sec_pick_move_to_bin.add_child(_2_act_move_to_bin)
    _2_sec_pick_move_to_bin.add_child(_2_act_open_gripper)

    _4_para_look_and_grab.add_child(_4_act_detect_object_cup)
    _4_para_look_and_grab.add_child(_4_act_detect_object_bin)
    _4_para_look_and_grab.add_child(_4_sel_check)
    _4_sel_check.add_child(_4_sec_check)
    _4_sel_check.add_child(_4_act_wait)
    _4_sec_check.add_child(_4_cond_found_obj)
    _4_sec_check.add_child(_4_query_obj)
    _4_sec_check.add_child(_4_query_bin)
    _4_sec_check.add_child(_4_sec_pick_move_to_bin)
    _4_sec_pick_move_to_bin.add_child(_4_act_move_to_obj)
    _4_sec_pick_move_to_bin.add_child(_4_act_close_gripper)
    _4_sec_pick_move_to_bin.add_child(_4_act_move_to_bin)
    _4_sec_pick_move_to_bin.add_child(_4_act_open_gripper)

    _5_para_look_and_grab.add_child(_5_act_detect_object_cup)
    _5_para_look_and_grab.add_child(_5_act_detect_object_bin)
    _5_para_look_and_grab.add_child(_5_sel_check)
    _5_sel_check.add_child(_5_sec_check)
    _5_sel_check.add_child(_5_act_wait)
    _5_sec_check.add_child(_5_cond_found_obj)
    _5_sec_check.add_child(_5_query_obj)
    _5_sec_check.add_child(_5_query_bin)
    _5_sec_check.add_child(_5_sec_pick_move_to_bin)
    _5_sec_pick_move_to_bin.add_child(_5_act_move_to_obj)
    _5_sec_pick_move_to_bin.add_child(_5_act_close_gripper)
    _5_sec_pick_move_to_bin.add_child(_5_act_move_to_bin)
    _5_sec_pick_move_to_bin.add_child(_5_act_open_gripper)

    self.root.add_child(_1_para_look_and_grab)
    self.root.add_child(_2_para_look_and_grab)
    self.root.add_child(_3_para_look_and_grab)
    self.root.add_child(_4_para_look_and_grab)
    self.root.add_child(_5_para_look_and_grab)


    # self.pub_ = rospy.Publisher('/beetree/dot',String)
    print self.root.generate_dot()

    # CODE FOR TESTING DELETE FUNCTIONALITY ####################################


# The way I would implement it is this: you have a parallel which is running "check dog" and a sequence "look and run". The sequence has two children "look" which is a condition that checks a value set by "check dog" and "run" which does what it says.  In this manner, you can still have the same behavior where every action in the sequence is atomic.

    # self.dot_widget1.set_dotcode(self.root.generate_dot())
    # self.dot_widget1.zoom_to_fit()

    # act_close_gripper.remove_self()
    # act_grab.remove_self()

    # self.dot_widget2.set_dotcode(self.root.generate_dot())
    # self.dot_widget2.zoom_to_fit()

    # # sec_pick_up.remove_all_children()
    # act_close_gripper = NodeAction(sec_place,'act_close_gripper','release')
    # act_grab = NodeAction(sec_pick_up,'act_grab','grab')

    # self.dot_widget3.set_dotcode(self.root.generate_dot())
    # self.dot_widget3.zoom_to_fit()

    # # sec_pick_move_to_bin.remove_child('sec_place')
    # act_wait = NodeAction(None,'act_wait','wait')
    # act_move_to_bin.add_sibling(act_wait)
    # self.dot_widget4.set_dotcode(self.root.generate_dot())
    # self.dot_widget4.zoom_to_fit()
    ############################################################################
    self.dot_widget1.set_dotcode(self.root.generate_dot())
    self.dot_widget1.zoom_to_fit()

    # act_stop = NodeAction('act_stop','stop')
    # act_lift.add_sibling(act_stop)

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

        


















