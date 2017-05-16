#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy
import rospkg
from std_msgs.msg import *
from threading import Thread
# Qt 
from PyQt4 import QtGui,  QtCore,  uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField, ColorOptions
# Predicator
from predicator_msgs.msg import *
from predicator_msgs import srv

# Colors
colors = ColorOptions().colors

# GUI
class NodeKnowledgeTestGUI(NodeGUI):
    def __init__(self):
        super(NodeKnowledgeTestGUI, self).__init__('purple')
        self.title.setText('SYSTEM KNOWLEDGE')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/knowledge.ui'
        self.ui = QWidget()
        uic.loadUi(ui_path, self.ui)
        self.layout_.addWidget(self.ui)

        self.ui.target_box.activated.connect(self.target_selected)
        self.ui.knowledge_box.activated.connect(self.predicate_selected)
        self.ui.value_box.activated.connect(self.value_selected)

        self.ui.target_box.addItem('<select value>')
        self.ui.target_box.insertSeparator(1)
        index = self.ui.target_box.model().index(0, self.ui.target_box.modelColumn(),self.ui.target_box.rootModelIndex());
        self.ui.target_box.model().itemFromIndex(index).setSelectable(False)

        self.ui.target_box.setEditable(False)
        self.ui.knowledge_box.setEditable(False)
        self.ui.value_box.setEditable(False)


        ### Git initial list of targets ###
        rospy.loginfo('Waiting for service...')
        try:
            rospy.wait_for_service('/predicator/get_possible_assignment',2)
            rospy.loginfo('FOUND')
        except:
            rospy.logerr('Could not find service')
        try:
            proxy = rospy.ServiceProxy('predicator/get_possible_assignment', srv.GetTypedList)
            
            rospy.loginfo('Found TARGETS:')
            self.found_assignments = proxy().data
            rospy.loginfo(self.found_assignments)
            for a in self.found_assignments:
                self.ui.target_box.addItem(str(a).upper())
        except rospy.ServiceException, e:
            print e

        # Finish
        self.reset()

    def reset(self):
        self.ui.knowledge_box.hide()
        self.ui.knowledge_label.hide()
        self.ui.result_label.hide()
        self.ui.result_tag.hide()
        self.ui.value_widget.hide()
        self.ui.statement_label.hide()
        self.ui.value_box.setCurrentIndex(0)
        self.ui.knowledge_box.setCurrentIndex(0)
        self.ui.target_box.setCurrentIndex(0)

    def reset_knowledge(self):
        self.ui.knowledge_box.clear()
        self.ui.knowledge_box.addItem('<select value>')
        self.ui.knowledge_box.insertSeparator(1)
        index = self.ui.knowledge_box.model().index(0, self.ui.knowledge_box.modelColumn(),self.ui.knowledge_box.rootModelIndex());
        self.ui.knowledge_box.model().itemFromIndex(index).setSelectable(False)

    def reset_value(self):
        self.ui.value_box.clear()
        self.ui.value_box.addItem('<select value>')
        self.ui.value_box.insertSeparator(1)
        self.ui.value_box.addItem('TRUE')
        self.ui.value_box.addItem('FALSE')
        index = self.ui.value_box.model().index(0, self.ui.value_box.modelColumn(),self.ui.value_box.rootModelIndex());
        self.ui.value_box.model().itemFromIndex(index).setSelectable(False)

    def target_selected(self,index):
        obj = str(self.ui.target_box.itemText(index)).lower()
        rospy.loginfo('Selected object ['+str(obj)+']')
        self.selected_target = str(obj)
        self.reset_knowledge()
        self.ui.knowledge_box.show()
        self.ui.knowledge_label.show()
        try:
            rospy.wait_for_service('/predicator/get_predicate_names_by_assignment',2)
        except:
            rospy.logerr('Could not find service')
            return
        try:
            proxy = rospy.ServiceProxy('predicator/get_predicate_names_by_assignment', srv.GetTypedList)
            rospy.loginfo('fetching predicates for ['+self.selected_target+']')
            self.found_predicates = proxy(self.selected_target).data
            rospy.loginfo(self.found_predicates)
            for a in self.found_predicates:
                self.ui.knowledge_box.addItem(str(a))
        except rospy.ServiceException, e:
            print e

    def predicate_selected(self,index):
        pred = str(self.ui.knowledge_box.itemText(index)).lower()
        rospy.loginfo('Selected predicate ['+str(pred).upper()+']')
        self.ui.value_box.clear()
        self.reset_value()
        self.ui.result_label.show()
        self.ui.result_tag.show()
        self.ui.value_widget.show()
        self.selected_pred = pred

        try:
            rospy.wait_for_service('/predicator/test_predicate',2)
        except:
            rospy.logerr('Could not find service')
        try:
            proxy = rospy.ServiceProxy('predicator/test_predicate', srv.TestPredicate)
            self.selected_pred_value = proxy(PredicateStatement( self.selected_pred, 1, PredicateStatement.TRUE, 1, [self.selected_target, '', ''],[])).found
            rospy.loginfo(self.selected_pred_value)
        except rospy.ServiceException, e:
            print e
        self.ui.result_label.setText(self.selected_target.upper()+':  '+self.selected_pred+' = '+str(self.selected_pred_value).upper()+'')

    def value_selected(self,index):
        val = str(self.ui.value_box.itemText(index)).lower()
        rospy.loginfo(val)
        self.selected_value = True if val == 'true' else False
        rospy.loginfo(self.selected_value)
        self.ui.statement_label.show()
        self.ui.statement_label.setText('This test will succeed if\n'+self.selected_target.upper()+':  '+self.selected_pred+'\nis '+str(self.selected_value).upper())


    def generate(self):
        if all([self.name.full(), self.selected_target != None, self.selected_value != None, self.selected_pred != None]):
            return NodeKnowledgeTest(self.get_name(), self.get_label(), self.selected_target, self.selected_value, self.selected_pred)
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

    def save_data(self,data):
        data['target'] = {'value':self.selected_target}
        data['predicate'] = {'value':self.selected_pred}
        data['value'] = {'value':self.selected_value}
        return data

    def load_data(self,data):
        if data.has_key('target'):
            if data['target']['value']!=None:
                self.selected_target = data['target']['value']
        if data.has_key('predicate'):
            if data['predicate']['value']!=None:
                self.selected_pred = data['predicate']['value']
        if data.has_key('value'):
            if data['value']['value']!=None:
                self.selected_value = data['value']['value']

# NODE
class NodeKnowledgeTest(Node):
    def __init__(self, name, label, target, value, predicate):
        L = 'IF '+target.upper()+'['+predicate+'] = '+str(value).upper()
        color = colors['purple'].normal
        super(NodeKnowledgeTest, self).__init__(name, L, color)
        self.target = target
        self.predicate = predicate
        self.value = value
        # Reset
        self.service_thread = Thread(target=self.make_service_call, args=('request',1))
        self.running = False
        self.needs_reset = False
        self.finished_with_success = None
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Knowledge Test [' + self.name_ + '] already returned ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                try:
                    self.service_thread.start()
                    self.running = True
                    return self.set_status('RUNNING')
                except Exception,  errtxt:
                    rospy.logerr('['+self.name_+']: FAILED TO START THREAD')
                    self.running = False
                    self.needs_reset = True
                    return self.set_status('FAILURE')
            else:
                # If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    self.running = False
                    self.needs_reset = True
                    if self.finished_with_success == True:
                        return self.set_status('SUCCESS')
                    else:
                        return self.set_status('FAILURE')

    def make_service_call(self,request,*args):
        # Find Service
        rospy.loginfo('Finding service...')
        try:
            rospy.wait_for_service('/predicator/test_predicate',2)
            rospy.loginfo('FOUND!')
        except:
            rospy.logerr('Could not find service')
        # Perform Test
        try:
            proxy = rospy.ServiceProxy('predicator/test_predicate', srv.TestPredicate)
            statement = PredicateStatement( self.predicate, 1, PredicateStatement.TRUE, 1, [self.target, '', ''],[])
            V = proxy(statement).found
            if V == self.value:
                rospy.loginfo('Value ['+str(V)+'] matched ['+str(self.value)+']')
                self.finished_with_success = True
                return
            else:
                rospy.loginfo('Value ['+str(V)+'] did NOT match ['+str(self.value)+']')
                self.finished_with_success = False
                return
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def reset_self(self):
        self.service_thread = Thread(target=self.make_service_call, args=('request',1))
        self.running = False
        self.needs_reset = False