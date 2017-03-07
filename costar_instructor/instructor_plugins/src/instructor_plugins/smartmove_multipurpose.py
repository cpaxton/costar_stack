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
import costar_robot_msgs
from costar_robot_msgs.srv import *
from smart_waypoint_manager import SmartWaypointManager
from predicator_msgs.msg import *

C = ColorOptions()

global_manager = None


# Node Wrappers -----------------------------------------------------------
class NodeActionSmartmoveMultiPurposeGUI(NodeGUI):
    def __init__(self, grasp=True):
        super(NodeActionSmartmoveMultiPurposeGUI,self).__init__('green')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/action_smart_grasp.ui'

        self.grasp = grasp

        self.title.setText('SMARTMOVE')
        self.title.setStyleSheet('background-color:'+C.colors['green'].normal+';color:#ffffff')
        self.setStyleSheet('background-color:'+C.colors['green'].normal+' ; color:#ffffff')

        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.layout_.addWidget(self.waypoint_ui)

        self.selected_region = 'none'
        self.selected_reference = 'none'
        self.selected_object = None
        self.selected_smartmove = None

        self.command_waypoint_name = None
        self.command_vel = .75
        self.command_acc = .75
        self.command_backoff = 0.05
        self.listener_ = tf.TransformListener()

        global global_manager
        if global_manager is None:
            global_manager = SmartWaypointManager()
        self.manager = global_manager

        self.waypoint_ui.reference_list.itemClicked.connect(self.reference_selected_cb)
        self.waypoint_ui.region_list.itemClicked.connect(self.region_selected_cb)
        self.waypoint_ui.object_list.itemClicked.connect(self.object_selected_cb)
        self.waypoint_ui.smartmove_list.itemClicked.connect(self.smartmove_selected_cb)

        self.waypoint_ui.acc_slider.valueChanged.connect(self.acc_changed)
        self.waypoint_ui.vel_slider.valueChanged.connect(self.vel_changed)
        self.waypoint_ui.backoff_slider.valueChanged.connect(self.backoff_changed)
        #self.waypoint_ui.refresh_btn.clicked.connect(self.update_relative_waypoints)

        self.manager.load_all()
        self.update_regions()
        self.update_references()
        self.update_objects()

    def vel_changed(self,t):
        self.waypoint_ui.vel_field.setText(str(float(t)))
        self.command_vel = float(t)/100*1.5

    def acc_changed(self,t):
        self.waypoint_ui.acc_field.setText(str(float(t)))
        self.command_acc = float(t)/100*1.5

    def backoff_changed(self,t):
        self.waypoint_ui.backoff_field.setText(str(float(t/10.)))
        self.command_backoff = float(t)/1000.

    def reference_selected_cb(self,item):
        self.selected_reference = str(item.text())

    def region_selected_cb(self,item):
        self.selected_region = str(item.text())

    def object_selected_cb(self,item):
        self.selected_object = str(item.text())
        self.update_smartmoves()
        
    def smartmove_selected_cb(self,item):
        self.selected_smartmove = str(item.text())

    def update_regions(self):
        # TODO predicator call to update different region options for the objects (i.e. "left of X")
        # populate regions with result
        # for now loading from librarian
        regions = self.manager.get_available_predicates()

        if regions is None:
          regions = []

        self.waypoint_ui.region_list.clear()
        for m in regions:
            self.waypoint_ui.region_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.region_list.sortItems()
        self.waypoint_ui.region_list.setCurrentRow(0)
        if self.waypoint_ui.region_list.currentItem() is not None and self.selected_region == 'none':
            self.selected_region = str(self.waypoint_ui.region_list.currentItem().text())

    def update_references(self):
        # TODO use a predicator call to populate list of references or look them up on rosparam
        # for now loading from librarian
        references= self.manager.get_reference_frames()

        if references is None:
          references = []

        self.waypoint_ui.reference_list.clear()
        for m in references:
            self.waypoint_ui.reference_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.reference_list.sortItems()
        self.waypoint_ui.reference_list.setCurrentRow(0)
        if self.waypoint_ui.reference_list.currentItem() is not None and self.selected_reference == 'none':
            self.selected_reference = str(self.waypoint_ui.reference_list.currentItem().text())

    def update_objects(self):
        objects = []
        objects = self.manager.get_available_object_classes()
        self.waypoint_ui.object_list.clear()
        for m in objects:
            self.waypoint_ui.object_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.object_list.sortItems()
        self.waypoint_ui.object_list.setCurrentRow(0)
        if self.waypoint_ui.object_list.currentItem() is not None and self.selected_object is None:
            self.selected_object = str(self.waypoint_ui.object_list.currentItem().text())

    def update_smartmoves(self):
        smartmoves = self.manager.get_moves_for_class(self.selected_object)
        self.waypoint_ui.smartmove_list.clear()
        for m in smartmoves:
            self.waypoint_ui.smartmove_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.smartmove_list.sortItems()
        self.waypoint_ui.smartmove_list.setCurrentRow(0)
        if self.waypoint_ui.smartmove_list.currentItem() is not None and self.selected_smartmove is None:
            self.selected_smartmove = str(self.waypoint_ui.smartmove_list.currentItem().text())

    def save_data(self,data):
        data['region'] = {'value':self.selected_region}
        data['object'] = {'value':self.selected_object}
        data['reference'] = {'value':self.selected_reference}
        data['smartmove'] = {'value':self.selected_smartmove}
        data['vel'] = {'value':self.command_vel}
        data['acc'] = {'value':self.command_acc}
        data['backoff'] = {'value':self.command_backoff}
        return data

    def load_data(self,data):
        self.manager.load_all()
        if data.has_key('region'):
            if data['region']['value']!=None:
                self.selected_region = (data['region']['value'])
        if data.has_key('object'):
            if data['object']['value']!=None:
                self.selected_object = (data['object']['value'])
        if data.has_key('reference'):
            if data['reference']['value']!=None:
                self.selected_reference = (data['reference']['value'])
        if data.has_key('smartmove'):
            if data['smartmove']['value']!=None:
                self.selected_smartmove = (data['smartmove']['value'])
        if data.has_key('vel'):
            if data['vel']['value']!=None:
                self.command_vel = data['vel']['value']
                self.waypoint_ui.vel_field.setText(str(float(self.command_vel)*100/1.5))
                self.waypoint_ui.vel_slider.setSliderPosition(int(float(self.command_vel)*100/1.5))
        if data.has_key('acc'):
            if data['acc']['value']!=None:
                self.command_acc = data['acc']['value']
                self.waypoint_ui.acc_field.setText(str(float(self.command_acc)*100/1.5))
                self.waypoint_ui.acc_slider.setSliderPosition(int(float(self.command_acc)*100/1.5))
        if data.has_key('backoff'):
            if data['backoff']['value']!=None:
                self.command_backoff = data['backoff']['value']
                self.waypoint_ui.backoff_field.setText(str(float(self.command_backoff)*1000.))
                self.waypoint_ui.backoff_slider.setSliderPosition(int(float(self.command_backoff)*1000.))
        self.update_regions()
        self.update_references()
        self.update_objects()

    def generate(self):
        if all([self.name.full(), self.selected_object, self.selected_smartmove]):
            rospy.logwarn('Generating SmartMove with reference='+str(self.selected_reference)+' and smartmove='+str(self.selected_smartmove))
            return NodeActionSmartmoveMultiPurpose( self.get_name(),
                                        self.get_label(),
                                        self.selected_region,
                                        self.selected_object,
                                        self.selected_smartmove,
                                        self.selected_reference,
                                        self.command_vel,
                                        self.command_acc,
                                        self.command_backoff,
                                        self.manager,
                                        self.grasp)

            #"%s %s %s %s"%(self.selected_smartmove,self.selected_objet,self.selected_region,self.selected_reference),
        else:
            rospy.logwarn('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'

    def refresh_data(self):
        self.manager.load_all()
        self.update_regions()
        self.update_references()
        self.update_objects()


class NodeActionSmartmoveGraspGUI(NodeActionSmartmoveMultiPurposeGUI):
    def __init__(self):
        super(NodeActionSmartmoveGraspGUI, self).__init__(True)

class NodeActionSmartmoveReleaseGUI(NodeActionSmartmoveMultiPurposeGUI):
    def __init__(self):
        super(NodeActionSmartmoveReleaseGUI, self).__init__(False)

# Nodes -------------------------------------------------------------------
class NodeActionSmartmoveMultiPurpose(Node):
    def __init__(self,name,label,selected_region,selected_object,selected_smartmove,selected_reference,vel,acc,backoff,smartmove_manager,grasp):
        if grasp:
            display_name = "GRASP"
        else:
            display_name = "RELEASE"
        L = 'SMART ' + display_name + \
            ' to \\n pose [%s]\\n'%(selected_smartmove) + \
            'for any [%s %s %s]'%(selected_object,selected_region,selected_reference) + '\n' + \
            "with backoff dist: %f cm"%(backoff*100)
        super(NodeActionSmartmoveMultiPurpose,self).__init__(name,
            L,
            C.colors['green'].normal)
        self.selected_region = selected_region
        self.selected_reference = selected_reference
        self.selected_object = selected_object
        self.selected_smartmove = selected_smartmove
        self.command_acc = acc
        self.command_vel = vel
        self.command_backoff = backoff
        self.grasp = grasp
        self.manager = smartmove_manager
        self.listener_ = smartmove_manager.listener
        # Thread
        self.service_thread = Thread(target=self.make_service_call, args=('',1))
        # Reset params
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False

    def execute(self):
        if self.needs_reset:
            rospy.loginfo('Waypoint Service [' + self.name_ + '] already ['+self.get_status()+'], needs reset')
            return self.get_status()
        else:
            if not self.running: # Thread is not running
                if self.finished_with_success == None: # Service was never called
                    try:
                        self.service_thread.start()
                        rospy.loginfo('SmartMove Service [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('SmartMove Service [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('SmartMove Service [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('SmartMove Service [' + self.name_ + '] failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')

    def reset_self(self):
        self.service_thread = Thread(target=self.make_service_call, args=('',1))
        self.running = False
        self.finished_with_success = None
        self.needs_reset = False

    def make_service_call(self,request,*args):

        self.manager.load_all()

        if self.grasp:
            srv_name = "/costar/SmartGrasp"
        else:
            srv_name = "/costar/SmartRelease"

        # Check to see if service exists
        try:
            rospy.wait_for_service(srv_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find SmartMove service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            smartmove_proxy = rospy.ServiceProxy(srv_name,SmartMove)
            msg = SmartMoveRequest()
            msg.pose = self.manager.lookup_waypoint(self.selected_object,self.selected_smartmove)
            msg.obj_class = self.selected_object
            msg.name = self.selected_smartmove
            predicate = PredicateStatement()
            predicate.predicate = self.selected_region 
            predicate.params = ['*',self.selected_reference,'world']
            msg.predicates = [predicate]
            msg.vel = self.command_vel
            msg.accel = self.command_acc
            msg.backoff = self.command_backoff
            # Send SmartMove Command
            rospy.logwarn('SmartMove Started')
            result = smartmove_proxy(msg)
            if 'FAILURE' in str(result.ack):
                rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.logwarn('Single Servo Move Finished')
                rospy.logwarn('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logwarn('There was a problem with the tf lookup:')
            rospy.logwarn(e)
            self.finished_with_success = False
            return
        except rospy.ServiceException, e:
            rospy.logwarn('Service failed!')
            rospy.logwarn(e)
            self.finished_with_success = False
            return
