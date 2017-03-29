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
from costar_robot_msgs.srv import SmartMove, Object, ObjectRequest, SmartMoveRequest
from smart_waypoint_manager import SmartWaypointManager
from predicator_msgs.msg import *

colors = ColorOptions().colors

global_manager = None

# Node Wrappers -----------------------------------------------------------
class NodeActionQueryGUI(NodeGUI):
    def __init__(self):
        super(NodeActionQueryGUI,self).__init__('purple')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/query.ui'

        self.title.setText('QUERY')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
        self.setStyleSheet('background-color:'+colors['purple'].normal+' ; color:#ffffff')

        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.layout_.addWidget(self.waypoint_ui)

        self.selected_region = 'none'
        self.selected_reference = 'none'
        self.selected_object = None
        self.selected_smartmove = None

        self.command_waypoint_name = None
        #self.listener_ = tf.TransformListener()

        global global_manager
        if global_manager is None:
            global_manager = SmartWaypointManager()
        self.manager = global_manager

        self.waypoint_ui.reference_list.itemClicked.connect(self.reference_selected_cb)
        self.waypoint_ui.region_list.itemClicked.connect(self.region_selected_cb)
        self.waypoint_ui.object_list.itemClicked.connect(self.object_selected_cb)
        self.waypoint_ui.smartmove_list.itemClicked.connect(self.smartmove_selected_cb)

        self.update_regions()
        self.update_references()
        self.update_objects()

    def refresh_data(self):
        self.update_regions()
        self.update_references()
        self.update_objects()

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

    def update_objects(self):
        objects = []
        rospy.loginfo("detecting objects")
        objects = self.manager.get_detected_object_classes()
        rospy.loginfo(objects)
        self.waypoint_ui.object_list.clear()
        for m in objects:
            self.waypoint_ui.object_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.object_list.sortItems()
        self.waypoint_ui.object_list.setCurrentRow(0)    

    def update_smartmoves(self):
        smartmoves = []
        self.manager.load_all()
        rospy.loginfo(self.selected_object)
        smartmoves = self.manager.get_moves_for_class(self.selected_object)
        rospy.loginfo(smartmoves)
        self.waypoint_ui.smartmove_list.clear()
        for m in smartmoves:
            self.waypoint_ui.smartmove_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.smartmove_list.sortItems()
        self.waypoint_ui.smartmove_list.setCurrentRow(0)  

    def save_data(self,data):
        data['region'] = {'value':self.selected_region}
        data['object'] = {'value':self.selected_object}
        data['reference'] = {'value':self.selected_reference}
        data['smartmove'] = {'value':self.selected_smartmove}
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
        self.update_regions()
        self.update_references()
        self.update_objects()

    def generate(self):
        if all([self.name.full(), self.selected_object, self.selected_smartmove]):
            # rospy.loginfo('Generating SmartMove with reference='+str(self.selected_reference)+' and smartmove='+str(self.selected_smartmove))
            return NodeActionQuery( self.get_name(),
                                        self.get_label(),
                                        self.selected_region,
                                        self.selected_object,
                                        self.selected_smartmove,
                                        self.selected_reference,
                                        self.manager)

            #"%s %s %s %s"%(self.selected_smartmove,self.selected_objet,self.selected_region,self.selected_reference),
        else:
            rospy.logerr('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'


# Nodes -------------------------------------------------------------------
class NodeActionQuery(Node):
    def __init__(self,name,label,selected_region,selected_object,selected_smartmove,selected_reference,smartmove_manager):
        L = 'QUERY \\n ['+selected_smartmove+'] \\n [' + selected_region + ' ' + selected_reference + ']'
        super(NodeActionQuery,self).__init__(name,L,colors['purple'].normal)
        self.selected_region = selected_region
        self.selected_reference = selected_reference
        self.selected_object = selected_object
        self.selected_smartmove = selected_smartmove
        self.manager = smartmove_manager
        #self.listener_ = smartmove_manager.listener
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
                        rospy.loginfo('Query Service [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('Query Service [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('Query Service [' + self.name_ + '] succeeded')
                        self.set_color(colors['gray'].normal)
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('Query Service [' + self.name_ + '] failed')
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

        # Check to see if service exists
        try:
            rospy.wait_for_service('/costar/Query')
        except rospy.ROSException as e:
            rospy.logerr('Could not find Query service')
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            query_proxy = rospy.ServiceProxy('/costar/Query',SmartMove)
            msg = SmartMoveRequest()
            msg.pose = self.manager.lookup_waypoint(self.selected_object,self.selected_smartmove)
            msg.obj_class = self.selected_object
            msg.name = self.selected_smartmove
            predicate = PredicateStatement()
            predicate.predicate = self.selected_region 
            predicate.params = ['*',self.selected_reference,'world']
            msg.predicates = [predicate]
            # Send SmartMove Command
            rospy.loginfo('Query Started')
            result = query_proxy(msg)
            if 'FAILURE' in str(result.ack):
                rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.loginfo('Single Servo Move Finished')
                rospy.loginfo('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr('There was a problem with the tf lookup:')
            rospy.logerr(e)
            self.finished_with_success = False
            return
        except rospy.ServiceException, e:
            rospy.logerr('Service failed!')
            rospy.logerr(e)
            self.finished_with_success = False
            return


# Node Wrappers -----------------------------------------------------------
class CollisionGUI(NodeGUI):
    def __init__(self,enable):
        super(CollisionGUI,self).__init__('purple')

        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_plugins') + '/ui/collision.ui'

        self.enable = enable
        self.title.setText('QUERY')
        self.title.setStyleSheet('background-color:'+colors['purple'].normal+';color:#ffffff')
        self.setStyleSheet('background-color:'+colors['purple'].normal+' ; color:#ffffff')

        self.waypoint_ui = QWidget()
        uic.loadUi(ui_path, self.waypoint_ui)
        self.layout_.addWidget(self.waypoint_ui)

        self.selected_object = None

        global global_manager
        if global_manager is None:
            global_manager = SmartWaypointManager()
        self.manager = global_manager

        self.waypoint_ui.object_list.itemClicked.connect(self.object_selected_cb)
        self.update_objects()

    def refresh_data(self):
        self.update_objects()

    def object_selected_cb(self,item):
        rospy.logerr("Selecting obj = "+str(item.text()))
        self.selected_object = str(item.text())

    def update_objects(self):
        objects = []
        objects = self.manager.get_detected_objects()
        self.waypoint_ui.object_list.clear()
        for m in objects:
            self.waypoint_ui.object_list.addItem(QListWidgetItem(m.strip('/')))
        self.waypoint_ui.object_list.sortItems()
        self.waypoint_ui.object_list.setCurrentRow(0)    

    def save_data(self,data):
        data['object'] = {'value':self.selected_object}
        return data

    def load_data(self,data):
        self.manager.load_all()
        if data.has_key('object'):
            if data['object']['value']!=None:
                self.selected_object = (data['object']['value'])
        self.update_objects()

    def generate(self):
        rospy.loginfo("Selecting obj = "+str([self.name.full(), self.selected_object]))
        if all([self.name.full(), self.selected_object]):
            return NodeActionCollision( self.get_name(),
                                        self.get_label(),
                                        self.selected_object,
                                        self.manager,
                                        self.enable)

            #"%s %s %s %s"%(self.selected_smartmove,self.selected_objet,self.selected_region,self.selected_reference),
        else:
            rospy.logerr('NODE NOT PROPERLY DEFINED')
            return 'ERROR: node not properly defined'


# Nodes -------------------------------------------------------------------
class NodeActionCollision(Node):
    def __init__(self,name,label,selected_object,smartmove_manager,enable):
        if enable:
            info="ENABLE COLLISION"
            self.srv_name = "/costar/EnableCollision"
        else:
            info="DISABLE COLLISION"
            self.srv_name = "/costar/DisableCollision"
        L = '%s WITH [%s]'%(info,selected_object)
        super(NodeActionCollision,self).__init__(name,L,colors['purple'].normal)
        self.selected_object = selected_object
        self.manager = smartmove_manager
        #self.listener_ = smartmove_manager.listener
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
                        rospy.loginfo('Query Service [' + self.name_ + '] running')
                        self.running = True
                        return self.set_status('RUNNING')
                    except Exception, errtxt:
                        rospy.loginfo('Query Service [' + self.name_ + '] thread failed')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('FAILURE')
                        
            else:# If thread is running
                if self.service_thread.is_alive():
                    return self.set_status('RUNNING')
                else:
                    if self.finished_with_success == True:
                        rospy.loginfo('Query Service [' + self.name_ + '] succeeded')
                        self.running = False
                        self.needs_reset = True
                        return self.set_status('SUCCESS')
                    else:
                        rospy.loginfo('Query Service [' + self.name_ + '] failed')
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

        # Check to see if service exists
        try:
            rospy.wait_for_service(self.srv_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find service: %s'%self.srv_name)
            self.finished_with_success = False
            return
        # Make servo call to set pose
        try:
            query_proxy = rospy.ServiceProxy(self.srv_name,Object)
            msg = ObjectRequest()
            msg.object = self.selected_object
            # Send SmartMove Command
            rospy.loginfo('Query Started')
            result = query_proxy(msg)
            if 'FAILURE' in str(result.ack):
                rospy.logwarn('Servo failed with reply: '+ str(result.ack))
                self.finished_with_success = False
                return
            else:
                rospy.loginfo('Single Servo Move Finished')
                rospy.loginfo('Robot driver reported: '+str(result.ack))
                self.finished_with_success = True
                return

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            rospy.logerr('There was a problem with the tf lookup:')
            rospy.logerr(e)
            self.finished_with_success = False
            return
        except rospy.ServiceException, e:
            rospy.logerr('Service failed!')
            rospy.logerr(e)
            self.finished_with_success = False
            return

class EnableCollisionGUI(CollisionGUI):
    def __init__(self):
        super(EnableCollisionGUI,self).__init__(True)

class DisableCollisionGUI(CollisionGUI):
    def __init__(self):
        super(DisableCollisionGUI,self).__init__(False)
