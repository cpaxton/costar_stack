#!/usr/bin/env python

#import roslib; roslib.load_manifest('instructor_core')
#from roslib import rospack

import rospy
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
import rospkg
from beetree import *

import instructor_core
from instructor_core.srv import *

import os,sys, inspect, ast
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import JointState
import rospkg
import tf; 
import tf_conversions as tf_c
from instructor_core.instructor_qt import *
# Using roslib.rospack even though it is deprecated
import threading
import yaml
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import time
from copy import deepcopy

# ==============================================================
# SRVs
import costar_robot_msgs
from costar_robot_msgs.srv import *
from std_srvs.srv import Empty as EmptySrv

### TIMER ###################################################
global tic_time
def tic():
    global tic_time
    tic_time = time.time()
def toc():
    global tic_time
    elapsed = time.time() - tic_time ############## TIMER
    rospy.logerr(elapsed) ################## TIMER

### TIMER ###################################################

color_options = ColorOptions()
colors = color_options.colors

### GUI ELEMENTS --------------------------------------------------------- ### 
from instructor_core.instructor_gui_components import *
from instructor_core.smart_move_dialog import SmartMoveDialog
from instructor_core.jog_dialog import JogDialog
from instructor_core.waypoint_manager_dialog import WaypointManagerDialog
from instructor_core.robot_interface import RobotInterface

#############################################################################

def clear_cmd():
    os.system(['clear','cls'][os.name == 'nt'])
    pass

'''
Load the plugins that become elements we can add to instructor.
'''
def load_instructor_plugins(cases=[]):
    # NOTE: we no longer need to use this, at least for the time being.
    #to_check = rospack.rospack_depends_on_1('beetree')
    to_check = ["instructor_core", "instructor_plugins"]
    rp = rospkg.RosPack()
    # to_check = rospack.get_depends_on('beetree', implicit=False)
    clear_cmd()
    # print to_check
    plugins = []    
    descriptions = []
    names = []
    types = []
    groups = []
    for pkg in to_check:
        m = rp.get_manifest(pkg)
        p_modules = m.get_export('instructor', 'plugin')
        p_types = m.get_export('instructor', 'type')
        p_descriptions = m.get_export('instructor', 'description')
        p_names = m.get_export('instructor', 'name')
        p_groups = m.get_export('instructor', 'group')
        p_cases = m.get_export('instructor', 'cases')
        if not p_modules:
            continue
        if p_modules == []:
            pass

        for p_module, p_description, p_name, p_type, p_group, p_case in zip(p_modules,p_descriptions,p_names,p_types,p_groups,p_cases):
            if len(cases) == 0:
                include = True
            else:
                include = False
                allowed_cases = p_case.split(',')
                #rospy.logerr("allowed for " + p_name + ": " + str(allowed_cases))
                for case in cases:
                    if case in allowed_cases:
                        include = True
                        break

            if include:
                package = __import__(pkg)
                sub_mod = p_module.split('.')[1:][0]
                module = getattr(package, sub_mod)
                plugins.append(module)
                descriptions.append(p_description)
                names.append(p_name)               
                types.append(p_type)
                groups.append(p_group)

    return plugins, descriptions, names, types, groups

class Instructor(QWidget):
    toast_signal = pyqtSignal()
    def __init__(self,app):
        super(Instructor,self).__init__()
        #rospy.logwarn('INSTRUCTOR: STARTING UP...')
        self.app_ = app
        self.types__ = ['LOGIC',
                'ACTION',
                'CONDITION',
                'QUERY',
                'PROCESS',
                'SERVICE',
                'VARIABLE']
        self.colors__ = ['blue',
                'green',
                'purple',
                'orange',
                'pink',
                'gray',
                'gray']
        self.labels__ = ['BUILDING BLOCKS',
                'ROBOT ACTIONS',
                'SYSTEM KNOWLEDGE',
                'QUERIES',
                'PROCESSES',
                'SERVICE',
                'VARIABLES']

        self.cases = rospy.get_param("~cases","")
        if len(self.cases) > 0:
            self.cases = self.cases.split(',')
        else:
            self.cases = []
        #rospy.logerr(str(self.cases))

        # Load the ui attributes into the main widget
        self.rospack__ = rospkg.RosPack()
        ui_path = self.rospack__.get_path('instructor_core') + '/ui/view.ui'
        uic.loadUi(ui_path, self)
        # Create the Graph Visualization Pane
        self.drawer = Drawer(self)
        self.splitter.setStyleSheet('background-color:#444444')
        self.dot_widget = DotWidget()
        self.instructor_layout.addWidget(self.dot_widget)  
        self.splitter.splitterMoved.connect(self.splitter_moved)
        self.resizeEvent = self.window_resized
        self.setMouseTracking(True)  
        # Finish Up
        self.show()
        # Running the tree
        self.running__ = False
        self.run_timer_ = QTimer(self)
        self.connect(self.run_timer_, QtCore.SIGNAL("timeout()"), self.run)
        # Set up ros_ok watchdog timer to handle termination and ctrl-c
        self.ok_timer_ = QTimer(self)
        self.connect(self.ok_timer_, QtCore.SIGNAL("timeout()"), self.check_ok)
        self.ok_timer_.start(100)

        self.toast_timer_ = QTimer(self)
        self.connect(self.toast_timer_, QtCore.SIGNAL("timeout()"), self.toast_reset)
        self.toast_timer_.setSingleShot(True)
        self.toast_signal.connect(self.toast_update)

        # Load Settings
        self.settings = QSettings('settings.ini', QSettings.IniFormat)
        self.settings.setFallbacksEnabled(False) 
        self.resize( self.settings.value('size', QSize(800, 800), type=QSize) )
        self.move(self.settings.value('pos', QPoint(50, 50), type=QPoint))
        # self.showMaximized()

        self.sound_pub = rospy.Publisher('/audri/sound/sound_player',
            String,
            queue_size=1000)

        ### ROBOT ###
        # Robot Buttons #
        self.teach_btn = InterfaceButton('TEACH',colors['gray'],txtsz=10)
        self.servo_btn = InterfaceButton('SERVO',colors['gray'],txtsz=10)
        self.status_layout.insertWidget(0,self.teach_btn)
        self.status_layout.addWidget(self.servo_btn)
        self.robot_ = RobotInterface(self.status_label,self.teach_btn,self.servo_btn,self.sound_pub,self.toast)
        self.toast_label.hide()
        self.teach_btn.clicked.connect(self.robot_.teach)
        self.servo_btn.clicked.connect(self.robot_.servo)
        self.servo_btn.clicked.connect(self.servo_button_cb)
        #############

        ### CORE STRUCTURES ###
        self.state__ = []
        self.current_tree = {}
        self.current_generators = {}
        self.all_generators = {}
        self.current_node_type = None
        self.selected_node_label = None
        self.active_node_type = None
        self.current_node_info ={}
        self.current_node_generator = None
        self.current_plugin_names = {}
        self.current_node_types = {}
        self.root_node = None
        self.selected_subtree = None
        self.left_selected_node = None
        self.available_plugins = []
        self.core_plugins = []
        self.current_plugins = []
        self.active_plugin_widgets = []
        self.is_set_up = False
        #######################

        # Get known Beetree Builder Node Plugins
        self.parse_plugin_info()
        # Set up the gui with plugin information
        # Set up communication between node view and the rest of the app
        self.connect(self.dot_widget,SIGNAL("clicked"), self.node_leftclick_cb)
        self.connect(self.dot_widget,SIGNAL("right_clicked"), self.node_rightclick_cb)
        self.connect(self.dot_widget,SIGNAL("clicked_location"),self.node_leftclick_location_cb)
        # Set up librarian
        self.lib_set_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.lib_save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.lib_load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.lib_list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.lib_delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)
        self.lib_set_type_service('instructor_node')
        self.lib_set_type_service('instructor_subtree')
        self.set_up_gui()

    def toast(self,m):
        self.toast_message = m
        self.toast_signal.emit()

    def toast_update(self):
        self.toast_label.setStyleSheet('background-color:'+colors['red'].normal+'; color:#ffffff')
        self.toast_label.setText(str(self.toast_message))
        self.status_label.hide()
        self.toast_label.show()
        self.toast_timer_.start(3000)

    def toast_reset(self):
        self.status_label.show()
        self.toast_label.hide()

    def refresh_available_plugins(self):
        if self.is_set_up:
            changed = False
            costar_attachment_namespace = '/costar/component/ui/'
            required_plugins = []
            if rospy.has_param(costar_attachment_namespace):
                ui_list = rospy.get_param(costar_attachment_namespace)
                enable_plugins = disable_plugins = []
                # Check that a plugin is available and add it if not aleeady enabled
                required_plugins = ui_list.values()

                # rospy.logwarn('Available Plugins: ' + str(self.available_plugins))
                # rospy.logwarn('Core Plugins: ' + str(self.core_plugins))
                
                # Check which plugins to enable
                for i in required_plugins:
                    if i not in self.current_plugins:
                        if i in self.available_plugins:
                            rospy.logwarn('['+str(i)+'] is a valid plugin.')
                            self.current_plugins.append(i)
                            rospy.logwarn('Adding: ['+str(i)+']')
                        else:
                            rospy.logwarn('['+str(i)+'] is NOT a valid plugin.')
                        
                # # Check that current plugins are still valid
                # for i in self.current_plugins:
                #     if i not in required_plugins:
                #         if i not in self.core_plugins:
                #             changed = True
                #             rospy.logwarn('Removing: ['+str(i)+']')
                #             self.current_plugins.remove(i)


            ### Update Plugin List 
            # Add new plugins
            for p in self.plugins.itervalues():
                description = p['description']
                if description in self.current_plugins:
                    if description not in self.active_plugin_widgets:
                        #rospy.logwarn('Adding widget for plugin: ['+description+']')
                        changed = True
                        self.component_widgets[p['type']].add_item_to_group(p['name'],p['group'])
                        self.active_plugin_widgets.append(description)

            for c in self.component_widgets.values():
                if len(c.items) == 0:
                    c.hide()

            if changed is True:
                rospy.logwarn('Current Plugins: ' + str(self.current_plugins))
                rospy.logwarn('Required Plugins: ' + str(required_plugins))
                rospy.logerr('Warning - system changed, you probably need to create a new tree.')

    def parse_plugin_info(self):
        rospy.logwarn('INSTRUCTOR: LOADING PLUGINS')
        self.plugins = {}
        self.node_counter = {}
        plugins, plugin_descriptions, plugin_names, plugin_types, plugin_groups = load_instructor_plugins(self.cases)
        for plug,desc,name,typ,grp in zip(plugins,plugin_descriptions,plugin_names,plugin_types, plugin_groups):
            self.plugins[name] = {'module':plug, 'type':typ, 'name':name, 'group':grp, 'description':desc, 'generator_type':str(type(plug()))}
        
        self.available_plugins = plugin_descriptions
        for p in self.plugins.itervalues():
            if any(['SYSTEM' in p['group'], 'ROBOT' in p['group']]):
                self.core_plugins.append(p['description'])
                self.current_plugins.append(p['description'])

        rospy.logwarn('INSTRUCTOR: LOADING GENERATORS')
        for name,plugin in self.plugins.items():
            self.all_generators[name] = plugin['module']()

    def splitter_moved(self,pos,index):
        self.drawer.resize(360,self.container_widget.geometry().height()-100)
        self.drawer.move(self.container_widget.geometry().left()-360,50)
        self.drawer.hide()

    def window_resized(self,event):
        visible = self.drawer.isVisible()
        self.drawer.resize(360,self.container_widget.geometry().height()-100)
        self.drawer.move(self.container_widget.geometry().left()-360,50)
        if visible:
            self.drawer.show()
        else:
            self.drawer.hide()
    
    def collapse_unused(self,id):
        self.hide_menu()
        self.load_subtree_list()
        # self.load_node_list()
        self.clear_node_info()
        self.load_sub_btn.hide()
        self.remove_sub_btn.hide()
        self.remove_saved_node_btn.hide()
        self.close_drawer()
        for k,v in self.component_widgets.items():
            if k is not id:
                v.contract()
        for k,v in self.load_widgets.items():
            if k is not id:
                v.contract()

    def open_drawer(self):
        self.regenerate_btn.hide()
        if self.drawer_state == 'CLOSED':
            # x,y = self.dot_widget.get_current_pos()
            # self.dot_widget.set_current_pos(x+400,y)
            self.drawer.show()
            self.open_animation = QPropertyAnimation(self.drawer, "geometry")
            self.open_animation.setDuration(200)
            self.drawer.startGeometry = self.drawer_close_geo
            self.drawer.endGeometry = self.drawer_open_geo
            self.open_animation.setStartValue(self.drawer.startGeometry)
            self.open_animation.setEndValue(self.drawer.endGeometry)
            self.open_animation.start()
            self.drawer_state = 'OPEN'

    def close_drawer(self):
        self.selected_node_field.set_color(colors['gray_light'])
        if self.drawer_state == 'OPEN':
            # x,y = self.dot_widget.get_current_pos()
            # self.dot_widget.set_current_pos(x-400,y)
            # self.clear_node_info()
            self.clear_node_info()
            if self.current_node_plugin_name is not None:
                self.all_generators[self.current_node_plugin_name] = self.plugins[self.current_node_plugin_name]['module']()
                self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                self.current_node_type = self.plugins[self.current_node_plugin_name]['type']
                # self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                self.drawer.node_info_layout.addWidget(self.current_node_generator)
                self.right_selected_node = None

            # self.drawer.
            # if self.current_node_generator is not None:
            #     self.drawer.node_info_layout.addWidget(self.current_node_generator)
            # self.clear_node_info()
            # self.drawer.show()
            self.open_animation = QPropertyAnimation(self.drawer, "geometry")
            self.open_animation.setDuration(200)
            self.drawer.startGeometry = self.drawer_open_geo
            self.drawer.endGeometry = self.drawer_close_geo
            self.open_animation.setStartValue(self.drawer.startGeometry)
            self.open_animation.setEndValue(self.drawer.endGeometry)
            self.open_animation.start()
            self.drawer_state = 'CLOSED'
    
    def set_up_gui(self):
        ### DRAWER ###
        self.drawer.resize(360,self.container_widget.geometry().height()-100)
        self.drawer.move(self.container_widget.geometry().left()-360,50)
        self.drawer_open_geo = QRect(self.drawer.geometry())
        self.drawer_close_geo = QRect(self.drawer.geometry().x()+360,self.drawer.geometry().y(),0,self.drawer.geometry().height())
        self.drawer.hide()
        self.drawer_state = 'CLOSED'
        self.add_node_above_btn = InterfaceButton('ADD ABOVE',colors['gray_light'])
        self.add_node_above_btn.clicked.connect(self.add_sibling_before_cb)
        self.add_node_below_btn = InterfaceButton('ADD BELOW',colors['gray_light'])
        self.add_node_below_btn.clicked.connect(self.add_sibling_after_cb)
        self.add_node_child_btn = InterfaceButton('ADD',colors['green_light'])
        self.add_node_child_btn.clicked.connect(self.add_child_cb)
        self.replace_node_btn = InterfaceButton('REPLACE',colors['gray_light'])
        self.replace_node_btn.clicked.connect(self.replace_child_cb)
        self.add_node_root_btn = InterfaceButton('ADD AS ROOT',colors['green_light'])
        self.add_node_root_btn.clicked.connect(self.add_root_cb)
        self.drawer.button_layout.addWidget(self.add_node_root_btn,0,0,1,2)
        self.drawer.button_layout.addWidget(self.add_node_child_btn,1,0)
        self.drawer.button_layout.addWidget(self.replace_node_btn,1,1)
        self.drawer.button_layout.addWidget(self.add_node_above_btn,2,0)
        self.drawer.button_layout.addWidget(self.add_node_below_btn,2,1)

        self.regenerate_btn = InterfaceButton('REGENERATE',colors['green_light'])
        self.regenerate_btn.clicked.connect(self.regenerate_node)
        self.drawer.button_layout.addWidget(self.regenerate_btn,3,0,1,2)
        self.regenerate_btn.hide()
        self.add_node_cancel_btn = InterfaceButton('CLOSE',colors['red'])
        self.add_node_cancel_btn.clicked.connect(self.close_drawer)
        self.drawer.button_layout.addWidget(self.add_node_cancel_btn,4,0,1,2)

        # WAYPOINT DIALOG #
        self.waypoint_dialog = WaypointManagerDialog()
        self.waypoint_dialog.hide()
        self.waypoint_btn.clicked.connect(self.show_waypoint_manager)
        self.waypoint_dialog_saved_geom = None

        # SMARTMOVE DIALOG
        self.smartmove_dialog = SmartMoveDialog(self.show_smartmove)
        self.smartmove_button.clicked.connect(self.show_smartmove)

        # CLEAR #
        self.clear_btn = InterfaceButton('CLEAR',colors['dark_red'])
        self.clear_btn.hide()
        self.clear_cancel_btn = InterfaceButton('Cancel',colors['dark_red'])
        self.clear_btn.clicked.connect(self.clear_all)
        self.clear_cancel_btn.clicked.connect(self.clear_all_cancel)
        self.clear_confirm = False

        # SAVE BUTTONS #
        self.load_sub_btn = InterfaceButton('LOAD',colors['green'])
        self.load_sub_btn.hide()
        self.load_sub_btn.clicked.connect(self.load_selected_subtree)
        self.remove_sub_btn = InterfaceButton('REMOVE',colors['red'])
        self.remove_sub_btn.hide()
        self.remove_sub_btn.clicked.connect(self.delete_selected_subtree)
        self.remove_saved_node_btn = InterfaceButton('REMOVE',colors['red'])
        self.remove_saved_node_btn.hide()
        self.remove_saved_node_btn.clicked.connect(self.delete_selected_saved_node)
        self.save_btn = InterfaceButton('SAVE',colors['gray'])
        self.delete_btn = InterfaceButton('DELETE',colors['red'])
        self.save_btn.hide()
        self.delete_btn.hide()
        self.delete_btn.clicked.connect(self.delete_cb)
        self.button_layout.addWidget(self.save_btn)
        self.button_layout.addWidget(self.delete_btn)
        self.button_layout.addWidget(self.clear_btn)
        self.button_layout.addWidget(self.clear_cancel_btn)
        self.clear_cancel_btn.hide()

        # RUN STOP BUTTONS #
        self.run_button.setMinimumHeight(48)
        self.run_button.clicked.connect(self.run_tree)
        self.run_button.show()
        self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}''')

        # Menu
        self.menu_widget.hide()
        self.menu_button.clicked.connect(self.show_menu)
        self.menu_button.setMinimumHeight(60)
        self.menu_visible = False
        self.open_gripper_button.clicked.connect(self.open_gripper_cb)
        self.close_gripper_button.clicked.connect(self.close_gripper_cb)
        self.gripper_combo.activated[QString].connect(self.gripper_mode_cb)
        self.detect_objects_button.clicked.connect(self.detect_objects_cb)
        self.calibrate_button.clicked.connect(self.calibrate_cb)
        self.update_scene_button.clicked.connect(self.update_scene_cb)

        # CONTEXT MENU #
        self.context_popup = Popup(self)
        self.context_inspect_btn = InterfaceButton('INSPECT',colors['pink'],parent=self)
        self.context_inspect_btn.setMinimumWidth(89)
        self.context_inspect_btn.clicked.connect(self.inspect_node)
        self.context_popup.layout.addWidget(self.context_inspect_btn)
        self.context_fold_btn = InterfaceButton('FOLD',colors['sea_green'],parent=self)
        self.context_fold_btn.setMinimumWidth(89)
        self.context_fold_btn.clicked.connect(self.collapse_node)
        self.context_popup.layout.addWidget(self.context_fold_btn)
        self.context_popup.hide()

        # VIEW BUTTONS #
        self.view_fit_btn = InterfaceButton('FIT',colors['gray'],txtsz=10)
        self.view_fit_btn.clicked.connect(self.view_fit)
        self.view_layout.addWidget(self.view_fit_btn)
        self.selected_node_field = InterfaceButton('SELECTED NODE: NONE',colors['gray'],txtsz=10)
        self.view_layout.addWidget(self.selected_node_field)
        self.inspect_btn = InterfaceButton('INSPECT NODE',colors['pink'],txtsz=10)
        self.inspect_btn.clicked.connect(self.inspect_node)
        self.view_layout.addWidget(self.inspect_btn)
        self.inspect_btn.hide()
        self.view_center_btn = InterfaceButton('CENTER ON NODE',colors['gray'],txtsz=10)
        self.view_center_btn.clicked.connect(self.view_center)
        self.view_layout.addWidget(self.view_center_btn)
        self.view_center_btn.hide()

        # COMPONENT WIDGETS
        self.component_widgets = {}
        self.load_widgets = {}
        # Create Expanding Columns for Components
        for t,label,c in zip(self.types__,self.labels__,self.colors__):
            w = ListContainer(t,label,colors[c])
            w.register_callbacks(self.collapse_unused,self.component_selected_callback)
            w.show()
            self.component_widgets[t] = w
            self.component_layout.addWidget(w)

        self.subtree_container = ListContainer('SUBTREES','SUBTREES',colors['green_light'],'large')
        # self.node_container = ListContainer('NODES','NODES',colors['pink'],'large')
        self.subtree_container.show()
        self.subtree_container.register_callbacks(self.collapse_unused,self.subtree_selected_callback)
        self.subtree_container.layout.addWidget(self.load_sub_btn)
        self.subtree_container.layout.addWidget(self.remove_sub_btn)
        # self.node_container.show()
        # self.node_container.register_callbacks(self.collapse_unused,self.node_selected_callback)
        # self.node_container.layout.addWidget(self.remove_saved_node_btn)
        self.load_widgets['SUBTREES'] = self.subtree_container
        # self.load_widgets['NODES'] = self.node_container
        self.component_layout.addWidget(self.subtree_container)
        # self.component_layout.addWidget(self.node_container)

        ### DIALOG ###
        self.save_dialog = Dialog()
        self.save_dialog.move(100,100)
        self.save_dialog.hide()
        self.save_dialog.save_cancel_btn.clicked.connect(self.hide_save_dialog)
        self.save_dialog.save_subtree_btn.clicked.connect(self.save_subtree)
        self.save_dialog.save_node_btn.clicked.connect(self.save_node)
        self.save_dialog.save_name_field.textChanged.connect(self.saved_name_updated_cb)
        self.save_btn.clicked.connect(self.show_save_dialog)

        # ### Update Plugin List 
        for n in self.plugins.itervalues():
            item = n['name']
            group = n['group']
            self.component_widgets[n['type']].add_item_to_group(item,group)

        for c in self.component_widgets.values():
            if len(c.items) == 0:
                c.hide()

        ### LOAD SUBTREE LIST ###
        self.load_subtree_list()
        # self.load_node_list()

        ### FINALIZE ###
        self.is_set_up = True

    def contract_all(self):
        for w in self.component_widgets.values():
            w.contract()
        self.subtree_container.contract()

    def calibrate_cb(self):
        rospy.logwarn('Calibrating...')
        self.toast('Please wait... calibrating...')
        self.calibrate_button.setEnabled(False)
        service_name = '/calibrate'
        self.send_service_command(service_name)
        rospy.logwarn('Calibrating...DONE')
        self.calibrate_button.setEnabled(True)

    def detect_objects_cb(self):
        rospy.logwarn('Detecting objects...')
        self.toast('Please wait... Detecting Objects')
        self.detect_objects_button.setEnabled(False)
        service_name = '/SPServer/SPSegmenter'
        self.send_service_command(service_name)
        rospy.logwarn('Detecting objects...DONE')
        self.detect_objects_button.setEnabled(True)

    def update_scene_cb(self):
        rospy.logwarn('Updating Scene objects...')
        self.toast('Please wait... Updating Scene')
        self.update_scene_button.setEnabled(False)
        service_name = '/planningSceneGenerator/planningSceneGenerator'
        self.send_service_command(service_name)
        rospy.logwarn('Updating Scene...DONE')
        self.update_scene_button.setEnabled(True)

    def gripper_mode_cb(self,text):
        mode = str(text).lower()
        if mode == 'scissor':
            self.send_gripper_command('open')
            self.send_gripper_command('scissor_mode')
        elif mode == 'basic':
            self.send_gripper_command('open')
            self.send_gripper_command('basic_mode')
        elif mode == 'pinch':
            self.send_gripper_command('open')
            self.send_gripper_command('pinch_mode')
        elif mode == 'wide':
            self.send_gripper_command('open')
            self.send_gripper_command('wide_mode')

    def open_gripper_cb(self):
        self.send_gripper_command('open')
        pass

    def close_gripper_cb(self):
        self.send_gripper_command('close')
        pass

    def send_gripper_command(self,option):
        service_name = '/costar/gripper/%s'%(option)
        self.send_service_command(service_name)

    def send_service_command(self,service_name):
        # Check to see if service exists
        try:
            rospy.wait_for_service(service_name)
        except rospy.ROSException as e:
            rospy.logerr('Could not find service ['+str(e)+']')
            return
        # Make servo call to robot
        try:
            service_cmd_proxy = rospy.ServiceProxy(service_name,EmptySrv)
            result = service_cmd_proxy()
            return
        except (rospy.ServiceException), e:
            rospy.logwarn('There was a problem with the service ['+str(e)+']')
            return

    def show_menu(self):
        if self.menu_visible == False:
            self.contract_all()
            self.menu_widget.show()
            self.menu_visible = True
        else:
            self.menu_visible = False
            self.menu_widget.hide()

    def hide_menu(self):
        self.menu_visible = False
        self.menu_widget.hide()

    def show_smartmove(self):
        if self.smartmove_dialog.isVisible():
            self.smartmove_dialog.saved_geom = self.smartmove_dialog.geometry()
            self.smartmove_dialog.hide()
        else:
            if self.smartmove_dialog.saved_geom is not None:
                self.smartmove_dialog.move(self.smartmove_dialog.saved_geom.x(),self.smartmove_dialog.saved_geom.y())
            else:
                self.smartmove_dialog.move(self.geometry().x()+self.geometry().width()/2-self.smartmove_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.smartmove_dialog.geometry().height()/2)
            self.smartmove_dialog.show()
            self.smartmove_dialog.update_all()

    def show_waypoint_manager(self):
        if self.waypoint_dialog.isVisible():
            self.waypoint_dialog_saved_geom = self.waypoint_dialog.geometry()
            self.waypoint_dialog.hide()
        else:
            if self.waypoint_dialog_saved_geom is not None:
                self.waypoint_dialog.move(self.waypoint_dialog_saved_geom.x(),self.waypoint_dialog_saved_geom.y())
            else:
                self.waypoint_dialog.move(self.geometry().x()+self.geometry().width()/2-self.waypoint_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.waypoint_dialog.geometry().height()/2)
            self.waypoint_dialog.show()

    def load_subtree_list(self):
        self.subtree_list = self.lib_list_service('instructor_subtree').entries
        self.subtree_container.remove_all()
        if len(self.subtree_list) > 0: # There are actually nodes to load
            for st in self.subtree_list:
                self.subtree_container.add_item(str(st))

        self.subtree_container.sort()

    # def load_node_list(self):
    #     generator_list = self.lib_list_service('instructor_node').entries
    #     self.node_container.remove_all()
    #     if len(generator_list) > 0: # There are actually nodes to load
    #         self.loadable_nodes = {}
    #         for g in generator_list:
    #             data = yaml.load(self.lib_load_service(id=g,type='instructor_node').text)
    #             self.loadable_nodes[data['name']] = data
    #         for n in self.loadable_nodes.itervalues():
    #             self.node_container.add_item(n['name'])

    def view_fit(self):
        if self.root_node is not None:
            self.dot_widget.zoom_to_fit()

    def view_center_default(self):
        x = 0.0
        y = 0.0
        jump = self.dot_widget.get_jump(x,y)
        if jump is not None:
            self.dot_widget.animate_to(jump.x,jump.y)

    def view_center(self):
        x = self.selected_location[0]
        y = self.selected_location[1]
        jump = self.dot_widget.get_jump(x,y)
        if jump is not None:
            self.dot_widget.animate_to(jump.x,jump.y)

    def show_save_dialog(self):
        self.save_dialog.move(self.geometry().x()+self.geometry().width()/2-self.save_dialog.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.save_dialog.geometry().height()/2)
        self.save_dialog.show()
        self.save_dialog.label.setText('To save the selected node, click SAVE NODE.\nTo save the subtree starting with the selected node, click SAVE SUBTREE.')

    def hide_save_dialog(self):
        self.save_dialog.hide()
        pass

    def collapse_node(self):
        if self.right_selected_node != None:
            val = self.right_selected_node
        else:
            val = self.left_selected_node

        node = self.current_tree[val]
        if self.current_node_types[val] == 'LOGIC': 
            if node.collapsed == True:
                node.set_collapsed(False)
                self.regenerate_tree()
                self.context_fold_btn.setText('FOLD')
            elif node.collapsed == False:
                node.set_collapsed(True)
                self.regenerate_tree()
                self.context_fold_btn.setText('UNFOLD')
            self.context_popup.hide()
        
# Show and Hide Info
    def show_info_cb(self,check):
        if check == Qt.Checked:
            for node in self.current_tree.itervalues():
                node.set_alt_view(True)
            self.regenerate_tree()

        if check == Qt.Unchecked:
            for node in self.current_tree.itervalues():
                node.set_alt_view(False)
            self.regenerate_tree()

# Run and Stop -----------------------------------------------------------------
    def servo_button_cb(self):
        if self.running__ == True:
            self.stop_tree()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.toast('ABORTED: Robot is no longer in servo')
            
    def run_tree(self):
         #rospy.logwarn(self.robot_.driver_status)
         if 'servo' not in str(self.robot_.driver_status).lower():
             self.toast('Robot is NOT in SERVO MODE')
             return 
         elif self.root_node == None:
             self.toast('There is no root node') 
             return
         else:
            if self.running__ == True:
                self.stop_tree()
                self.robot_.stop_servo()
                self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
                self.run_button.setText('EXECUTE PLAN')
            else:
                self.root_node.reset()
                self.sound_pub.publish(String("notify_4"))
                self.running__ = True
                self.run_timer_.start(5)
                rospy.logwarn('INSTRUCTOR: Task Tree STARTING')
                self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #F62459;border-radius: 0px;background-color: #F62459;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #F62459;border-radius: 0px;background-color: #F62459;color:#ffffff}''')
                self.run_button.setText('STOP EXECUTION')


    def run(self):
        result = self.root_node.execute()
        #rospy.logwarn(result)
        # self.regenerate_tree()
        if result[:7] == 'SUCCESS':
            rospy.loginfo('INSTRUCTOR: Task Tree FINISHED WITH SUCCESS: %s'%result)
            self.sound_pub.publish(String("notify_4_succeed"))
            self.run_timer_.stop()
            self.running__ = False
            self.root_node.reset()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.regenerate_tree()
        elif result[:7] == 'FAILURE':
            rospy.logerr('INSTRUCTOR: Task Tree FINISHED WITH FAILURE: %s'%result)
            self.run_timer_.stop()
            self.running__ = False
            # self.root_node.reset()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.regenerate_tree()
            rospy.sleep(.5)
            self.sound_pub.publish(String("notify_4_fail"))
        elif result == 'NODE_ERROR':
            rospy.logerr('INSTRUCTOR: Task Tree ERROR')
            self.run_timer_.stop()
            self.running__ = False
            # self.root_node.reset()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #3FC380;border-radius: 0px;background-color: #3FC380;color:#ffffff}''')
            self.run_button.setText('EXECUTE PLAN')
            self.regenerate_tree()
        elif result == 'RUNNING':
            self.regenerate_tree(True)
            pass

    def stop_tree(self):
        rospy.logwarn('INSTRUCTOR: Task Tree STOPPED')
        self.run_timer_.stop()
        self.running__ = False
        # self.root_node.reset()
        self.regenerate_tree()

# Save and Load Nodes ----------------------------------------------------------
    def save_node(self):
        if self.right_selected_node != None:
            if type(self.current_node_generator.generate()) == str:
                rospy.logerr('The node must be fully defined to save it.')
            else:
                print ''
                generator_to_save = {'node_type':self.current_node_type, 'name':self.save_name, 'plugin_name':self.current_node_plugin_name, 'generator_info':self.current_node_generator.save()}
                D = yaml.dump(generator_to_save)
                print D
                print self.lib_save_service(id=self.save_name,type='instructor_node',text=D)
                self.hide_save_dialog()

        elif self.current_node_type != None:
            if type(self.current_node_generator.generate()) == str:
                rospy.logerr('The node must be fully defined to save it.')
            else:
                print ''
                generator_to_save = {'node_type':self.current_node_type, 'name':self.save_name, 'plugin_name':self.current_node_plugin_name, 'generator_info':self.current_node_generator.save()}
                D = yaml.dump(generator_to_save)
                print D
                print self.lib_save_service(id=self.save_name,type='instructor_node',text=D)
                self.hide_save_dialog()

    def load_node_info(self):
        print 'Attempting to load node'
        try:
            self.open_drawer()
            node_data = self.loadable_nodes[self.selected_load_node]
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                print 'Node to load matches known nodes ['+ node_plugin_name +']'
                self.clear_node_info()
                self.current_node_generator = self.plugins[node_plugin_name]['module']()
                self.current_node_type = self.plugins[node_plugin_name]['type']
                self.current_node_plugin_name = node_plugin_name
                rospy.logwarn(self.current_node_generator.get_name())
                self.drawer.node_info_layout.addWidget(self.current_node_generator)
                # Add in parameters from saved file
                self.current_node_generator.load(node_data['generator_info'])
                self.current_node_generator.name.set_field(self.increment_node_name(node_plugin_name))
                # Close gui

        except KeyError as e:
            print "Not a valid node"

    def node_selected_callback(self,val):
        self.remove_saved_node_btn.show()
        self.selected_load_node = val
        self.load_node_info()

    def delete_selected_saved_node(self):
        if self.selected_load_node != None:
            self.lib_delete_service(id=self.selected_load_node,type='instructor_node')
            # self.load_node_list()

# Save and Load Subtrees -------------------------------------------------------

    def saved_name_updated_cb(self,t):
        self.save_name = str(t)
        pass

    def save_subtree(self):
        if self.left_selected_node == None:
            rospy.logerr('There is no head node selected to save')
        else:
            if self.save_name != None:
                tree = self.walk_tree(self.current_tree[self.left_selected_node]) # this should start from the selected node
                # tree = self.walk_tree(self.root_node) #this will always start from root
                D = yaml.dump({'name':self.save_name,'tree':tree})
                rospy.logwarn('SAVING SUBTREE to %s'%self.save_name)
                print self.lib_save_service(id=self.save_name,type='instructor_subtree',text=D)
                # Hide on successful save    
                #self.subtree_save_widget.hide()
                self.hide_save_dialog()
                self.load_subtree_list()
            else:
                rospy.logerr('You must enter a name to save the subtree')

    def walk_tree(self,node):
        t = [self.walk_tree(C) for C in node.children_]

        # Remove invalid children.
        # TODO: figure out why this happens and make it not happen any more.
        t = [st for st in t if st is not None]

        if node.name_ not in self.current_plugin_names:
            rospy.logerr("Invalid child name: %s"%node.name_)
            return None

        # Generate Info
        generator = self.all_generators[self.current_plugin_names[node.name_]]
        generator.load(self.current_node_info[node.name_])
        # generator = self.current_generators[node.name_]
        node_type = self.current_node_types[node.name_]
        plugin_name = self.current_plugin_names[node.name_]
        generator_to_save = {'node_type':node_type, 'name':node.name_, 'plugin_name':plugin_name, 'generator_info':generator.save()}

        return {'name':node.name_, 'save_info':generator_to_save, 'children':t}

    def subtree_selected_callback(self,val):
        # self.selected_subtree = str(self.subtree_load_model.itemFromIndex(val).text())
        self.selected_subtree = val
        self.load_sub_btn.show()
        self.remove_sub_btn.show()
        self.load_sub_btn.setText('LOAD [ '+val.upper()+' ]')
        self.remove_sub_btn.setText('REMOVE [ '+val.upper()+' ]')

        self.selected_subtree_data = yaml.load(self.lib_load_service(id=self.selected_subtree,type='instructor_subtree').text)
        selected_subtree_root_name = self.selected_subtree_data['tree']['save_info']['plugin_name']
        print selected_subtree_root_name
        if 'root' in selected_subtree_root_name.lower():
            print 'You have selected a tree with a root node.  If a node is currently selected, this subtree will be added as a child tree of the selected node.  If the graph is empty, this subtree will be added as the entire tree.'
        else:
            print 'The selected subtree has no root node.  If a node is currently selected, this subtree will be added as a child tree.  If no node is selected, the subtree will be added along with a root node.'

    def delete_selected_subtree(self):
        if self.selected_subtree != None:
            self.lib_delete_service(id=self.selected_subtree,type='instructor_subtree')
            self.load_subtree_list()

    def load_selected_subtree(self):
        if self.selected_subtree != None:
            clear_cmd()
            print 'loading subtree'
            if self.root_node != None: # There is an existing tree, with a root node
                rospy.logwarn('Loading as subtree...')
                if self.left_selected_node == None:
                    rospy.logwarn('You must select a node as a parent for the subtree')
                else:
                    # Test to see if the subtree has a root...
                    node_plugin_name = self.selected_subtree_data['tree']['save_info']['plugin_name']
                    if node_plugin_name == 'Root':
                        rospy.logwarn('Found root in subtree... removing')
                        start_point = self.selected_subtree_data['tree']['children'][0]
                    else:
                        rospy.logwarn('No root, starting with '+node_plugin_name)
                        start_point = self.selected_subtree_data['tree']
                    try:
                        self.recursive_add_nodes(start_point,self.current_tree[self.left_selected_node])
                        self.regenerate_tree(center=True)
                    except Exception as e:
                        rospy.logerr('There was a problem loading the tree, most likely a node that didnt generate properly')
                        rospy.logerr(e)

            else: # The tree is empty
                rospy.logwarn('Loading as full tree...')
                try:
                    # Test to see if the subtree has a root...
                    node_plugin_name = self.selected_subtree_data['tree']['save_info']['plugin_name']
                    if node_plugin_name != 'Root':
                        # No root, generate one...
                        self.generate_root()
                        self.recursive_add_nodes(self.selected_subtree_data['tree'], self.root_node)
                        self.regenerate_tree(center=True)
                    else:
                        self.recursive_add_nodes(self.selected_subtree_data['tree'],None)
                        self.regenerate_tree(center=True)
                except Exception as e:
                    rospy.logerr('There was a problem loading the tree, most likely a node that didnt generate properly')
                    rospy.logerr(e)

            # Finish
            self.load_sub_btn.hide()
            self.remove_sub_btn.hide()
            self.selected_subtree = None
            self.view_fit()
        else:
            rospy.logwarn('You must select a subtree to load')

    def recursive_add_nodes(self,info,parent):
        if parent == None:
            node_data = info['save_info']
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                node_type = self.plugins[node_plugin_name]['type']
                plugin_name = node_plugin_name
                # Add in parameters from saved file
                self.all_generators[node_plugin_name].load(node_data['generator_info'])
                # Generate Node
                node_to_add = self.all_generators[node_plugin_name].generate()
                current_name = self.all_generators[node_plugin_name].get_name()
                # Add node
                self.current_tree[current_name] = node_to_add
                self.current_node_info[current_name] = self.all_generators[node_plugin_name].save()
                self.current_plugin_names[current_name] = plugin_name
                self.current_node_types[current_name] = node_type
                self.root_node = node_to_add
            else:
                rospy.logwarn('node plugin undefined')
            # recusively call child nodes
            for C in info['children']:
                self.recursive_add_nodes(C,node_to_add)
        else: 
            node_data = info['save_info']
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                node_type = self.plugins[node_plugin_name]['type']
                plugin_name = node_plugin_name
                self.all_generators[node_plugin_name].load(node_data['generator_info'])
                # Generate Child
                self.all_generators[node_plugin_name].name.set_field(self.increment_node_name(node_plugin_name))
                node_to_add = self.all_generators[node_plugin_name].generate()
                current_name = self.all_generators[node_plugin_name].get_name()
                self.current_tree[current_name] = node_to_add
                self.current_tree[parent.name_].add_child(node_to_add)
                self.current_node_info[current_name] = self.all_generators[node_plugin_name].save()
                self.current_plugin_names[current_name] = plugin_name
                self.current_node_types[current_name] = node_type
            else:
                rospy.logwarn(node_plugin_name)
                rospy.logwarn('node plugin undefined')
            # recusively call child nodes
            for C in info['children']:
                self.recursive_add_nodes(C,node_to_add)

# Callbacks --------------------------------------------------------------------
    def increment_node_name(self,name):
        # rospy.logwarn('incrementing ['+name+']')
        if 'root' not in name.lower():
            if self.node_counter.has_key(name):
                self.node_counter[name]+=1
                num = self.node_counter[name]
            else:
                num = 0
                self.node_counter[name] = num
            auto_name = str(str(name)+'_'+str(num)).replace(' ','_').lower()
            return auto_name
        else:
            return 'root'
    
    def component_selected_callback(self,name):
        rospy.logwarn('Selected node with type [' + str(self.plugins[name]['type']) + '] from list.')
        self.open_drawer()
        # if self.plugins[name]['type'] == 'LOGIC':
        #     self.add_node_root_btn.show()
        # else:
        #     self.add_node_root_btn.hide()

        if self.plugins[name]['name'] == 'Root':
            self.add_node_root_btn.show()
            self.add_node_child_btn.hide()
            self.replace_node_btn.hide()
            self.add_node_above_btn.hide()
            self.add_node_below_btn.hide()
        else:
            self.add_node_root_btn.hide()
            self.add_node_child_btn.show()
            self.replace_node_btn.show()
            self.add_node_above_btn.show()
            self.add_node_below_btn.show()

        if self.plugins.has_key(name):
            self.clear_node_info()
            self.all_generators[name] = self.plugins[name]['module']()
            self.current_node_generator = self.all_generators[name]
            self.current_node_type = self.plugins[name]['type']
            self.current_node_plugin_name = name
            self.current_node_generator.name.set_field(self.increment_node_name(name))
            self.drawer.node_info_layout.addWidget(self.current_node_generator)
            self.right_selected_node = None
        else:
            self.clear_node_info()

    def node_leftclick_location_cb(self,event):
        self.selected_location = event
        pass

    def node_leftclick_cb(self,event):
        # rospy.logwarn(event)
        if event == 'none':
            # Update Context Menu
            self.context_popup.hide()
            if self.root_node is not None:
                self.root_node.reset()
            # Update Selected Node
            self.left_selected_node = None
            self.selected_node_field.setText('SELECTED NODE: NONE')
            self.selected_node_field.set_color(colors['gray_light'])
            self.inspect_btn.hide()
            self.view_center_btn.hide()
            # self.run_button.hide()
            self.load_sub_btn.hide()
            self.remove_sub_btn.hide()
            self.remove_saved_node_btn.hide()
            self.selected_subtree = None
            self.save_btn.hide()
            self.delete_btn.hide()
            self.clear_btn.hide()
            # Remove all highlights
            if len(self.current_tree.keys()) != 0:
                self.remove_node_highlights()
                self.regenerate_tree()
            self.clear_all_cancel()
            self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}QPushButton#run_button:pressed{border: 2px solid #333333;border-radius: 0px;background-color: #333333;color:#ffffff}''')
        else:
            if event in self.current_tree:
                self.left_selected_node = event
                # Update Context Menu
                self.context_popup.hide()
                # Update Selected Node Field
                self.inspect_btn.show()
                self.view_center_btn.show()
                self.save_btn.show()
                self.delete_btn.show()
                self.selected_node_field.setText(str(event).upper())
                self.selected_node_field.set_color(colors['green'])
                if self.current_node_types[event] == 'KNOLWEDGE':
                    self.selected_node_field.set_color(colors['purple'])
                if self.current_node_types[event] == 'LOGIC':
                    self.selected_node_field.set_color(colors['blue'])
                    if 'root' in event.lower():
                        self.run_button.setStyleSheet('''QPushButton#run_button{border: 2px solid #3FC380;
border-radius: 0px;
background-color: #3FC380;
color:#ffffff}
QPushButton#run_button:pressed{
border: 2px solid #3FC380;
border-radius: 0px;
background-color: #3FC380;
color:#ffffff}''')
                        self.clear_btn.show()
                        # self.run_button.show()
                        pass
                    else:
                        # self.run_button.hide()
                        pass
                else:
                    self.save_btn.hide()

                # Remove all highlights
                self.remove_node_highlights()
                # highlight the selected node
                self.current_tree[str(event)].set_flag(True)
                self.regenerate_tree()
                self.right_selected_node = None
            else:
                print '[' + event + '] NOT found in current tree...'

    def remove_node_highlights(self):
        for n in self.current_tree.itervalues():
            if n.flag_ == True:
                n.set_flag(False)

    def node_rightclick_cb(self,event):
        # Get Click Interface Pose
        # experimental
        self.left_selected_node = None
        # experimental
        currentPos = self.mapFromGlobal(QCursor.pos())
        rospy.logwarn(currentPos)
        self.context_popup.move(currentPos.x()-90,currentPos.y()-70)

        if event == 'none':
            self.right_selected_node = None
            self.selected_node_field.setText('SELECTED NODE: NONE')
            self.selected_node_field.set_color(colors['gray_light'])
            self.inspect_btn.hide()
            self.view_center_btn.hide()
            self.save_btn.hide()
            self.delete_btn.hide()
            # Update Context Menu
            self.context_popup.hide()
            self.close_drawer()
        else:
            if event in self.current_tree:
                # Update Context Menu
                self.context_popup.show()
                if self.current_node_types[event] == 'LOGIC':
                    self.context_fold_btn.show()
                    if self.current_tree[str(event)].collapsed == False:
                        self.context_fold_btn.setText('FOLD')
                    else:
                        self.context_fold_btn.setText('UNFOLD')
                else:
                    self.context_fold_btn.hide()
                # Update Inspected Node Field
                self.selected_node_field.setText(str(event).upper())
                self.selected_node_field.set_color(colors['highlight_red'])
                self.inspect_btn.show()
                self.right_selected_node = event
                
            else:
                print '[' + event + '] NOT found in current tree...'

    def inspect_node(self):
        self.selected_node_field.set_color(colors['pink'])
        self.clear_node_info()
        self.open_drawer()
        if self.left_selected_node:
            self.current_node_generator = self.all_generators[self.current_plugin_names[self.left_selected_node]]
        else:
            self.current_node_generator = self.all_generators[self.current_plugin_names[self.right_selected_node]]
        self.current_node_generator.name.set_read_only(True)
        if self.left_selected_node:
            self.current_node_generator.load(self.current_node_info[self.left_selected_node])
            self.current_node_type = self.current_node_types[self.left_selected_node]
            self.current_node_plugin_name = self.current_plugin_names[self.left_selected_node]
        else:    
            self.current_node_generator.load(self.current_node_info[self.right_selected_node])
            self.current_node_type = self.current_node_types[self.right_selected_node]
            self.current_node_plugin_name = self.current_plugin_names[self.right_selected_node]
        self.drawer.node_info_layout.addWidget(self.current_node_generator)
        self.regenerate_btn.show()
        self.context_popup.hide()
        # self.save_state()
        pass

    def regenerate_node(self):
        if self.left_selected_node:
            current_name = self.left_selected_node
        else:
            current_name = self.right_selected_node

        if current_name:
            # current_name = self.right_selected_node
            replacement_node = self.current_node_generator.generate()
            if type(replacement_node) == str:
                rospy.logerr(str(replacement_node))
            else:
                current_child = self.current_tree[current_name]
                current_parent = self.current_tree[current_name].get_parent()
                if current_parent.replace_child(current_child, replacement_node):
                    self.current_tree[current_name] = replacement_node
                    self.current_node_generator.name.set_read_only(False)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.regenerate_tree()
                    self.regenerate_btn.hide()
                    self.close_drawer()
                    self.inspect_btn.hide()
                    self.view_center_btn.hide()
                    # self.save_state()
                else:
                    rospy.logwarn('Child to replace was not a member of the parent. Something went wrong.')
        
# Add and delete nodes ---------------------------------------------------------
    # This is a little brittle because it assumes that the name of the root plugin 
    # will not change... which I think is ok considering it is a core component
    def generate_root(self):
        rospy.logwarn('Generating new ROOT node')
        self.all_generators['Root'] = self.plugins['Root']['module']()
        self.current_node_generator = self.all_generators['Root']
        self.current_node_type = self.plugins['Root']['type']
        self.current_node_plugin_name = 'Root'
        node_to_add = self.current_node_generator.generate()
        if type(node_to_add) == str:
            rospy.logerr(str(node_to_add))
        else:
            current_name = self.current_node_generator.get_name()
            self.current_tree[current_name] = node_to_add
            self.current_node_info[current_name] = self.current_node_generator.save()
            self.current_plugin_names[current_name] = self.current_node_plugin_name
            self.current_node_types[current_name] = self.current_node_type
            self.root_node = node_to_add
            self.regenerate_tree()

    def add_root_cb(self):
        rospy.logwarn('adding ROOT node')
        if self.current_node_type != None:
            if self.root_node == None:
                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.root_node = node_to_add
                    self.regenerate_tree()
                    self.close_drawer()
                    # self.save_state()
                    self.node_leftclick_cb(current_name)
            else:
                rospy.logwarn('THERE IS ALREADY A ROOT NODE')
        else:
            rospy.logwarn('NO NODE SELECTED')


    
    def add_child_cb(self):
        if self.current_node_type != None:
            rospy.logwarn('adding node of type ' + str(self.current_node_type))
            if self.left_selected_node == None:
                rospy.logerr('There is no parent node selected')
            else:

                if not self.current_node_types[self.left_selected_node] == 'LOGIC':
                    rospy.logwarn('Parent must be a logic node')
                    return

                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    self.current_tree[self.left_selected_node].add_child(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    ## debugging
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.regenerate_tree()
                    # Prime GUI for another node of the same type
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                    self.drawer.node_info_layout.addWidget(self.current_node_generator)
                    # self.save_state()
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

    def add_sibling_before_cb(self):
        print 'adding sibling node of type ' + self.current_node_type
        if self.current_node_type != None:
            if self.left_selected_node == None:
              rospy.logerr('There is no left sibling node selected')
            else:
                parent_node_name = self.current_tree[self.left_selected_node].get_parent().get_node_name()
                if not self.plugins[parent_node_name]['type'] == 'LOGIC':
                    rospy.logwarn('Parent must be a logic node')
                    return

                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    sibling_node = self.current_tree[self.left_selected_node]
                    sibling_node.add_sibling_before(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type 
                    self.regenerate_tree()
                    # Prime GUI for another node of the same type
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                    self.drawer.node_info_layout.addWidget(self.current_node_generator)
                    # self.save_state()
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

    def replace_child_cb(self):
        if self.left_selected_node != None:
            current_name = self.left_selected_node
            rospy.logwarn(current_name)
            new_name = self.current_node_generator.get_name()
            rospy.logwarn(new_name)
            replacement_node = self.current_node_generator.generate()
            # current_name = self.current_node_generator.get_name()
            current_child = self.current_tree[current_name]
            current_parent = self.current_tree[current_name].get_parent()
            if current_parent.replace_child(current_child, replacement_node):
                self.current_tree[new_name] = replacement_node
                self.current_node_info[new_name] = self.current_node_generator.save()
                self.current_plugin_names[new_name] = self.current_node_plugin_name
                self.current_node_types[new_name] = self.current_node_type
                if self.current_node_types[new_name] == 'LOGIC':
                    self.delete_node(current_name,keep_children=True)
                else:
                    self.delete_node(current_name,keep_children=False)
                self.regenerate_tree()
                self.close_drawer()
                # self.save_state()
                if current_name in self.current_node_types:   
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

            else:
                rospy.logerr('Replacing node failed: '+ current_name + ' , '+ new_name)

    def add_sibling_after_cb(self):
        print 'adding sibling node of type ' + self.current_node_type
        if self.current_node_type != None:
            if self.left_selected_node == None:
              rospy.logerr('There is no left sibling node selected')
            else:
                parent_node_name = self.current_tree[self.left_selected_node].get_parent().get_node_name()
                if not self.plugins[parent_node_name]['type'] == 'LOGIC':
                    rospy.logwarn('Parent must be a logic node')
                    return

                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    sibling_node = self.current_tree[self.left_selected_node]
                    sibling_node.add_sibling_after(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type 
                    self.regenerate_tree()
                    # Prime GUI for another node of the same type
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    self.current_node_generator.name.set_field(self.increment_node_name(self.current_node_plugin_name))
                    self.drawer.node_info_layout.addWidget(self.current_node_generator)
                    # self.save_state()
                    if self.current_node_types[current_name] == 'LOGIC':
                        self.node_leftclick_cb(current_name)

    def clear_all(self):
        if self.clear_confirm == False:
            self.clear_btn.setText('CONFIRM CLEAR ?')
            self.clear_cancel_btn.show()
            self.clear_confirm = True
            self.clear_btn.set_color(colors['orange'])
        else:
            self.clear_btn.setText('CLEAR')
            self.clear_confirm = False 
            self.clear_btn.set_color(colors['dark_red'])
            self.clear_cancel_btn.hide()   
            # Stop Execution if running
            if self.root_node is not None:
                self.dot_widget.zoom_image(1, center=True)
                self.stop_tree()

                # self.root_node = None

                self.dot_widget.set_dotcode('digraph behavior_tree {}')
                self.clear_tree()
                # Update Context Menu
                self.context_popup.hide()
                # Update Selected Node
                self.left_selected_node = None
                self.selected_node_field.setText('SELECTED NODE: NONE')
                self.selected_node_field.set_color(colors['gray_light'])
                self.inspect_btn.hide()
                self.view_center_btn.hide()
                # self.run_button.hide()
                self.load_sub_btn.hide()
                self.remove_sub_btn.hide()
                self.remove_saved_node_btn.hide()
                self.selected_subtree = None

    def clear_all_cancel(self):
        self.clear_cancel_btn.hide()
        self.clear_btn.setText('CLEAR')
        self.clear_confirm = False 
        self.clear_btn.set_color(colors['dark_red']) 


    def clear_tree(self):
        self.current_tree.clear()
        self.current_generators.clear()
        self.current_node_types.clear()
        self.current_plugin_names.clear()       
        self.root_node = None
        # self.save_state()

    def clear_node(self,name):
        self.current_tree.pop(name)
        self.current_node_info.pop(name)
        self.current_node_types.pop(name)
        self.current_plugin_names.pop(name)
        # self.save_state()

    def delete_cb(self):
        if self.root_node == None:
            # self.clear_tree()
            # self.dot_widget.set_dotcode('digraph behavior_tree {}') 
            pass
        else:
            if self.left_selected_node == None:
                rospy.logerr('There is no node selected in the gui')
                return
            else:
                self.delete_node(self.left_selected_node)
            self.left_selected_node = None

    def delete_node(self,name,keep_children=False):
        node = self.current_tree[name]
        if keep_children == False:
            for child in node.get_child_names():
                self.delete_node(child,keep_children)
        if node == self.root_node:
            # self.root_node = None
            self.dot_widget.set_dotcode('digraph behavior_tree {}')
            self.clear_tree()
        else:
            node.remove_self()
            self.clear_node(name)
            self.regenerate_tree()
        rospy.logwarn('Removed node ['+name+']')

    def regenerate_tree(self,runtime=False,center=False):
        if self.root_node == None:
            self.dot_widget.set_dotcode('digraph behavior_tree {}')
        else:
            #rospy.logwarn(self.root_node.generate_dot(runtime))
            self.dot_widget.set_dotcode(self.root_node.generate_dot(runtime),False)
        # rospy.logwarn(self.current_tree)

# UI Specific Functions --------------------------------------------------------
    def clear_node_info(self):
        for i in reversed(range(self.drawer.node_info_layout.count())): 
            item = self.drawer.node_info_layout.takeAt(i)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

    def keyPressEvent(self, event):
        if type(event) == QtGui.QKeyEvent:
            if event.key() == 16777220:
                if self.selected_subtree != None:
                    self.load_selected_subtree()
                elif 'root' in self.current_node_plugin_name.lower():
                    self.add_root_cb()
                else:
                    self.add_child_cb()
            elif event.key() == 16777223:
                self.delete_cb()
            elif event.key() == 16777219:
                self.delete_cb()
            elif event.key() == 32: # space
                self.view_fit()
            elif event.key() == 70: # "f"
                # self.view_center()
                self.collapse_node()
            elif event.key() == 87: # "w"
                self.show_waypoint_manager()
            elif event.key() == 16777216: # "esc"
                self.close_drawer()
            else:
                # rospy.logwarn(event.key())
                pass
            event.accept()
        else:
            event.ignore()
    
# App Specific Functions --------------------------------------------------------------
    def closeEvent(self, event):
        print 'saving info'
        self.settings.setValue('size', self.size())
        self.settings.setValue('pos', self.pos())
        self.settings.sync()
        event.accept()

    def check_ok(self):
        self.update()
        self.robot_.update_status()
        if rospy.is_shutdown():
          self.close()
          self.app_.exit()

# MAIN #######################################################
if __name__ == '__main__':
  rospy.init_node('beetree',anonymous=True)
  app = QApplication(sys.argv)
  wrapper = Instructor(app)
  # Running
  app.exec_()
  # Done








