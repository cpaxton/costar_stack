#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
import rospkg
from beetree import *
import instructor_core
import os,sys, inspect, ast
from std_msgs.msg import *
import rospkg
# Using roslib.rospack even though it is deprecated
from roslib import rospack
import yaml
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import time
from copy import deepcopy

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

def clear_cmd():
    os.system(['clear','cls'][os.name == 'nt'])

def load_instructor_plugins():
    # Using roslib.rospack even though it is deprecated, because this function in rospkg is broken
    to_check = rospack.rospack_depends_on_1('beetree')
    rp = rospkg.RosPack()
    # to_check = rospack.get_depends_on('beetree', implicit=False)
    clear_cmd()
    print 'Found packages that have beetree dependency...'
    # print to_check
    plugins = []    
    descriptions = []
    names = []
    types = []
    for pkg in to_check:
        m = rp.get_manifest(pkg)
        p_modules = m.get_export('instructor', 'plugin')
        p_types = m.get_export('instructor', 'type')
        p_descriptions = m.get_export('instructor', 'description')
        p_names = m.get_export('instructor', 'name')
        # p_descriptions = manifest.get_export('rcommander', 'tab')
        if not p_modules:
            continue
        # print '- Package ['+pkg+'] has plugins:'
        if p_modules == []:
            pass
          # print '--- NONE'

        for p_module, p_description, p_name,p_type in zip(p_modules,p_descriptions,p_names,p_types):
            # print '--- ' + p_module
            # print '---- DESCRIPTION: ' + p_description
            # print '---- NAME: ' + p_name
            # print '---- TYPE: ' + p_type
            # try:
            roslib.load_manifest(pkg)
            package = __import__(pkg)
            sub_mod = p_module.split('.')[1:][0]
            # print sub_mod
            module = getattr(package, sub_mod)
            plugins.append(module)
            descriptions.append(p_description)
            names.append(p_name)               
            types.append(p_type)                

    return plugins, descriptions, names, types

class Instructor(QWidget):
    def __init__(self,app):
        super(Instructor,self).__init__()
        rospy.logwarn('INSTRUCTOR: STARTING UP...')
        self.app_ = app
        # self.setMinimumWidth(700)
        # self.setMinimumHeight(500)
        self.types__ = ['LOGIC', 'ACTION', 'SERVICE', 'CONDITION', 'QUERY', 'PROCESS', 'VARIABLE']
        # Load the ui attributes into the main widget
        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_core') + '/ui/main_alt.ui'
        uic.loadUi(ui_path, self)
        # Create the Graph Visualization Pane
        self.dot_widget = DotWidget()
        self.instructor_layout.addWidget(self.dot_widget)    
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

        # Load Settings
        self.settings = QSettings('settings.ini', QSettings.IniFormat)
        self.settings.setFallbacksEnabled(False) 
        self.resize( self.settings.value('size', QSize(800, 800), type=QSize) )
        self.move(self.settings.value('pos', QPoint(50, 50), type=QPoint))
        # self.showMaximized()

        # Set up Behavior Tree structure
        self.current_tree = {}
        self.current_generators = {}
        self.all_generators = {}
        
        self.current_node_info ={}
        self.current_node_generator = None

        self.current_plugin_names = {}
        self.current_node_types = {}
        self.root_node = None
        self.gui_selected_node = None
        self.selected_node_field.setText('NONE')
        # Get known Beetree Builder Node Plugins
        self.parse_plugin_info()
        # Set up the gui with plugin information
        self.set_up_gui()
        # Set up communication between node view and the rest of the app
        self.connect(self.dot_widget,SIGNAL("clicked"), self.node_gui_selected_cb)
        self.connect(self.dot_widget,SIGNAL("right_clicked"), self.node_gui_alt_selected_cb)

        # Set up librarian
        self.set_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.set_type_service('instructor_node')
        self.set_type_service('instructor_subtree')

    def parse_plugin_info(self):
        rospy.logwarn('INSTRUCTOR: LOADING PLUGINS')
        self.plugins = {}
        plugins, plugin_descriptions, plugin_names, plugin_types = load_instructor_plugins()
        for plug,desc,name,typ in zip(plugins,plugin_descriptions,plugin_names,plugin_types):
            self.plugins[name] = {'module':plug, 'type':typ, 'name':name, 'description':desc, 'generator_type':str(type(plug()))}
        
        rospy.logwarn('INSTRUCTOR: LOADING GENERATORS')
        for name,plugin in self.plugins.items():
            self.all_generators[name] = plugin['module']()
            rospy.logwarn('... '+str(name))

    def set_up_gui(self):
        self.selected_node_label = None
        self.active_node_type = None
        # Create node list 
        self.node_model = QStandardItemModel()
        self.node_model.setHorizontalHeaderLabels(['Name', 'Description'])
        self.node_create_tree_view.setModel(self.node_model)
        self.node_create_tree_view.clicked.connect(self.selected_list_node_cb)
        self.node_create_tree_view.expanded.connect(self.node_expanded_cb)
        # Set up groups for standard types
        type_items = {t:QStandardItem(t + ' NODES') for t in self.types__}
        # Load in nodes
        for n in self.plugins.itervalues():
            item = QStandardItem(n['name'])
            type_items[n['type']].appendRow(item)
        for row in type_items.itervalues():
            self.node_model.appendRow(row)
        self.node_create_tree_view.resizeColumnToContents(0)

        # Connect Buttons in GUI for Adding Nodes
        self.add_node_root_btn.clicked.connect(self.add_root_cb)
        
        self.add_node_child_btn.clicked.connect(self.add_child_cb)
        self.add_node_child_btn.hide()
        
        self.add_node_sibling_btn.clicked.connect(self.add_sibling_after_cb)
        self.add_node_sibling_btn.hide()

        self.delete_node_btn.clicked.connect(self.delete_cb)
        self.delete_node_btn.hide()
        
        self.save_node_btn.clicked.connect(self.save_node)
        
        self.node_load_btn.clicked.connect(self.load_node)

        self.regenerate_btn.clicked.connect(self.regenerate_node)
        self.regenerate_btn.hide()

        # self.node_load_tree_view.hide()
        
        self.load_node_widget.hide()
        # self.node_load_label.hide()
        # self.node_load_cancel_btn.hide()
        self.node_load_tree_view.clicked.connect(self.selected_load_node_cb)
        self.node_load_tree_view.expanded.connect(self.load_node_expanded_cb)
        self.node_load_cancel_btn.clicked.connect(self.cancel_load_node)
        
        self.subtree_load_btn.clicked.connect(self.load_subtree)
        self.subtree_load_cancel_btn.clicked.connect(self.cancel_load_subtree)
        # self.subtree_load_cancel_btn.hide()
        
        self.run_btn.clicked.connect(self.run_tree)
        self.run_btn.hide()
        self.stop_btn.clicked.connect(self.stop_tree)
        self.stop_btn.hide()

        self.subtree_save_btn.clicked.connect(self.save_subtree)
        self.subtree_save_ok_btn.clicked.connect(self.ok_save_subtree)
        self.subtree_save_cancel_btn.clicked.connect(self.cancel_save_subtree)
        self.subtree_save_field.textChanged.connect(self.subtree_save_name_update)
        self.subtree_save_widget.hide()

        self.load_tree_widget.hide()
        # self.subtree_load_tree_view.hide()
        # self.subtree_load_commit_btn.hide()
        self.subtree_load_tree_view.clicked.connect(self.selected_load_subtree)
        self.subtree_load_tree_view.expanded.connect(self.subtree_load_expanded)
        self.subtree_load_commit_btn.clicked.connect(self.commit_load_subtree)

        self.show_info_checkbox.stateChanged.connect(self.show_info_cb)

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
    def run_tree(self):
        self.running__ = True
        self.run_timer_.start(100)
        rospy.logwarn('INSTRUCTOR: Task Tree STARTING')

    def run(self):
        result = self.root_node.execute()
        rospy.logwarn(result)
        self.regenerate_tree()
        if result == 'SUCCESS':
            rospy.logwarn('INSTRUCTOR: Task Tree FINISHED WITH SUCCESS')
            self.run_timer_.stop()
            self.running__ = False
            self.root_node.reset()
        elif result == 'FAILURE':
            rospy.logerr('INSTRUCTOR: Task Tree FINISHED WITH FAILURE')
            self.run_timer_.stop()
            self.running__ = False
            self.root_node.reset()
        elif result == 'NODE_ERROR':
            rospy.logwarn('INSTRUCTOR: Task Tree ERROR')
            self.run_timer_.stop()
            self.running__ = False
            self.root_node.reset()
        elif result == 'RUNNING':
            self.regenerate_tree(True)
            pass

    def stop_tree(self):
        rospy.logwarn('INSTRUCTOR: Task Tree STOPPED')
        self.run_timer_.stop()
        self.running__ = False
        self.root_node.reset()

# Save and Load Nodes ----------------------------------------------------------
    def save_node(self):
        # These two options should be mutually exclusive

        if self.gui_alt_selected_node != None:
            if type(self.current_node_generator.generate()) == str:
                rospy.logerr('The node must be fully defined to save it.')
            else:
                print ''
                generator_to_save = {'node_type':self.current_node_type, 'name':self.current_node_generator.get_name(), 'plugin_name':self.current_node_plugin_name, 'generator_info':self.current_node_generator.save()}
                D = yaml.dump(generator_to_save)
                print D
                print self.save_service(id=self.current_node_generator.get_name(),type='instructor_node',text=D)

        elif self.current_node_type != None:
            if type(self.current_node_generator.generate()) == str:
                rospy.logerr('The node must be fully defined to save it.')
            else:
                print ''
                generator_to_save = {'node_type':self.current_node_type, 'name':self.current_node_generator.get_name(), 'plugin_name':self.current_node_plugin_name, 'generator_info':self.current_node_generator.save()}
                D = yaml.dump(generator_to_save)
                print D
                print self.save_service(id=self.current_node_generator.get_name(),type='instructor_node',text=D)

        
    def load_node(self):
        print 'Generating list of nodes that are saved by LIBRARIAN'
        self.create_node_widget.hide()
        self.load_tree_widget.hide()
        self.clear_node_info()

        generator_list = self.list_service('instructor_node').entries
        if len(generator_list) > 0: # There are actually nodes to load
            self.load_node_widget.show()
            self.loadable_nodes = {}
            for g in generator_list:
                data = yaml.load(self.load_service(id=g,type='instructor_node').text)
                # print data
                self.loadable_nodes[data['name']] = data

            self.node_load_model = QStandardItemModel()
            self.node_load_model.setHorizontalHeaderLabels(['Name', 'Description'])
            self.node_load_tree_view.setModel(self.node_load_model)

            # Set up groups for standard types
            type_items = {t:QStandardItem(t + ' NODES') for t in self.types__}
            # Load in nodes
            for n in self.loadable_nodes.itervalues():
                item = QStandardItem(n['name'])
                type_items[n['node_type']].appendRow(item)
            for row in type_items.itervalues():
                self.node_load_model.appendRow(row)
            self.node_load_tree_view.resizeColumnToContents(0)

    def load_node_info(self):
        print 'Attempting to load node'
        try:
            node_data = self.loadable_nodes[self.selected_load_node]
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                print 'Node to load matches known nodes ['+ node_plugin_name +']'
                self.clear_node_info()
                self.current_node_generator = self.plugins[node_plugin_name]['module']()
                self.current_node_type = self.plugins[node_plugin_name]['type']
                self.current_node_plugin_name = node_plugin_name
                self.node_info_layout.addWidget(self.current_node_generator)
                # Add in parameters from saved file
                self.current_node_generator.load(node_data['generator_info'])
                # Close gui

        except KeyError as e:
            print "Not a valid node"

    def cancel_load_node(self):
        # self.node_load_tree_view.hide()
        self.load_node_widget.hide()
        # self.node_load_cancel_btn.hide()
        # self.node_load_label.hide()
        self.loadable_nodes = {}
        self.node_load_model = None
        self.clear_node_info()
        # self.node_create_tree_view.show()
        self.create_node_widget.show()
        # self.node_create_label.show()

    def selected_load_node_cb(self,val):
        self.selected_load_node = str(self.node_load_model.itemFromIndex(val).text())
        self.load_node_info()

# Save and Load Subtrees -------------------------------------------------------
    def save_subtree(self):
        print 'showing'
        self.subtree_save_widget.show()
        self.subtree_save_name = None

    def subtree_save_name_update(self,t):
        self.subtree_save_name = str(t)
        pass

    def ok_save_subtree(self):
        if self.gui_selected_node == None:
            rospy.logerr('There is no head node selected to save')
        else:
            if self.subtree_save_name != None:
                tree = self.walk_tree(self.current_tree[self.gui_selected_node]) # this should start from the selected node
                # tree = self.walk_tree(self.root_node) #this will always start from root
                D = yaml.dump({'name':self.subtree_save_name,'tree':tree})
                rospy.logwarn('SAVING SUBTREE')
                rospy.logwarn(D)
                print self.save_service(id=self.subtree_save_name,type='instructor_subtree',text=D)
                # Hide on successful save    
                self.subtree_save_widget.hide()
            else:
                rospy.logerr('You must enter a name to save the subtree')

    def cancel_save_subtree(self):
        self.subtree_save_widget.hide()
        pass

    def walk_tree(self,node):
        t = [self.walk_tree(C) for C in node.children_]
        # Generate Info
        generator = self.all_generators[self.current_plugin_names[node.name_]]
        generator.load(self.current_node_info[node.name_])
        # generator = self.current_generators[node.name_]
        node_type = self.current_node_types[node.name_]
        plugin_name = self.current_plugin_names[node.name_]
        generator_to_save = {'node_type':node_type, 'name':node.name_, 'plugin_name':plugin_name, 'generator_info':generator.save()}

        return {'name':node.name_, 'save_info':generator_to_save, 'children':t}

    def selected_load_subtree(self,val):
        self.selected_load_subtree = str(self.subtree_load_model.itemFromIndex(val).text())

        self.selected_load_subtree_data = yaml.load(self.load_service(id=self.selected_load_subtree,type='instructor_subtree').text)
        selected_subtree_root_name = self.selected_load_subtree_data['tree']['save_info']['plugin_name']
        print selected_subtree_root_name
        if 'root' in selected_subtree_root_name.lower():
            print 'You have selected a tree with a root node.  If a node is currently selected, this subtree will be added as a child tree of the selected node.  If the graph is empty, this subtree will be added as the entire tree.'
        else:
            print 'The selected subtree has no root node.  If a node is currently selected, this subtree will be added as a child tree.  If no node is selected, the subtree will be added along with a root node.'

    def commit_load_subtree(self):
        if self.selected_load_subtree != None:
            clear_cmd()
            print 'loading subtree'
            if self.root_node != None: # there is a root node
                rospy.logwarn('Loading as subtree...')
                if self.gui_selected_node == None:
                    print'must select a node to add the sub tree'
                else:
                    try:
                        self.recursive_add_nodes(self.selected_load_subtree_data['tree'],self.current_tree[self.gui_selected_node])
                        self.add_node_child_btn.show()
                        self.delete_node_btn.show()
                        self.run_btn.show()
                        self.stop_btn.show()
                        self.add_node_sibling_btn.show()
                        self.regenerate_tree(center=True)
                    except Exception as e:
                        rospy.logerr('There was a problem loading the tree, most likely a node that didnt generate properly')
                        rospy.logerr(e)
            else:
                rospy.logwarn('Loading as full tree...')
                try:
                    self.recursive_add_nodes(self.selected_load_subtree_data['tree'],None)
                    self.add_node_child_btn.show()
                    self.delete_node_btn.show()
                    self.add_node_root_btn.hide()
                    self.add_node_sibling_btn.show()
                    self.run_btn.show()
                    self.stop_btn.show()
                    self.regenerate_tree(center=True)
                except Exception as e:
                    rospy.logerr('There was a problem loading the tree, most likely a node that didnt generate properly')
                    rospy.logerr(e)

            self.load_tree_widget.hide()
            self.create_node_widget.show()
        else:
            print 'You must select a subree to load'

    def recursive_add_nodes(self,info,parent):
        if parent == None: 
            # add root node
            print 'adding root ['+ str(info['name']) +'] to graph'
            node_data = info['save_info']
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                rospy.logwarn('Root node matches known nodes ['+ node_plugin_name +']')
                # node_generator = self.plugins[node_plugin_name]['module']()
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
                # self.current_generators[current_name] = node_generator
                self.current_plugin_names[current_name] = plugin_name
                self.current_node_types[current_name] = node_type
                self.root_node = node_to_add
            else:
                rospy.logwarn('node plugin undefined')
            # recusively call child nodes
            for C in info['children']:
                self.recursive_add_nodes(C,node_to_add)
        else: 
            # add none root node
            rospy.logwarn('adding node ['+ str(info['name']) +'] to graph as child of ' + parent.name_)
            node_data = info['save_info']
            node_plugin_name = node_data['plugin_name']
            if self.plugins.has_key(node_plugin_name):
                rospy.logwarn('Node matches known nodes ['+ node_plugin_name +']')
                # node_generator = self.plugins[node_plugin_name]['module']()
                node_type = self.plugins[node_plugin_name]['type']
                plugin_name = node_plugin_name
                self.all_generators[node_plugin_name].load(node_data['generator_info'])
                # Generate Child
                node_to_add = self.all_generators[node_plugin_name].generate()
                current_name = self.all_generators[node_plugin_name].get_name()
                self.current_tree[current_name] = node_to_add
                self.current_tree[parent.name_].add_child(node_to_add)
                self.current_node_info[current_name] = self.all_generators[node_plugin_name].save()
                # self.current_generators[current_name] = node_generator
                self.current_plugin_names[current_name] = plugin_name
                self.current_node_types[current_name] = node_type
            else:
                rospy.logwarn(node_plugin_name)
                rospy.logwarn('node plugin undefined')
            # recusively call child nodes
            for C in info['children']:
                self.recursive_add_nodes(C,node_to_add)

    def load_subtree(self):
        print 'Generating list of trees that are saved by LIBRARIAN'
        # self.node_create_tree_view.hide()
        self.create_node_widget.hide()
        self.load_node_widget.hide()
        # self.node_create_label.hide()
        self.clear_node_info()

        self.subtree_list = self.list_service('instructor_subtree').entries
        if len(self.subtree_list) > 0: # There are actually nodes to load
            self.load_tree_widget.show()
            # self.loadable_subtrees = {}
            # for s in self.subtree_list:
            #     data = yaml.load(self.load_service(id=s,type='instructor_subtree').text)
            #     # print data
            #     self.loadable_subtrees[data['name']] = data

            self.subtree_load_model = QStandardItemModel()
            self.subtree_load_model.setHorizontalHeaderLabels(['Subtree Name'])
            self.subtree_load_tree_view.setModel(self.subtree_load_model)

            # Load in subtrees to list
            # for ls in self.loadable_subtrees.itervalues():
            #     item = QStandardItem(ls['name'])
            #     self.subtree_load_model.appendRow(item)

            for t in self.subtree_list:
                item = QStandardItem(str(t))
                self.subtree_load_model.appendRow(item)
            self.subtree_load_tree_view.resizeColumnToContents(0)
            self.subtree_load_tree_view.sortByColumn(0,0)

    def cancel_load_subtree(self):
        # self.subtree_load_tree_view.hide()
        # self.load_tree_widget.show()
        self.create_node_widget.show()
        # self.subtree_load_cancel_btn.hide()
        # self.subtree_load_label.hide()
        self.load_tree_widget.hide()
        self.loadable_subtrees = {}
        self.subtree_load_model = None
        self.clear_node_info()
        # self.node_create_tree_view.show()
        self.create_node_widget.show()
        # self.node_create_label.show()
        pass

# Callbacks --------------------------------------------------------------------
    def selected_list_node_cb(self,val):
        name = str(self.node_model.itemFromIndex(val).text())
        print 'Selected node [' + name + '] from list.'
        if self.plugins.has_key(name):
            self.clear_node_info()
            ### THIS WORKS OK NOW, BUT BETTER TO HAVE A REGENERATE FUNCTION AND NOT REINSTANTIATE 
            rospy.logwarn('REGENERATING GENERATOR FOR ['+name+']')
            self.all_generators[name] = self.plugins[name]['module']()
            ###
            self.current_node_generator = self.all_generators[name]
            self.current_node_type = self.plugins[name]['type']
            self.current_node_plugin_name = name
            self.node_info_layout.addWidget(self.current_node_generator)
            self.gui_alt_selected_node = None
            rospy.logwarn('done')
        else:
            self.clear_node_info()

    def node_gui_selected_cb(self,event):
        if event == 'none':
            self.gui_selected_node = None
            self.selected_node_field.setText('NONE')
            self.selected_node_field.setStyleSheet('background-color:#FFB85C')
            # Remove all highlights
            if len(self.current_tree.keys()) != 0:
                for n in self.current_tree.itervalues():
                    if n.flag_ == True:
                        n.set_flag(False)
                self.regenerate_tree()
        else:
            if event in self.current_tree:
                # print '[' + event + '] found in current tree'
                self.gui_selected_node = event
                self.selected_node_field.setText(str(event).upper())
                self.selected_node_field.setStyleSheet('background-color:#B2E376')
                # Remove all highlights
                for n in self.current_tree.itervalues():
                    if n.flag_ == True:
                        n.set_flag(False)
                # highlight the selected node
                self.current_tree[str(event)].set_flag(True)
                self.regenerate_tree()
                self.gui_alt_selected_node = None
            else:
                print '[' + event + '] NOT found in current tree...'

    def node_gui_alt_selected_cb(self,event):
        # print event
        if event == 'none':
            self.gui_alt_selected_node = None
        else:
            if event in self.current_tree:
                self.gui_alt_selected_node = event
                self.clear_node_info()
                self.current_node_generator = self.all_generators[self.current_plugin_names[event]]
                self.current_node_generator.name.set_read_only(True)
                self.current_node_generator.load(self.current_node_info[event])
                # self.current_node_generator
                # self.current_node_generator = self.plugins[self.current_plugin_names[event]]['module']()
                # self.current_node_generator.name.set_read_only(True)
                # self.current_node_generator.load(self.current_generators[self.gui_alt_selected_node].save())
                ## debug
                # print '### loaded generator'
                # print yaml.dump(self.current_node_generator.save())
                ##
                self.current_node_type = self.current_node_types[self.gui_alt_selected_node]
                self.current_node_plugin_name = self.current_plugin_names[self.gui_alt_selected_node]
                self.node_info_layout.addWidget(self.current_node_generator)
                self.regenerate_btn.show()
            else:
                print '[' + event + '] NOT found in current tree...'

    def regenerate_node(self):
        if self.gui_alt_selected_node:
            current_name = self.gui_alt_selected_node
            replacement_node = self.current_node_generator.generate()
            if type(replacement_node) == str:
                rospy.logerr(str(replacement_node))
            else:
                # current_name = self.current_node_generator.get_name()
                current_child = self.current_tree[current_name]
                current_parent = self.current_tree[current_name].get_parent()
                if current_parent.replace_child(current_child, replacement_node):
                    self.current_tree[current_name] = replacement_node
                    self.current_node_generator.name.set_read_only(False)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    # self.current_generators[current_name] = self.current_node_generator
                    ## debugging
                    # print '### CURRENT ADD'
                    # print yaml.dump(self.current_node_generator.save())
                    # print '### NEW TREE'
                    # for s in self.current_generators.itervalues():
                    #     rospy.logwarn(yaml.dump(s.save()))
                    ##
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.regenerate_tree()
                    self.add_node_sibling_btn.show()
                    # Prime GUI for another node of the same type
                    print 'regenerating UI' ## debug
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    # self.current_node_generator.name.set_read_only(True)
                    # self.current_node_generator.load(self.current_node_info[event])
                    # new_node_generator = self.plugins[self.current_node_plugin_name]['module']()
                    # new_node_generator.load(self.current_node_generator.save())
                    # self.current_node_generator = new_node_generator
                    self.node_info_layout.addWidget(self.current_node_generator)
                    self.regenerate_btn.hide()
                else:
                    rospy.logwarn('Child to replace was not a member of the parent. Something went wrong.')
        
# Add and delete nodes ---------------------------------------------------------
    def add_root_cb(self):
        print 'adding root node of type ' + self.current_node_type
        if self.current_node_type != None:
          node_to_add = self.current_node_generator.generate()
          if type(node_to_add) == str:
            rospy.logerr(str(node_to_add))
          else:
            current_name = self.current_node_generator.get_name()
            self.current_tree[current_name] = node_to_add
            self.current_node_info[current_name] = self.current_node_generator.save()
            # self.current_generators[current_name] = self.current_node_generator
            self.current_plugin_names[current_name] = self.current_node_plugin_name
            self.current_node_types[current_name] = self.current_node_type
            self.root_node = node_to_add
            self.add_node_child_btn.show()
            self.delete_node_btn.show()
            self.add_node_root_btn.hide()
            self.run_btn.show()
            self.stop_btn.show()
            self.regenerate_tree()
            self.clear_node_info()
    
    def add_child_cb(self):
        print '### CURRENT TREE'
        for s in self.current_generators.itervalues():
            print yaml.dump(s.save())
        print 'adding child node of type ' + self.current_node_type
        if self.current_node_type != None:
            if self.gui_selected_node == None:
                rospy.logerr('There is no parent node selected')
            else:
                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    self.current_tree[self.gui_selected_node].add_child(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    # self.current_generators[current_name] = self.current_node_generator
                    ## debugging
                    print '### CURRENT ADD'
                    print yaml.dump(self.current_node_generator.save())
                    print '### NEW TREE'
                    for s in self.current_generators.itervalues():
                        print yaml.dump(s.save())
                    ##
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type
                    self.regenerate_tree()
                    self.add_node_sibling_btn.show()
                    # Prime GUI for another node of the same type
                    print 'regenerating UI' ## debug
                    self.clear_node_info()
                    self.current_node_generator = self.all_generators[self.current_node_plugin_name]
                    # new_node_generator = self.plugins[self.current_node_plugin_name]['module']()
                    # new_node_generator.load(self.current_node_generator.save())
                    # self.current_node_generator = new_node_generator
                    self.node_info_layout.addWidget(self.current_node_generator)

    def add_sibling_after_cb(self):
        print 'adding sibling node of type ' + self.current_node_type
        if self.current_node_type != None:
            if self.gui_selected_node == None:
              rospy.logerr('There is no left sibling node selected')
            else:
                node_to_add = self.current_node_generator.generate()
                if type(node_to_add) == str:
                    rospy.logerr(str(node_to_add))
                else:
                    current_name = self.current_node_generator.get_name()
                    self.current_tree[current_name] = node_to_add
                    sibling_node = self.current_tree[self.gui_selected_node]
                    sibling_node.add_sibling_after(node_to_add)
                    self.current_node_info[current_name] = self.current_node_generator.save()
                    # self.current_generators[current_name] = self.current_node_generator
                    self.current_plugin_names[current_name] = self.current_node_plugin_name
                    self.current_node_types[current_name] = self.current_node_type 
                    self.regenerate_tree()
                    self.add_node_sibling_btn.show()

    def delete_cb(self):
        print 'removing node of type ' + self.current_node_type
        if self.gui_selected_node == None:
            rospy.logerr('There is no node selected in the gui')
        elif 'root' in self.gui_selected_node:
            print 'deleting root node and clearing tree'
            self.current_tree.clear()
            self.current_generators.clear()
            self.current_node_types.clear()
            self.current_plugin_names.clear()
            self.root_node.remove_all_children()
            del(self.root_node)
            self.add_node_root_btn.show()
            self.add_node_child_btn.hide()
            self.add_node_sibling_btn.hide()
            self.dot_widget.set_dotcode('digraph behavior_tree {}')
        else:
            node = self.current_tree[self.gui_selected_node]
            if node.remove_self():
                self.current_tree.pop(self.gui_selected_node)
                self.current_node_info.pop(self.gui_selected_node)
                # self.current_generators.pop(self.gui_selected_node)
                self.current_node_types.pop(self.gui_selected_node)
                self.current_plugin_names.pop(self.gui_selected_node)
                self.regenerate_tree()
                # Reset the selected node
                self.gui_selected_node = None
                self.selected_node_field.setText('NONE')
                self.selected_node_field.setStyleSheet('background-color:#FFB85C')

    def regenerate_tree(self,runtime=False,center=True):
        # print self.root_node.generate_dot()
        self.dot_widget.set_dotcode(self.root_node.generate_dot(runtime),center)

# UI Specific Functions --------------------------------------------------------
    def clear_node_info(self):

        for i in reversed(range(self.node_info_layout.count())): 
            item = self.node_info_layout.takeAt(i)
            widget = item.widget()
            widget.setParent(None)
            # if widget is not None:
            #     widget.deleteLater()

            # children = widget.findChildren(QtGui.QWidget)
            # for j in children:
            #     j.deleteLater()

            # self.node_info_layout.removeWidget(widget)
            # del(widget)
            # self.node_info_layout.removeItem(item)


        # if self.current_node_generator is not None:
        #     del(self.current_node_generator)
        #     self.current_node_generator = None
        # else:
        #     rospy.logwarn('no node generator yet')


        # rospy.logwarn(len(self.findChildren(QtGui.QWidget)))

            # self.node_info_layout.itemAt(i).widget().setParent(None)


    def node_expanded_cb(self):
        self.node_create_tree_view.resizeColumnToContents(0)
        
    def load_node_expanded_cb(self):
        self.node_load_tree_view.resizeColumnToContents(0)

    def subtree_load_expanded(self):
        self.subtree_load_tree_view.resizeColumnToContents(0)

    def keyPressEvent(self, event):
        if type(event) == QtGui.QKeyEvent:
            #here accept the event and do something
            # print event.key()
            if event.key() == 16777220:
                if 'root' in self.current_node_plugin_name.lower():
                    self.add_root_cb()
                else:
                    self.add_child_cb()
            elif event.key() == 16777223:
                self.delete_cb()
            elif event.key() == 16777219:
                self.delete_cb()
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








