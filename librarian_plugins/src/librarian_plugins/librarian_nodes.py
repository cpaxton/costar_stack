#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_plugins')
import rospy 
from std_msgs.msg import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# Beetree and Instructor
import beetree; from beetree import Node
from instructor_core import NodeGUI
from instructor_core.instructor_qt import NamedField
from instructor_core.instructor_qt import NamedComboBox
# For testing the service node
from instructor_plugins.srv import *

from threading import Thread

from librarian_msgs.msg import *
from librarian_msgs.srv import *

#import rosparam


lister = rospy.ServiceProxy('/librarian/list', List)
saver = rospy.ServiceProxy('/librarian/save', Save)
loader = rospy.ServiceProxy('/librarian/load', Load)
param_loader = rospy.ServiceProxy('/libarian/load', Load)

'''
Get a list in a folder (or empty)
'''
def getList(folder):
    return lister(folder)

# Sample Node Wrappers -----------------------------------------------------------

'''
LoadParametersGUI
load parameters from a file
uses the librarian/load_params service

(written on 2014/07/02)
'''
class LoadParametersGUI(NodeGUI):
    def __init__(self):
        super(LoadParametersGUI,self).__init__()

        self.type_ = NamedComboBox('Item Type')
        self.file_ = NamedComboBox('Item Name')
        self.layout_.addWidget(self.type_)
        self.layout_.addWidget(self.file_)
        
        self.type_list = getList('')
        self.type_.add_items(self.type_list.entries)
        self.type_.interface().currentIndexChanged.connect(self.selected)
        self.file_list = []

        if len(self.type_.get()) > 0:
            self.file_list = getList(self.type_list.entries[int(self.type_.get())])
            self.file_.add_items(self.file_list.entries)

    def selected(self,t):
        value = str(t)
        self.file_list = getList(self.type_list.entries[int(self.type_.get())])
        self.file_.interface().clear()
        self.file_.add_items(self.file_list.entries)
        if len(self.type_.get()) > 0:
            file_list = getList(self.type_.get())
            self.file_.add_items(file_list.entries)

    def generate(self,parent=None):

        if len(self.file_list.entries) > 0:
            fname = self.file_list.entries[int(self.file_.get())]
        else:
            return 'ERROR: no items found!'

        if len(self.type_list.entries) > 0:
            tname = self.type_list.entries[int(self.type_.get())]
        else:
            return 'ERROR: no types found!'
        if all([self.name.full(),self.label.full()]):
            return LoadParameters(parent,self.get_name(),self.get_label(),
                    tname, fname)
        else:
            return 'ERROR: node not properly defined'

'''
LoadTextFromFileGUI()
get both the folder and item name from drop down boxes
send to node

(written 2014/07/02)
'''
class LoadTextFromFileGUI(NodeGUI):
    def __init__(self):
        super(LoadTextFromFileGUI,self).__init__()
        self.type_ = NamedComboBox('Item Type')
        self.file_ = NamedComboBox('Item Name')
        self.layout_.addWidget(self.type_)
        self.layout_.addWidget(self.file_)

        self.type_list = getList('')
        self.type_.add_items(self.type_list.entries)
        self.type_.interface().currentIndexChanged.connect(self.selected)
        self.file_list = []

        if len(self.type_.get()) > 0:
            self.file_list = getList(self.type_list.entries[int(self.type_.get())])
            self.file_.add_items(self.file_list.entries)

    def selected(self,t):
        value = str(t)
        self.file_list = getList(self.type_list.entries[int(self.type_.get())])
        self.file_.interface().clear()
        self.file_.add_items(self.file_list.entries)

    def generate(self,parent=None):

        if len(self.file_list.entries) > 0:
            fname = self.file_list.entries[int(self.file_.get())]
        else:
            return 'ERROR: no items found!'

        if len(self.type_list.entries) > 0:
            tname = self.type_list.entries[int(self.type_.get())]
        else:
            return 'ERROR: no types found!'

        print fname
        print tname

        if all([self.name.full(),self.label.full()]):
            return LoadText(parent,self.get_name(),self.get_label(),
                    fname, tname)
        else:
            return 'ERROR: node not properly defined'

'''
SaveParameterGUI
takes in text fields to get parameters
gets names
gets ids

(written on 2014/07/02)
'''
class SaveParameterGUI(NodeGUI):
    def __init__(self):
        super(SaveParameterGUI,self).__init__()
        self.type_ = NamedComboBox('Item Type')
        self.file_ = NamedField('Item Name','')
        self.layout_.addWidget(self.type_)
        self.layout_.addWidget(self.file_)
        #self.param_ = NamedComboBox('Parameter')
        self.param_ = NamedField('ROS Parameter','')
        self.layout_.addWidget(self.param_)

        type_list = getList('')
        self.type_.add_items(type_list.entries)

        #self.param_.add_items(rosparam.get_params(''))

    def generate(self,parent=None):

        if len(self.type_list.entries) > 0:
            tname = self.type_list.entries[int(self.type_.get())]
        else:
            return 'ERROR: no types found!'

        if all([self.name.full(),self.label.full()],self.file_.full()):
            return SaveParameter(parent,self.get_name(),self.get_label(),
                    tname, self.file_.get(), self.param_.get())
        else:
            return 'ERROR: node not properly defined'

'''
LoadEntryFromFileGUI
loads a text entry from a dictionary in a single configuration file somewhere
not really a high priority to implement this

(written on 2014/07/02)
'''
class LoadEntryFromFileGUI(NodeGUI):
    def __init__(self):
        super(LoadEntryFromFileGUI,self).__init__()
        self.type_ = NamedComboBox('Item Type')
        self.file_ = NamedComboBox('Item Name')
        self.entry_ = NamedField('Item Entry','')
        self.layout_.addWidget(self.type_)
        self.layout_.addWidget(self.file_)
        self.layout_.addWidget(self.entry_)
        self.type_list = getList('')
        self.type_.add_items(self.type_list.entries)
        self.type_.interface().currentIndexChanged.connect(self.selected)
        self.file_list = []

    def selected(self,t):
        value = str(t)
        self.file_list = getList(self.type_list.entries[int(self.type_.get())])
        self.file_.interface().clear()
        self.file_.add_items(self.file_list.entries)

    def generate(self,parent=None):

        if len(self.file_list.entries) > 0:
            fname = self.file_list.entries[int(self.file_.get())]
        else:
            return 'ERROR: no items found!'

        if len(self.type_list.entries) > 0:
            tname = self.type_list.entries[int(self.type_.get())]
        else:
            return 'ERROR: no types found!'

        if all([self.name.full(),self.label.full(), self.entry_.full()]):
            return LoadEntry(parent,self.get_name(),self.get_label(),
                    tname, fname, self.entry_.get())
        else:
            return 'ERROR: node not properly defined'

# Sample Nodes -------------------------------------------------------------------
class LoadText(Node):
    def __init__(self,parent,name,label,folder,fname):
        color='#5B8EEB'
        super(LoadText,self).__init__(False,parent,name,label,color)
        self.folder_ = folder
        self.name_ = fname

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Load Text from File'
    def execute(self):
        print 'Executing Service: ' + self.name_
        self.node_status_ = 'SUCCESS'
        print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_

class LoadEntry(Node):
    def __init__(self,parent,name,label,folder,fname,entry):
        color='#5B8EEB'
        super(LoadEntry,self).__init__(False,parent,name,label,color)
        self.folder_ = folder
        self.name_ = fname
        self.entry_ = entry

    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Load Entry from Text File'
    def execute(self):
        # TODO
        print 'Executing Service: ' + self.name_
        self.node_status_ = 'SUCCESS'
        print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_

'''
LoadParameters
Loads parameters to ROS with a Librarian service call

(written as of 2014/07/02)
'''
class LoadParameters(Node):
    def __init__(self,parent,name,label,folder,fname):
        super(LoadParameters,self).__init__(False,parent,name,label,'#92D665')
        self.service_thread = Thread(target=self.make_service_call, args=('request',1))
        self.running = False
        self.finished_with_success = None
        self.folder_ = folder
        self.name_ = fname
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Load Parameters From File'

    def execute(self):
        if not self.running: # Thread is not running
            if self.finished_with_success == None: # Service was never called
                try:
                    self.service_thread.start()
                    return set_status('RUNNING')
                    self.running = True
                except Exception, errtxt:
                    return set_status('FAILURE')
                    self.running = False
        else:
            # If thread is running
            if self.service_thread.is_alive():
                return set_status('RUNNING')
            else:
                if self.finished_with_success == True:
                    return set_status('SUCCESS')
                    self.running = False
                else:
                    return set_status('FAILURE')
                    self.running = False

    def make_service_call(self,request,*args):
        rospy.wait_for_service('librarian/load_params')
        try:
            test_service = rospy.ServiceProxy('librarian/load_params', librarian_msgs.srv.LoadParams)
            self.result = test_service(id=self.name_, type=self.folder_)
            self.finished_with_success = True
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False

'''
SaveParameter
Take a single thing off the ROS parameter server and put it in a file

(written as of 2014/07/02)
'''
class SaveParameter(Node):
    def __init__(self,parent,name,label,folder,fname,param):
        super(SaveParameter,self).__init__(False,parent,name,label,'#92D665')
        self.service_thread = Thread(target=self.make_service_call, args=('request',1))
        self.running = False
        self.finished_with_success = None
        self.folder_ = folder
        self.name_ = fname
        self.param_ = param
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Save Parameter to File'

    def execute(self):
        if not self.running: # Thread is not running
            if self.finished_with_success == None: # Service was never called
                try:
                    self.service_thread.start()
                    return set_status('RUNNING')
                    self.running = True
                except Exception, errtxt:
                    return set_status('FAILURE')
                    self.running = False
        else:
            # If thread is running
            if self.service_thread.is_alive():
                return set_status('RUNNING')
            else:
                if self.finished_with_success == True:
                    return set_status('SUCCESS')
                    self.running = False
                else:
                    return set_status('FAILURE')
                    self.running = False

    def make_service_call(self,request,*args):
        rospy.wait_for_service('librarian/save')
        try:
            val = rosparam.get_param(self.param_)
            test_service = rospy.ServiceProxy('librarian/save', librarian_msgs.srv.Save)
            self.result = test_service(type=self.folder_, id=self.name_, text=val)
            self.finished_with_success = True
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False
