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

'''
# create a thread class to keep things up to date
class MyThread(QtCore.QThread):
    updated = QtCore.pyqtSignal(str)

    def __init__(self) 

    def run( self ):
        # do some functionality
        if len(gui.type_.get()) > 0:
            file_list = getList(gui.type_.get())
            gui.file_.add_items(file_list.entries)

    def stop (self ):
        self.exit()
'''

# Sample Node Wrappers -----------------------------------------------------------
class LoadParametersGUI(NodeGUI):
    def __init__(self):
        super(LoadParametersGUI,self).__init__()

        self.type_ = NamedComboBox('Item Type')
        self.file_ = NamedComboBox('Item Name')
        self.layout_.addWidget(self.type_)
        self.layout_.addWidget(self.file_)
        
        type_list = getList('')
        self.type_.add_items(type_list.entries)

        if len(self.type_.get()) > 0:
            file_list = getList(self.type_.get())
            self.file_.add_items(file_list.entries)

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return LoadParameters(parent,self.get_name(),self.get_label(),
                    self.type_.get(), self.file_.get())
        else:
            return 'ERROR: node not properly defined'

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
        print self.type_.get()
        self.file_list = getList(self.type_list.entries[self.type_.get()])
        self.file_.interface().clear()
        self.file_.add_items(self.file_list.entries)
        print "selected"

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full()]):
            return LoadText(parent,self.get_name(),self.get_label(),
                    self.type_.get(), self.file_.get())
        else:
            return 'ERROR: node not properly defined'

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
        if all([self.name.full(),self.label.full()],self.file_.full()):
            return SaveParameter(parent,self.get_name(),self.get_label(),
                    self.type_.get(), self.file_.get(), self.param_.get())
        else:
            return 'ERROR: node not properly defined'

class LoadEntryFromFileGUI(NodeGUI):
    def __init__(self):
        super(LoadEntryFromFileGUI,self).__init__()
        self.type_ = NamedComboBox('Item Type')
        self.file_ = NamedComboBox('Item Name')
        self.entry_ = NamedField('Item Entry','')
        self.layout_.addWidget(self.type_)
        self.layout_.addWidget(self.file_)
        self.layout_.addWidget(self.entry_)
        type_list = getList('')
        self.type_.add_items(type_list.entries)

    def generate(self,parent=None):
        if all([self.name.full(),self.label.full(), self.entry_.full()]):
            return LoadEntry(parent,self.get_name(),self.get_label(),
                    self.type_.get(), self.file_.get(), self.entry_.get())
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
        # TODO
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
        rospy.wait_for_service('instructor_plugins/test_service')
        try:
            test_service = rospy.ServiceProxy('instructor_plugins/test_service', AddTwoInts)
            self.result = test_service(request)
            self.finished_with_success = True
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False

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
        rospy.wait_for_service('instructor_plugins/test_service')
        try:
            test_service = rospy.ServiceProxy('instructor_plugins/test_service', AddTwoInts)
            self.result = test_service(request)
            self.finished_with_success = True
        except rospy.ServiceException, e:
            print e
            self.finished_with_success = False
