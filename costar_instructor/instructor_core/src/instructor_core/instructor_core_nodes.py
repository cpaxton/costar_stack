#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy, rospkg 
from std_msgs.msg import *
import beetree
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from instructor_core.instructor_qt import NamedField, NamedComboBox
import inspect

# BASE Node GUI Class ----------------------------------------------------------
class NodeGUI(QWidget):
    def __init__(self,color='blue'):
        super(NodeGUI,self).__init__()
        rospack = rospkg.RosPack()
        ui_path = rospack.get_path('instructor_core') + '/ui/node_gui.ui'
        uic.loadUi(ui_path,self)

        self.name = NamedField('Name','',color)
        self.label = NamedField('Label','',color)
        self.name_layout.addWidget(self.name)
        self.name_layout.addWidget(self.label)
    def reset(self):
        pass
    def generate(self):
        pass
    def get_name(self):
        return self.name.get()
    def get_label(self):
        return self.label.get()
    def save(self):
        data = {}
        for name, obj in inspect.getmembers(self):
            if isinstance(obj, NamedField):
                data[name] = {'type':'NamedField','field_name':obj.get_name(), 'value':obj.get()}
            if isinstance(obj, NamedComboBox):
                data[name] = {'type':'NamedComboBox','field_name':obj.get_name(), 'current_value':obj.get(),'index':obj.get_index(), 'values':obj.get_values()}
                pass
        data = self.save_data(data)
        return data
    def save_data(self,data):
        return data
    def load(self,data):
        for name, obj in inspect.getmembers(self):
            if isinstance(obj, NamedField):
                datum = data[name]
                obj.set_name(datum['field_name'])
                obj.set_field(datum['value'])
            if isinstance(obj, NamedComboBox):
                datum = data[name]
                obj.set_name(datum['field_name'])
                obj.add_items(datum['values'])
                obj.set_index(datum['index'])
                obj.set_value(datum['current_value'])
        self.load_data(data)
    def load_data(self,data):
        pass
    def refresh_data(self):
        pass
#-------------------------------------------------------------------------------


# CORE Classes -----------------------------------------------------------------
class NodeRootGUI(NodeGUI):
    def __init__(self):
        super(NodeRootGUI,self).__init__()
        self.title.setText('ROOT')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')
        self.name.set_field('root')

    def generate(self):
        return beetree.NodeRoot('root','root')

class NodeParallelOneGUI(NodeGUI):
    def __init__(self):
        super(NodeParallelOneGUI,self).__init__()
        self.title.setText('PARALLEL ONE LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeParallelOne(self.get_name(),self.get_label())
        else:
            return 'ERROR: node name not defined'

class NodeParallelAllGUI(NodeGUI):
    def __init__(self):
        super(NodeParallelAllGUI,self).__init__()
        self.title.setText('PARALLEL ALL LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeParallelAll(self.get_name(),self.get_label())
        else:
            return 'ERROR: node name not defined'

class NodeParallelRemoveGUI(NodeGUI):
    def __init__(self):
        super(NodeParallelRemoveGUI,self).__init__()
        self.title.setText('PARALLEL REMOVE LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeParallelRemove(self.get_name(),self.get_label())
        else:
            return 'ERROR: node name not defined'

class NodeSequenceGUI(NodeGUI):
    def __init__(self):
        super(NodeSequenceGUI,self).__init__()
        self.title.setText('SEQUENCE LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeSequence(self.get_name(),self.get_label())
        else:
            return 'ERROR: node name not defined'

class NodeIteratorGUI(NodeGUI):
    def __init__(self):
        super(NodeIteratorGUI,self).__init__()
        self.title.setText('ITERATOR LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeIterator(self.get_name(),self.get_label())
        else:
            return 'ERROR: node name not defined'

class NodeSelectorGUI(NodeGUI):
    def __init__(self):
        super(NodeSelectorGUI,self).__init__()
        self.title.setText('SELECTOR LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeSelector(self.get_name(),self.get_label())
        else:
            return 'ERROR: node name not defined'

class NodeDecoratorRepeatGUI(NodeGUI):
    def __init__(self):
        super(NodeDecoratorRepeatGUI,self).__init__()
        self.title.setText('REPEAT LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')
        self.repeat = NamedField('Repeat (-1) for infinity',self.get_label())
        self.repeat.set_field('-1')
        self.layout_.addWidget(self.repeat)

    def generate(self):
        if all([self.name.full(),self.repeat.full()]):
            return beetree.NodeDecoratorRepeat(self.get_name(),self.get_label(),int(self.repeat.get()))
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeDecoratorResetGUI(NodeGUI):
    def __init__(self):
        super(NodeDecoratorResetGUI,self).__init__()
        self.title.setText('RESET LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')
        self.reset = NamedField('Reset (-1) for infinity',self.get_label())
        self.layout_.addWidget(self.reset)

    def generate(self):
        if all([self.name.full(),self.reset.full()]):
            return beetree.NodeDecoratorReset(self.get_name(),self.get_label(),int(self.reset.get()))
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeDecoratorIgnoreFailGUI(NodeGUI):
    def __init__(self):
        super(NodeDecoratorIgnoreFailGUI,self).__init__()
        self.title.setText('IGNORE FAIL LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')
        self.repeat = NamedField('Repeat',self.get_label())
        self.layout_.addWidget(self.repeat)

    def generate(self):
        if all([self.name.full()]):
            return beetree.NodeDecoratorIgnoreFail(self.get_name(),self.get_label())
        else:
            return 'ERROR: check that all menu items are properly selected for this node'

class NodeDecoratorWaitForSuccessGUI(NodeGUI):
    def __init__(self):
        super(NodeDecoratorWaitForSuccessGUI,self).__init__()
        self.title.setText('WAIT FOR SUCCESS LOGIC')
        self.title.setStyleSheet('background-color:#00C8FF;color:#ffffff')
        self.timeout = NamedField('Timeout',self.get_label())
        self.timeout.set_field('-1')
        self.layout_.addWidget(self.timeout)

    def generate(self):
        if all([self.name.full(),self.timeout.full()]):
            return beetree.NodeDecoratorWaitForSuccess(self.get_name(),self.get_label(),int(self.timeout.get()))
        else:
            return 'ERROR: check that all menu items are properly selected for this node'


