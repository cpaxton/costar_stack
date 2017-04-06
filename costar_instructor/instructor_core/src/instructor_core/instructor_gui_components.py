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
from instructor_core.srv import *
import os,sys, inspect, ast
from std_msgs.msg import *
import rospkg
import tf; 
import tf_conversions as tf_c
from instructor_core.instructor_qt import ColorOptions, Color
# Using roslib.rospack even though it is deprecated
import threading
# from roslib import rospack
import rospkg
import yaml
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import time
from copy import deepcopy
import costar_robot_msgs
from costar_robot_msgs.srv import *

color_options = ColorOptions()
colors = color_options.colors

class Dialog(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/save_dialog.ui'
        uic.loadUi(w_path, self)
        self.save_subtree_btn = InterfaceButton('SAVE SUBTREE',colors['gray'])
        self.save_cancel_btn = InterfaceButton('CANCEL',colors['red'])
        self.button_layout.addWidget(self.save_subtree_btn,1,0)
        self.button_layout.addWidget(self.save_cancel_btn,2,0)

class Button(QPushButton):
    def __init__(self, name, label, txtsz=12, color=Color('#222222','#ffffff'),parent=None, text_color=Color('#ffffff', '#ffffff', '#dddddd')):
        QPushButton.__init__(self, parent)
        self.setMouseTracking(True)
        self.name = name
        self.label = label
        self.setText(self.label)
        # COLOR
        self.color = color
        self.text_color = text_color
        self.default_style = 'border: solid 4px #ffffff; background-color:'+self.color.normal+';color:'+self.text_color.normal+';border:none;'
        self.hover_style = 'border: solid 4px #ffffff; background-color:'+self.color.hover+';color:'+self.text_color.normal+';border:none;'
        self.setStyleSheet(self.default_style)
        self.font = QtGui.QFont("Ubuntu", txtsz, QtGui.QFont.Bold)
        self.setFont(self.font)
        self.setFocusPolicy(QtCore.Qt.NoFocus)
        self.setMinimumHeight(30)
    def set_color(self,color):
        self.color = color
        self.default_style = 'border: solid 4px #ffffff; background-color:'+self.color.normal+';color:'+'#ffffff'+';border:none;'
        self.hover_style = 'border: solid 4px #ffffff; background-color:'+self.color.hover+';color:'+'#ffffff'+';border:none;'
        self.setStyleSheet(self.default_style)
    def enterEvent(self, event):
        self.setStyleSheet(self.hover_style)
    def leaveEvent(self, event):
        self.setStyleSheet(self.default_style)

class Field(QLabel):
    def __init__(self,name,color,txtsz=12,parent=None):
        QLabel.__init__(self,name,parent)
        self.font = QtGui.QFont("Ubuntu", txtsz, QtGui.QFont.Bold)
        self.setFont(self.font)
        self.setStyleSheet('background-color:'+color+';color:#ffffff')
        self.setAlignment(QtCore.Qt.AlignCenter)
        p = QSizePolicy()
        p.setVerticalPolicy(QSizePolicy.Expanding)
        self.setSizePolicy(p)
    def set_color(self,color):
        self.setStyleSheet('background-color:'+color+';color:#ffffff')

class HeadingButton(Button):
    def __init__(self, name, color, txtsz=12,parent=None):
        Button.__init__(self,name, name, txtsz=txtsz, color=color)

class ListItemButton(Button):
    def __init__(self, name, color, parent=None):
        Button.__init__(self, name, name, color)

class InterfaceButton(Button):
    def __init__(self, name, color, txtsz=12, parent=None):
        Button.__init__(self, name, name, txtsz=12, color=color)
        self.setMinimumHeight(30)

class ItemButton(Button):
    def __init__(self, name, color, size, fold=True, callback=None,parent=None):
        if fold == True:
            label = self.halve_string(name)
        else:
            label = name
        Button.__init__(self, name, label, txtsz=size, color=color)

        self.callback = callback
        self.clicked.connect(self.was_clicked)

    def was_clicked(self):
        self.callback(self.name) 

    def halve_string(s):
        l = len(s)
        h = l/2
        spaces = [i for i,ss in enumerate(s) if ss == ' ']
        d = 100000
        ind = 0
        for sp in spaces:
            dist = abs(h-sp)
            if dist < d:
                d = dist
                ind = sp
                print ind
        if ind == 0:
            return s
        else:
            s_new = s[:ind]+'\n'+s[ind:]
            return s_new

class HeadingContainer(QWidget):
    def __init__(self, name, label, color, size='small',parent=None):
        QWidget.__init__(self, parent)
        self.label = label
        self.name = name
        self.color = color
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/view_list_container.ui'
        uic.loadUi(w_path, self)
        self.button = HeadingButton(self.label,txtsz=10,color=self.color)
        self.button.setMinimumHeight(24)
        self.label_layout.addWidget(self.button)
        self.button.setEnabled(False)
        self.scrollArea.hide()

class Container(QWidget):
    def __init__(self, name, label, color, size='small', parent=None):
        QWidget.__init__(self,parent)
        self.label = label
        self.name = name
        self.color = color
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/view_list_container_vert.ui'
        uic.loadUi(w_path, self)
        self.scrollArea.verticalScrollBar().setStyleSheet(self.scroll_style(self.color))
        self.scrollArea.hide()
        self.scrollArea.setStyleSheet('background-color:'+self.color.hover+';')
        self.button = HeadingButton(self.label,self.color)
        if size == 'small':
            self.button.setMinimumHeight(48)
        elif size == 'large':
            self.button.setMinimumHeight(56)
        self.label_layout.addWidget(self.button)
        self.button.clicked.connect(self.expand)
        self.expanded = False
        self.items = {}
        self.groups = {}
        self.group_members = {}

    def register_callbacks(self,contract_cb, selected_cb=None):
        self.contract_cb = contract_cb
        self.selected_cb = selected_cb

    def expand(self):
        self.contract_cb(self.name)
        if self.expanded == False:
            self.scrollArea.show()
            self.expanded = True
        else:
            self.scrollArea.hide()
            self.expanded = False

    def contract(self):
        self.scrollArea.hide()
        self.expanded = False
        self.button.setStyleSheet(self.button.default_style)
        self.button.current_style = self.button.default_style

    def add_item(self, name):
        pass

    def remove_all(self):
        for i in reversed(range(self.container.count())): 
            item = self.container.takeAt(i)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)

    def scroll_style(self,color):
        st = self.scroll_bar
        st = st.replace('_BG_',color.hover)
        st = st.replace('_FG_',color.normal)
        return st

    # scroll_bar = '''QScrollBar::sub-page:vertical {
    #  margin-top:3px;
    #  margin-bottom:3px;
    #  background-color: _BG_;
    # }
    # QScrollBar::add-page:vertical {
    #  margin-top:3px;
    #  margin-bottom:3px;
    #  background-color: _BG_;
    # }
    # QScrollBar::handle:vertical {
    #  background: _FG_;
    # }  
    # QScrollBar::add-line:vertical {
    #  width: 0px;  
    #  height: 0px;
    # }
    # QScrollBar::sub-line:vertical {
    #     width: 0px;  
    #     height: 0px;
    # }'''

    scroll_bar = '''QScrollBar::sub-page:vertical {
     margin-top:3px;
     margin-bottom:3px;
     background-color: _BG_;
    }
    QScrollBar::add-page:vertical {
     margin-top:3px;
     margin-bottom:3px;
     background-color: _BG_;
    }
    QScrollBar:vertical { width: 28px; }
    QScrollBar::handle:vertical {
     background: #666666;
    }  
    QScrollBar::add-line:vertical {
     width: 0px;  
     height: 0px;
    }
    QScrollBar::sub-line:vertical {
        width: 0px;  
        height: 0px;
    }'''

class ListContainer(Container):
    def __init__(self, name, label, color,size='small',parent=None):
        Container.__init__(self,name,label,color,size,parent)

    def sort(self):
        for name, widget in self.items.items():
            self.container.removeWidget(widget)
        items = self.items.keys()
        items.sort()

        for i in items:
            self.container.addWidget(self.items[i])

    def add_item(self,name):
        w = ItemButton(name,self.color,12,fold=False,callback=self.selected_cb)
        w.setMinimumHeight(36)
        w.show()
        self.items[name]=w
        self.container.addWidget(w)

    def add_group(self,g_name):
        # w = ItemButton(g_name,colors['green_light'],10,fold=False,callback=self.selected_cb)
        w = Field(g_name,self.color.hover,txtsz=10)
        w.setMinimumHeight(18)
        w.setMaximumHeight(19)
        w.show()
        self.groups[g_name]=w
        self.group_members[g_name] = []
        self.container.addWidget(w)

    def add_item_to_group(self,name,group):
        w = ItemButton(name,self.color,12,fold=False,callback=self.selected_cb)
        w.setMinimumHeight(36)
        w.show()
        self.items[name]=w
        # Check that group exists
        if not self.groups.has_key(group):
            self.add_group(group)

        index = self.container.indexOf(self.groups[group])
        self.container.insertWidget(index+1,w)
        self.group_members[group].append(name)

    def remove_item_from_group(self,name,group):
        rospy.logwarn('REMOVED ITEM ['+str(name)+']')
        item = self.items[name]
        item.hide()
        item.deleteLater()
        del(self.items[name])
        self.group_members[group].remove(name)

        # Remove group if empty
        if len(self.group_members[group]) == 0:
            group_index = self.container.indexOf(self.groups[group])
            item = self.container.takeAt(group_index)
            group_widget = item.widget()
            if group_widget is not None:
                group_widget.setParent(None)
            del(self.groups[group])

        # TODO Get rid of empty space by fixing indexes for added items that are now gone

class GridContainer(Container):
    def __init__(self, name, label, color, size='small',parent=None):
        Container.__init__(self,name,label,color,size,parent)
        self.xp = 0
        self.yp = 0

    def add_item(self,name):
        w = ItemButton(name,self.color,14,fold=True,callback=self.selected_cb)
        w.setMinimumHeight(96)
        w.show()
        self.items[name]=w
        self.container.addWidget(w,self.xp,self.yp)
        if self.yp == 1:
            self.yp = 0
            self.xp += 1
        else:
            self.yp +=1

class Drawer(QWidget):
    def __init__(self,parent):
        QWidget.__init__(self,parent)
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/drawer_2.ui'
        uic.loadUi(w_path, self)

    def window_resized(self,event):
        visible = self.drawer.isVisible()
        self.resize(360,self.container_widget.geometry().height()-100)
        self.move(self.container_widget.geometry().left()-360,50)
        if visible:
            self.show()
        else:
            self.hide()

class NotificationDialog(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        # GUI
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/notifications.ui'
        uic.loadUi(w_path, self)
        self.show_hide = self.show_hide_slide
        self.selected_object = None
        self.selected_move = None
        self.saved_geom = None
        self.new_move_name = None

        # self.manager = SmartWaypointManager(ns="")
        
        self.done_button.clicked.connect(self.done)
        # self.name_field.textChanged.connect(self.move_name_cb)
        self.notification_list.itemClicked.connect(self.object_selected)
        self.move_list.itemClicked.connect(self.move_selected)

        # self.add_move_button.clicked.connect(self.add_move)
        # self.delete_move_button.clicked.connect(self.delete_move)

        self.update_objects()
        self.update_moves()

    def notify(self, message, severity='info'):
        """Adds a message to the notification dialog, showing it if not visible.

        # Arguments

        message: string message to show in the dialog
        severity: one of 'info', 'warn', 'error'
        """
        self.notification_list.addItem(QListWidgetItem(message))
        if not self.isVisible():
             self.show_hide_slide()
        
        if severity is 'warn':
            rospy.logwarn(message)
        elif severity is 'error':
            rospy.logerr(message)
        else:
            rospy.loginfo(message)

    def show_hide_slide(self):
        if self.isVisible():
            self.saved_geom = self.geometry()
            self.hide()
        else:
            if self.saved_geom is not None:
                self.move(self.saved_geom.x(),self.saved_geom.y())
            else:
                self.move(self.geometry().x()+self.geometry().width()/2-self.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.smartmove_dialog.geometry().height()/2)
            self.show()
            self.update_all()

    def update_all(self):
        self.update_objects()
        self.update_moves()

    def done(self):
        self.notification_list.clear()
        self.show_hide()

    # def move_name_cb(self,text):
    #     self.new_move_name = str(text)
    #     self.update_objects()

    def object_selected(self,item):
        self.selected_object = str(item.text())
        self.object_field.setText(self.selected_object)
        self.update_moves()

    def move_selected(self,item):
        # self.update_objects()
        self.selected_move = str(item.text())

    # def update_objects(self):
    #     self.notification_list.clear()
    #     self.found_objects = self.manager.get_detected_objects()
    #     # Populate objects in list
    #     if self.found_objects is not None:
    #         self.found_objects.sort()
    #         idx = None
    #         for i,m in enumerate(self.found_objects):
    #             name = m.strip('/')
    #             if self.selected_object is not None and self.selected_object == name:
    #                 idx = i
    #             self.notification_list.addItem(QListWidgetItem(name))

    #         if idx is None:
    #             self.notification_list.setCurrentRow(0)
    #             if self.notification_list.currentItem() is not None:
    #                 self.selected_object = str(self.notification_list.currentItem().text())
    #         else:
    #             self.notification_list.setCurrentRow(idx)

    # def update_moves(self):
    #     if self.selected_object is not None:
    #         self.move_list.clear()

    #         # Populate moves in list
    #         self.manager.load_all()
    #         self.found_moves = self.manager.get_moves_for_object(self.selected_object)
    #         if self.found_moves is not None:
    #             self.found_moves.sort()
    #             idx = None
    #             for i, m in enumerate(self.found_moves):
    #                 name = m.strip('/')
    #                 self.move_list.addItem(QListWidgetItem(name))
    #                 if self.selected_move is not None and self.selected_move == name:
    #                     idx = i

    #             if idx is None:
    #                 self.move_list.setCurrentRow(0)
    #                 if self.move_list.currentItem() is not None:
    #                     self.selected_move = str(self.move_list.currentItem().text())
    #             else:
    #                 self.move_list.setCurrentRow(idx)

    # def add_move(self):
    #     self.update_objects()
    #     rospy.loginfo('add move %s'%self.new_move_name)
    #     if self.new_move_name is not None and self.selected_object is not None:
    #         # librarian call to add a new move with new_move_name
    #         self.manager.save_new_waypoint(self.selected_object,self.new_move_name)
    #         # Update moves
    #         self.update_moves()
    #         pass

    # def delete_move(self):
    #     self.update_objects()
    #     rospy.loginfo('delete move %s'%self.selected_move)
    #     if self.selected_move is not None:
    #         # librarian call to remove move for selected class
    #         self.manager.delete(self.selected_move)

    #         # Update moves
    #         self.update_moves()
    #     pass


class Popup(QWidget):
    def __init__(self,parent):
        QWidget.__init__(self,parent)
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/context.ui'
        uic.loadUi(w_path, self)
