import rospy
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
from xdot.xdot_qt import DotWidget
import rospkg
from beetree import *

from smart_waypoint_manager import SmartWaypointManager

class SmartMoveDialog(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)
        # GUI
        rp = rospkg.RosPack()
        w_path = rp.get_path('instructor_core') + '/ui/smart_move.ui'
        uic.loadUi(w_path, self)
        self.show_hide = self.show_hide_slide
        self.selected_object = None
        self.selected_move = None
        self.saved_geom = None
        self.new_move_name = None

        self.manager = SmartWaypointManager(ns="")
        
        self.done_button.clicked.connect(self.done)
        self.name_field.textChanged.connect(self.move_name_cb)
        self.object_list.itemClicked.connect(self.object_selected)
        self.move_list.itemClicked.connect(self.move_selected)

        self.add_move_button.clicked.connect(self.add_move)
        self.delete_move_button.clicked.connect(self.delete_move)

        self.update_objects()
        self.update_moves()


    def show_hide_slide(self):
        if self.isVisible():
            self.saved_geom = self.geometry()
            self.hide()
        else:
            if self.saved_geom is not None:
                self.move(self.saved_geom.x(),self.saved_geom.y())
            else:
                self.move(self.geometry().x()+self.geometry().width()/2-self.geometry().width()/2,self.geometry().y()+self.geometry().height()/2-self.geometry().height()/2)
            self.show()
            self.update_all()

    def update_all(self):
        self.update_objects()
        self.update_moves()

    def done(self):
        self.show_hide()

    def move_name_cb(self,text):
        self.new_move_name = str(text)
        self.update_objects()

    def object_selected(self,item):
        self.selected_object = str(item.text())
        self.object_field.setText(self.selected_object)
        self.update_moves()

    def move_selected(self,item):
        # self.update_objects()
        self.selected_move = str(item.text())

    def update_objects(self):
        self.object_list.clear()
        self.found_objects = self.manager.get_detected_objects()
        # Populate objects in list
        if self.found_objects is not None:
            self.found_objects.sort()
            idx = None
            for i,m in enumerate(self.found_objects):
                name = m.strip('/')
                if self.selected_object is not None and self.selected_object == name:
                    idx = i
                self.object_list.addItem(QListWidgetItem(name))

            if idx is None:
                self.object_list.setCurrentRow(0)
                if self.object_list.currentItem() is not None:
                    self.selected_object = str(self.object_list.currentItem().text())
            else:
                self.object_list.setCurrentRow(idx)

    def update_moves(self):
        if self.selected_object is not None:
            self.move_list.clear()

            # Populate moves in list
            self.manager.load_all()
            self.found_moves = self.manager.get_moves_for_object(self.selected_object)
            if self.found_moves is not None:
                self.found_moves.sort()
                idx = None
                for i, m in enumerate(self.found_moves):
                    name = m.strip('/')
                    self.move_list.addItem(QListWidgetItem(name))
                    if self.selected_move is not None and self.selected_move == name:
                        idx = i

                if idx is None:
                    self.move_list.setCurrentRow(0)
                    if self.move_list.currentItem() is not None:
                        self.selected_move = str(self.move_list.currentItem().text())
                else:
                    self.move_list.setCurrentRow(idx)

    def add_move(self):
        self.update_objects()
        rospy.loginfo('add move %s'%self.new_move_name)
        if self.new_move_name is not None and self.selected_object is not None:
            # librarian call to add a new move with new_move_name
            self.manager.save_new_waypoint(self.selected_object,self.new_move_name)
            # Update moves
            self.update_moves()
            pass

    def delete_move(self):
        self.update_objects()
        rospy.loginfo('delete move %s'%self.selected_move)
        if self.selected_move is not None:
            # librarian call to remove move for selected class
            self.manager.delete(self.selected_move)

            # Update moves
            self.update_moves()
        pass
