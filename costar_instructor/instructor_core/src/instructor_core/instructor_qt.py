#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy 
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *

class Color:
    def __init__(self,normal,hover):
        self.normal = normal
        self.hover = hover

class ColorOptions:
    def __init__(self):
      self.colors = {}
      self.colors['purple'] = Color(normal='#8E44AD', hover='#9B59B6')
      self.colors['blue'] = Color(normal='#22A7F0', hover='#6BB9F0')
      self.colors['jellybean_blue'] = Color(normal='#2574A9', hover='#6BB9F0')
      self.colors['red'] = Color(normal='#D91E18', hover='#E74C3C')
      self.colors['gray'] = Color(normal='#34495E', hover='#67809F')
      self.colors['sea_green'] = Color(normal='#1BA39C', hover='#65C6BB')
      self.colors['gray_disabled'] = Color(normal='#34495E', hover='#34495E')
      self.colors['gray_light'] = Color(normal='#6C7A89',hover='#95A5A6')
      self.colors['green'] = Color(normal='#26A65B', hover='#3FC380')
      self.colors['green_light'] = Color(normal='#1BA39C', hover='#65C6BB')
      self.colors['orange'] = Color(normal='#E87E04', hover='#EB9532')
      self.colors['pink'] = Color(normal='#DB0A5B', hover='#E26A6A')
      self.colors['highlight_red'] = Color(normal='#F62459',hover='#F62459')
      self.colors['dark_red'] = Color(normal='#D24D57',hover='#E08283')

color_options = ColorOptions()
colors = color_options.colors

class Button(QPushButton):
    def __init__(self, name, label, txtsz=12, color=Color('#222222','#ffffff'),parent=None):
        QPushButton.__init__(self, parent)
        self.setMouseTracking(True)
        self.name = name
        self.label = label
        self.setText(self.label)
        # COLOR
        self.color = color
        self.default_style = 'border: solid 4px #ffffff; background-color:'+self.color.normal+';color:'+'#ffffff'+';border:none;'
        self.hover_style = 'border: solid 4px #ffffff; background-color:'+self.color.hover+';color:'+'#ffffff'+';border:none;'
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


class OverlayDialog(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent, QtCore.Qt.WindowStaysOnTopHint)

# A useful QWidget wrapper for named text fields -------------------------------
class NamedField(QWidget):
  def __init__(self,name,text,color='blue',parent=None):
    QWidget.__init__(self, parent)
    self.name = name
    self.value = text
    self.color = color
    self.layout = QHBoxLayout()
    self.layout.setContentsMargins(0,0,0,0)
    self.setLayout(self.layout)
    self.title = QLabel(name)
    self.title.setFont(QtGui.QFont("Ubuntu", 14, QtGui.QFont.Bold))
    self.title.setStyleSheet('background-color:'+colors[self.color].normal+' ; color:#ffffff')
    self.title.setAlignment(Qt.AlignCenter)
    self.title.setMinimumWidth(119)
    # self.title.adjustSize()
    self.title.setSizePolicy(QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum,QtGui.QSizePolicy.Maximum))
    # self.title.setMaximumWidth(120)
    self.title.setMinimumHeight(31)
    self.title.setMaximumHeight(32)
    self.field = QLineEdit(text)
    self.field.setFont(QtGui.QFont("Ubuntu", 14, QtGui.QFont.Bold))
    self.field.setStyleSheet('background-color:'+colors['gray_light'].normal+' ; color:#ffffff; border:none')
    self.field.setMinimumHeight(31)
    self.field.setMaximumHeight(32)
    self.field.setAlignment(Qt.AlignCenter)
    self.layout.addWidget(self.title)
    self.layout.addWidget(self.field)
    self.field.textChanged.connect(self.update_field_cb)
  def interface(self):
    return self.field
  def clear_field(self):
    self.field.clear()
    self.value = ''
  def set_field(self,text):
    self.field.setText(text)
    self.value = text
  def set_name(self,n):
    self.name = n
    self.title.setText(n)
    self.title.adjustSize()
  def get(self):
    return self.value
  def get_name(self):
    return self.name
  def empty(self):
    if self.value == '':
      return True
    else:
      return False
  def full(self):
    if self.value == '':
      return False
    else:
      return True
  def update_field_cb(self,t):
    if self.value != None:
      self.value = str(t)
      self.field.setStyleSheet('background-color:'+colors[self.color].hover+' ; color:#ffffff; border:none')
    if t == '':
      self.field.setStyleSheet('background-color:'+colors['gray_light'].normal+' ; color:#ffffff; border:none')
  def set_read_only(self,val):
    self.field.setDisabled(val)

class NoteField(QWidget):
  def __init__(self,text,color='green',parent=None):
    QWidget.__init__(self, parent)
    self.color = color
    self.layout = QHBoxLayout()
    self.layout.setContentsMargins(0,0,0,0)
    self.setLayout(self.layout)
    self.field = QLabel(text)
    self.field.setFont(QtGui.QFont("Ubuntu", 11, QtGui.QFont.Bold))
    self.field.setWordWrap(True)
    self.field.setStyleSheet('background-color:'+colors[self.color].hover+' ; color:#ffffff; border:none')
    self.field.setAlignment(Qt.AlignCenter)
    self.layout.addWidget(self.field)

class NamedComboBox(QWidget):
  def __init__(self,name,parent=None):
    QWidget.__init__(self, parent)
    self.value = ''
    self.name = name
    self.layout = QHBoxLayout()
    self.setLayout(self.layout)
    self.title = QLabel(name)
    self.title.setFont(QtGui.QFont("Ubuntu", 11, QtGui.QFont.Bold))
    self.layout.addWidget(self.title)
    self.comb = QComboBox()
    self.layout.addWidget(self.comb)
    self.comb.currentIndexChanged.connect(self.update_field_cb)
    self.setMinimumHeight(50)
    self.setMaximumHeight(50)
  def interface(self):
    return self.comb
  def set_index(self,i):
    self.comb.setCurrentIndex(i)
  def set_value(self,v):
    self.value = v
  def get(self):
    return self.value
  def get_index(self):
    return self.comb.currentIndex()
  def get_name(self):
    return self.name
  def get_values(self):
    return [self.comb.itemText(i) for i in range(self.comb.count())]
  def update_field_cb(self,t):
    if self.value != None:
      self.value = str(t)
  def add_item(self,item):
    self.comb.addItem(item)
  def add_items(self,items):
    self.comb.addItems(items)
