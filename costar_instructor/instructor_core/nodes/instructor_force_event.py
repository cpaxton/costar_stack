#! /usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy
import sys, yaml
# import rviz
import rospkg
from librarian_msgs.msg import *
from librarian_msgs.srv import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# VTK
import vtk
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
# Robotiq
from robotiq_force_torque_sensor.msg import *
import robotiq_force_torque_sensor.srv as ft_srv
# Predicator
from predicator_msgs.msg import *

class TrackBallCameraFacePicker(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self,arrow_label,direction_selected,parent=None):
            self.direction_selected = direction_selected
            self.AddObserver("LeftButtonPressEvent",self.leftButtonPressEvent)
            self.selectedMapper = vtk.vtkDataSetMapper()
            self.selectedActor = vtk.vtkActor()
            self.arrows = {}
            self.labels = {}
            self.label_offsets = {}
            self.arrow_colors = {}
            self.forces = {}
            self.selected_arrow = None
            self.arrow_label = arrow_label
            self.X = self.Y = self.Z = 0
            self.flash = False
            self.flash_counter = .1
            self.flash_accending = True

    def set_flash(self,f):
        self.flash = f
    def set_arrows(self,a):
        self.arrows = a
    def set_labels(self,l):
        self.labels = l
    def set_arrow_colors(self,c):
        self.arrow_colors = c
    def set_label_offsets(self,o):
        self.label_offsets = o
    def update_forces(self,f):
        self.forces = f

    def leftButtonPressEvent(self,obj,event):
        pos = self.GetInteractor().GetEventPosition()

        ### Get Clicked Actor ### 
        prop_picker = vtk.vtkPropPicker()
        prop_picker.Pick(pos[0], pos[1], 0, self.GetDefaultRenderer())
        a = prop_picker.GetActor()

        self.arrow_label.setText('Force Direction: NONE')
        self.direction_selected(None)
        self.selected_arrow = None
        for name, arrow in self.arrows.items():
            if a == arrow:
                # print "Selected Actor ["+str(name)+"]"
                self.arrow_label.setText('Force Direction: '+str(name))
                self.direction_selected(str(name))
                self.selected_arrow = arrow
                self.selected_arrow.GetProperty().SetColor(1.0, .4, 0)
                self.set_flash(True)
            else:
                arrow.GetProperty().SetColor(self.arrow_colors[name])
        if self.selected_arrow == None:
            self.set_flash(False)
        self.GetInteractor().GetRenderWindow().Render()
        ### MOVEMENT
        self.OnLeftButtonDown()

        # print self.GetDefaultRenderer().GetActiveCamera().GetPosition()
        # print self.GetDefaultRenderer().GetActiveCamera().GetViewUp()
        # print self.GetDefaultRenderer().GetActiveCamera().GetFocalPoint()
    
    def set_current_direction(self,a):
        if a is not None:
            for name, arrow in self.arrows.items():
                if name == a:
                    # print "Selected Actor ["+str(name)+"]"
                    self.arrow_label.setText('Force Direction: '+str(name))
                    self.direction_selected(str(name))
                    self.selected_arrow = arrow
                    self.selected_arrow.GetProperty().SetColor(1.0, .4, 0)
                    self.set_flash(True)
                else:
                    arrow.GetProperty().SetColor(self.arrow_colors[name])
        else:
            self.arrow_label.setText('Force Direction: NONE')
            self.direction_selected(None)
            self.selected_arrow = None
            for name,arrow in self.arrows.items():
                arrow.GetProperty().SetColor(self.arrow_colors[name])

        # RENDER UPDATE
        self.GetInteractor().GetRenderWindow().Render()

    def update_labels(self):

        for a_id in self.arrows.keys():

            p = self.arrows[a_id].GetPosition()
            c = vtk.vtkCoordinate()
            c.SetCoordinateSystemToWorld()
            offset = self.label_offsets[a_id]
            c.SetValue([p[0]+offset[0],p[1]+offset[1],p[2]+offset[2]])
            v = c.GetComputedViewportValue(self.GetDefaultRenderer())
            self.labels[a_id].SetDisplayPosition(v[0],v[1])
            force_val = self.forces[a_id]
            self.labels[a_id].SetInput(str(self.filter(int(abs(force_val))))+'N')

    def filter(self,f):
        v = f
        if abs(v) < 1.5:
            v = 0
        return v

    def update_arrows(self):

        for a_id in self.arrows.keys():
            force_val = self.forces[a_id]
            opacity =  abs(float(self.filter(force_val))/75.0)*.90 + .10 if force_val > 0 else .1
            self.arrows[a_id].GetProperty().SetOpacity(opacity)

        if self.selected_arrow is not None:

            if self.flash:
                self.selected_arrow.GetProperty().SetOpacity(self.flash_counter)
                if self.flash_accending:
                    if self.flash_counter <= .4:
                        self.flash_counter = self.flash_counter+.05
                    else:
                        self.flash_accending = False    
                else:
                    if self.flash_counter >= .2:
                        self.flash_counter = self.flash_counter-.05 
                    else:
                        self.flash_accending = True

        self.update_labels()
        self.GetInteractor().GetRenderWindow().Render()

class ForceEvent():
    def __init__(self,name,force,direction):
        self.name = name
        self.force = force
        self.direction = direction

class ForceEventCreator(QWidget):
    pop_status = QtCore.pyqtSignal(str,str)
    def __init__(self,app):
        QWidget.__init__(self)
        rospy.init_node('force_event',anonymous=True)
        self.app_ = app
        self.rp = rospkg.RosPack()
        i_path = self.rp.get_path('instructor_core')

        self.gripper_stl_path = str(i_path + '/meshes/gripper_bin.STL')
        self.reversed = False
        self.started_event = True
        self.events = {}
        self.last_force_reading = None
        self.msg_number = 0
        self.forces = {}
        self.forces['+x'] = 0
        self.forces['-x'] = 0
        self.forces['+y'] = 0
        self.forces['-y'] = 0
        self.forces['+z'] = 0
        self.forces['-z'] = 0
        self.arrows = {}
        self.arrow_colors = {}
        self.labels = {}
        self.label_offsets = {}
        self.test_timer = QTimer()
        
        ### Set up librarian
        rospy.logwarn('Waypoint Manager Waiting for Librarian Services')
        rospy.wait_for_service('/librarian/add_type',5)
        self.set_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)
        self.set_type_service('instructor_force_events')
        rospy.logwarn('Librarian Proxies Created')

        ### Set up predicator
        self.pub_list = rospy.Publisher('/predicator/input', PredicateList)
        self.pub_valid = rospy.Publisher('/predicator/valid_input', ValidPredicates)
        rospy.sleep(.5)
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = []
        pval.assignments = ['force_events']
        self.pub_valid.publish(pval)

        ### CREATE UI
        self.create_ui()
        self.load()

        # ROS
        self.ok_timer = QTimer(self)
        self.connect(self.ok_timer, SIGNAL("timeout()"), self.check_ok)
        self.ok_timer.start(100)

        ### FORCE SUBSCRIBER
        self.force_sub = rospy.Subscriber('/robotiq_force_torque_sensor',ft_sensor,self.force_cb)
        self.zero_sensor()

    def update_predicates(self):
        ps = PredicateList()
        ps.pheader.source = rospy.get_name()
        ps.statements = []

        predicates = self.events.keys()
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = predicates
        pval.assignments = ['force_events']
        self.pub_valid.publish(pval)

        for e in self.events.values():
            force_limit = e.force
            direction = e.direction
            event_name = e.name

            if self.forces[direction] > float(force_limit): # Above Limit
                rospy.logwarn('force event ' + str(event_name) + " is TRUE")
                statement = PredicateStatement( predicate=str(event_name),
                                          confidence=1,
                                          value=True,
                                          num_params=1,
                                          params=['force_events', '', ''])
                ps.statements += [statement]
            else: # Not above limit
                # Dont send a false predicate??
                pass

        self.pub_list.publish(ps)

    def save(self):
        # print yaml.dump(self.events)
        rospy.logwarn('Saving Events...')
        try:
            self.save_service(id='force_events',type='instructor_force_events',text=yaml.dump(self.events))
        except (rospy.service.ServiceException) as e:
            pass

        rospy.sleep(.5)
        rospy.logwarn('Done.')
        

    def load(self):
        events = yaml.load(self.load_service(id='force_events',type='instructor_force_events').text)
        if events is not None:
            self.events = events
            for e in self.events.values():
                self.ui.select_event_combo.addItem(e.name.upper() + ' ('+str(e.force)+'N, '+e.direction+')')
        else:
            rospy.logwarn('No Events Found')

    def force_cb(self,msg):
        self.last_force_reading = msg
        self.msg_number = self.msg_number + 1
        # print self.msg_number
        if self.msg_number%5 == 0:
            if self.last_force_reading is not None:

                self.forces['+x'] = abs(msg.Fx) if msg.Fx < 0 else 0
                self.forces['-x'] = abs(msg.Fx) if msg.Fx >= 0 else 0
                self.forces['+y'] = abs(msg.Fy) if msg.Fy < 0 else 0
                self.forces['-y'] = abs(msg.Fy) if msg.Fy >= 0 else 0
                self.forces['+z'] = abs(msg.Fz) if msg.Fz > 0 else 0
                self.forces['-z'] = abs(msg.Fz) if msg.Fz <= 0 else 0

                self.interactionStyle.update_forces(self.forces)
                self.interactionStyle.update_arrows()
            else:
                print 'invalid force message'

    def zero_sensor(self):
        try:
            rospy.wait_for_service('/robotiq_force_torque_sensor_acc',2)
        except rospy.ROSException as e:
            rospy.logwarn('Could not find force sensor zero service')
            return

        try:
            zero_sensor_proxy = rospy.ServiceProxy('/robotiq_force_torque_sensor_acc',ft_srv.sensor_accessor)
            msg = ft_srv.sensor_accessorRequest()
            msg.command = "SET ZRO"
            result = zero_sensor_proxy(msg)
            rospy.logwarn(result)
        except rospy.ServiceException, e:
            rospy.logwarn(e)

    def reset_view(self):
        self.ren.ResetCamera()
        self.ren.GetActiveCamera().SetPosition(362, -298, 254)
        self.ren.GetActiveCamera().SetViewUp(0, 0, 1)
        self.ren.GetActiveCamera().SetFocalPoint(67.44, -8.655, -1.0075)
        self.ren.GetActiveCamera().SetClippingRange(.1,1000)
        self.vtkWidget.GetRenderWindow().Render()

    def create_ui(self):        
        ### CREATE UI AND LAYOUT
        self.ui_path = self.rp.get_path('instructor_core') + '/ui/force_event.ui'
        self.layout = QVBoxLayout()
        self.ui = QWidget()
        uic.loadUi(self.ui_path, self.ui)
        self.layout.addWidget(self.ui)
        self.setLayout(self.layout)
        ### CONNECTIONS AND DEFAULTS
        self.ui.reset_view_button.clicked.connect(self.reset_view)
        self.ui.swap_arrows_button.clicked.connect(self.swap_arrows)
        self.ui.swap_arrows_button.hide()
        self.ui.create_event_button.clicked.connect(self.create_event)
        self.ui.new_event_button.clicked.connect(self.new_event)
        self.ui.zero_sensor_button.clicked.connect(self.zero_sensor)
        self.ui.swap_arrows_button.setText('Switch to Force Detected')
        self.ui.swap_label.setText("FORCE APPLIED BY ROBOT")
        self.ui.swap_label.hide()
        self.ui.selected_arrow_label.setText('Force Direction: NONE')
        self.ui.event_widget.hide()
        self.ui.status_label.hide()
        self.pop_status.connect(self.pop_status_label)
        self.ui.test_checkbox.stateChanged.connect(self.enable_test)
        self.ui.select_event_combo.activated.connect(self.event_selected)
        self.ui.manage_widget.hide()
        self.ui.manage_update_button.clicked.connect(self.update_selected)
        self.ui.manage_delete_button.clicked.connect(self.delete_selected)
        self.ui.manage_cancel_button.clicked.connect(self.cancel_selected)
        ### CREATE RENDER WIDGET AND INTERACTOR
        self.render_frame = QtGui.QFrame()
        self.vl = QtGui.QVBoxLayout()
        self.vtkWidget = QVTKRenderWindowInteractor(self.render_frame)
        self.vl.addWidget(self.vtkWidget)
 
        ### CREATE RENDERER
        self.ren = vtk.vtkRenderer()
        self.vtkWidget.GetRenderWindow().AddRenderer(self.ren)
        self.interactionStyle = TrackBallCameraFacePicker(self.ui.selected_arrow_label,self.direction_selected)
        self.interactionStyle.SetDefaultRenderer(self.ren)
        self.vtkWidget.SetInteractorStyle(self.interactionStyle)
        self.vtkWidget.SetPicker(vtk.vtkAreaPicker())
        self.iren = self.vtkWidget.GetRenderWindow().GetInteractor()

        self.interactionStyle.update_forces(self.forces)

        ### Create Gripper
        gripper_reader = vtk.vtkSTLReader()
        gripper_reader.SetFileName(self.gripper_stl_path)         
        gripper_polydata = gripper_reader.GetOutput()
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(gripper_polydata)
        else:
            mapper.SetInputData(gripper_polydata)
        self.ee_actor = vtk.vtkActor()
        self.ee_actor.SetMapper(mapper)
        self.set_color(self.ee_actor,[.25,.25,.25])
        self.interactionStyle.Data = gripper_polydata   

        ### CREATE ARROWS
        self.arrows['+z'] = self.create_arrow([150,0,-10],[0,0,0])
        self.set_color(self.arrows['+z'],[.1,.3,.9])
        self.arrow_colors['+z'] = self.arrows['+z'].GetProperty().GetColor()
        self.arrows['-z'] = self.create_arrow([-50,0,-10],[0,0,180])
        self.set_color(self.arrows['-z'],[.1,.3,.9])
        self.arrow_colors['-z'] = self.arrows['-z'].GetProperty().GetColor()
        self.arrows['+x'] = self.create_arrow([75,100,-10],[0,0,90])
        self.set_color(self.arrows['+x'],[.9,.1,.1])
        self.arrow_colors['+x'] = self.arrows['+x'].GetProperty().GetColor()
        self.arrows['-x'] = self.create_arrow([75,-100,-10],[0,0,270])
        self.set_color(self.arrows['-x'],[.9,.1,.1])
        self.arrow_colors['-x'] = self.arrows['-x'].GetProperty().GetColor()
        self.arrows['+y'] = self.create_arrow([75,0,50],[0,270,0])
        self.set_color(self.arrows['+y'],[.1,.9,.1])
        self.arrow_colors['+y'] = self.arrows['+y'].GetProperty().GetColor()
        self.arrows['-y'] = self.create_arrow([75,0,-75],[0,90,0])
        self.set_color(self.arrows['-y'],[.1,.9,.1])
        self.arrow_colors['-y'] = self.arrows['-y'].GetProperty().GetColor()

        ### CREATE TEXT
        self.labels['+z'] = self.create_text_actor('N',[0,0,1])
        self.labels['-z'] = self.create_text_actor('N',[0,0,1])
        self.labels['+x'] = self.create_text_actor('N',[1,0,0])
        self.labels['-x'] = self.create_text_actor('N',[1,0,0])
        self.labels['+y'] = self.create_text_actor('N',[0,1,0])
        self.labels['-y'] = self.create_text_actor('N',[0,1,0])

        self.label_offsets['+x'] = [0,25,25]
        self.label_offsets['-x'] = [0,-35,25]
        self.label_offsets['+z'] = [40,0,25]
        self.label_offsets['-z'] = [-35,0,25]
        self.label_offsets['+y'] = [25,0,25]
        self.label_offsets['-y'] = [25,0,-25]

        ### ADD ACTORS
        self.ren.AddActor(self.arrows['+z'])
        self.ren.AddActor(self.arrows['-z'])
        self.ren.AddActor(self.labels['-z'])
        self.ren.AddActor(self.labels['+z'])
        self.ren.AddActor(self.arrows['+x'])
        self.ren.AddActor(self.arrows['-x'])
        self.ren.AddActor(self.labels['-x'])
        self.ren.AddActor(self.labels['+x'])
        self.ren.AddActor(self.arrows['+y'])
        self.ren.AddActor(self.arrows['-y'])
        self.ren.AddActor(self.labels['-y'])
        self.ren.AddActor(self.labels['+y'])

        self.interactionStyle.set_arrows(self.arrows)
        self.interactionStyle.set_labels(self.labels)
        self.interactionStyle.set_arrow_colors(self.arrow_colors)
        self.interactionStyle.set_label_offsets(self.label_offsets)
        self.ren.AddActor(self.ee_actor)
        
        ### SET UP RENDER ENVIRONMENT
        self.ren.SetBackground(.95,.95,.95)
        self.ren.ResetCamera()
        self.ren.GetActiveCamera().SetPosition(362, -298, 254)
        self.ren.GetActiveCamera().SetViewUp(0, 0, 1)
        self.ren.GetActiveCamera().SetFocalPoint(67.44, -8.655, -1.0075)
        self.ren.GetActiveCamera().SetClippingRange(.1,1000)
        self.render_frame.setLayout(self.vl)
        self.ui.graphics_layout.addWidget(self.render_frame)
        
        ### START RENDERER 
        self.swap_arrows()
        self.pop_status_label('STARTED SUCCESSFULLY')
        self.show()
        self.iren.Initialize()

        pass

    def hide_status_label(self):
        self.ui.status_label.hide()
        pass

    @QtCore.pyqtSlot(str,str)
    def pop_status_label(self,text,style='None'):
        if style != None:
            self.ui.status_label.setStyleSheet(style)
        else:
            self.ui.status_label.setStyleSheet('background-color:#2574A9; color:#ffffff')
        self.ui.status_label.setText(text)
        self.ui.status_label.show()
        QTimer.singleShot(3000, self.hide_status_label)

    def direction_selected(self,d):
        self.current_direction = d
        pass

    def new_event(self):
        self.cancel_selected()
        self.ui.event_widget.show()
        self.started_event = True
        self.ui.name_field.setText('')
        self.ui.force_spin.setValue(0)
        self.current_direction = None
        print 'Starting New Event'

    def event_selected(self,sid):
        self.cancel_selected()
        if sid is not 0:
            self.current_event_label = self.ui.select_event_combo.itemText(sid)
            self.current_event_name = str(self.current_event_label.split('(')[0]).strip(' ').lower()
            self.current_event = self.events[self.current_event_name]
            self.ui.event_widget.show()
            self.ui.manage_widget.show()
            self.ui.name_field.setText(self.current_event_name)
            self.ui.force_spin.setValue(int(self.current_event.force))
            self.interactionStyle.set_current_direction(self.current_event.direction)

    def update_selected(self):
        # update self.events and pop message but dont hide anything
        # might want to save here and elsewhere, like when creating an event
        pass

    def delete_selected(self):
        sid = self.ui.select_event_combo.findText(self.current_event_label)
        self.ui.select_event_combo.removeItem(sid)
        self.events.pop(self.current_event_name)
        self.pop_status.emit('DELETED EVENT: '+ str(self.current_event_name).upper(),'background-color:#5AD132; color:#ffffff')
        self.current_event_name = ''
        self.current_event = None
        self.current_event_label = ''
        self.cancel_selected()
        pass

    def cancel_selected(self):
        self.ui.manage_widget.hide()
        self.ui.event_widget.hide()
        self.ui.select_event_combo.setCurrentIndex(0)
        self.interactionStyle.set_current_direction(None)

    def create_event(self):
        if self.started_event == True:
            name = str(self.ui.name_field.text())
            force = float(self.ui.force_spin.value())

            if name == '':
                rospy.logerr('You must specify a name for the event.')
                self.pop_status.emit('No Name Specified','background-color:#B52A2A; color:#ffffff')
                return
            if force == 0:
                rospy.logerr('You must specify a force value.')
                self.pop_status.emit('No Force Specified','background-color:#B52A2A; color:#ffffff')
                return
            if self.current_direction == None:
                rospy.logerr('You must select a force direction.')
                self.pop_status.emit('No Direction Selected','background-color:#B52A2A; color:#ffffff')
                return

            ### Create force event
            print 'Crating Force Event'
            print '-- Name: '+ str(name)
            print '-- Force: '+ str(force)
            print '-- Direction: '+ str(self.current_direction)

            self.events[name] = ForceEvent(name.lower(),force,self.current_direction)
            self.ui.select_event_combo.addItem(name.upper() + ' ('+str(force)+'N, '+self.current_direction+')')
            self.pop_status.emit('CREATED EVENT: '+ str(name).upper(),'background-color:#5AD132; color:#ffffff')
            self.ui.event_widget.hide()
            self.stop_test()
        else:
            print 'No Event Started'

    def enable_test(self,v):
        if v == 2:
            # self.interactionStyle.set_flash(True)
            self.test_timer = QTimer(self)
            self.connect(self.test_timer, SIGNAL("timeout()"), self.test)
            self.test_timer.start(100)
        else:
            self.test_timer.stop()
            # self.interactionStyle.set_flash(False)

    def stop_test(self):
        self.test_timer.stop()
        self.ui.test_checkbox.setCheckState(Qt.Unchecked)

    def test(self):
        force = float(self.ui.force_spin.value())
        if force == 0:
            self.pop_status.emit('No Force Specified','background-color:#B52A2A; color:#ffffff')
            self.stop_test()
        elif self.current_direction == None:    
            self.pop_status.emit('No Direction Specified','background-color:#B52A2A; color:#ffffff')
            self.stop_test()
        else:
            # print '34BFB6'
            if self.forces[self.current_direction] < 1.5:
                self.ui.test_label.setText('PUSH')
                self.ui.test_label.setStyleSheet('background-color:#2979B3; color:#ffffff')
                self.set_color(self.interactionStyle.selected_arrow,[41/255.0,121/255.0,179/255.0])
            elif self.forces[self.current_direction] > float(self.ui.force_spin.value()):
                self.ui.test_label.setText('PASS!')
                self.ui.test_label.setStyleSheet('background-color:#5AD132; color:#ffffff')
                self.set_color(self.interactionStyle.selected_arrow,[90/255.0,209/255.0,50/255.0])
            else:
                self.ui.test_label.setText('ALMOST...')
                self.ui.test_label.setStyleSheet('background-color:#34BFB6; color:#ffffff')
                self.set_color(self.interactionStyle.selected_arrow,[52/255.0,191/255.0,182/255.0])

    ### VTK METHODS ###
    def create_text_actor(self,text,color):
        A = vtk.vtkTextActor()
        A.SetInput(text)
        A.GetTextProperty().SetFontFamilyToArial()
        A.GetTextProperty().SetFontSize(18)
        A.GetTextProperty().SetColor(color)
        return A

    def set_color(self,a,c):
        a.GetProperty().SetColor(c)
        a.GetProperty().SetDiffuse(4)
        a.GetProperty().SetSpecular(0.9)
        a.GetProperty().SetSpecularPower(20)

    def set_opacity(self,a,v):
        a.GetProperty().SetOpacity(v)
        pass

    def swap_arrows(self):
        if self.reversed == False:
            self.arrows['+z'].SetPosition([-125,0,-10])
            self.arrows['-z'].SetPosition([225,0,-10])
            self.arrows['+x'].SetPosition([75,-175,-10])
            self.arrows['-x'].SetPosition([75,175,-10])
            self.arrows['+y'].SetPosition([75,0,-150])
            self.arrows['-y'].SetPosition([75,0,125])
            self.ui.swap_arrows_button.setText('Switch to Force Applied')
            self.ui.swap_label.setText("FORCE DETECTED BY ROBOT")
            self.vtkWidget.GetRenderWindow().Render()
            self.reversed = True
        else:
            self.arrows['+z'].SetPosition([150,0,-10])
            self.arrows['-z'].SetPosition([-50,0,-10])
            self.arrows['+x'].SetPosition([75,100,-10])
            self.arrows['-x'].SetPosition([75,-100,-10])
            self.arrows['+y'].SetPosition([75,0,50])
            self.arrows['-y'].SetPosition([75,0,-75])
            self.vtkWidget.GetRenderWindow().Render()
            self.ui.swap_label.setText("FORCE APPLIED BY ROBOT")
            self.ui.swap_arrows_button.setText('Switch to Force Detected')
            self.reversed = False

    def create_arrow(self,p,q):
        arrowSource = vtk.vtkArrowSource()
        arrowSource.SetTipLength(.5)
        arrowSource.SetTipRadius(.2)
        arrowSource.SetShaftRadius(.075)
        arrowSource.SetShaftResolution(20)
        arrowSource.SetTipResolution(20)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(arrowSource.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.SetScale(75)
        actor.SetPosition(p)
        actor.SetOrientation(q)
        return actor

    ### ROS LOOP AND CLOSE ###
    def check_ok(self):
        self.update_predicates()
        if rospy.is_shutdown():
          self.close()
          self.app_.exit()

    def closeEvent(self, event):
        print 'Closing...'
        self.save()
        event.accept()
        print 'Finished.'

### MAIN LOOP ##########################################################
if __name__ == '__main__':
    app = QApplication( sys.argv )
    test_widget = ForceEventCreator(app)
    test_widget.show()
    app.exec_()


