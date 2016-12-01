#!/usr/bin/env python

from PyQt4 import QtGui
from PyQt4.QtGui import QWidget
from vtk.qt4.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
import sys

import time
import numpy as np
import pcl
# from pyhull.convex_hull import ConvexHull
# RubberBandPointPicker
# from http://www.cmake.org/Wiki/VTK/Examples/Cxx/Picking/HighlightSelectedPoints
#
#
class RubberBandFacePicker(vtk.vtkInteractorStyleRubberBandPick):
  def __init__(self,parent=None):
    print "Press 'r' to toggle selection mode"
    self.AddObserver("LeftButtonReleaseEvent",self.leftButtonReleasedEvent)
    self.selectedMapper = vtk.vtkDataSetMapper()
    self.selectedActor = vtk.vtkActor()   
    self.selectedActor.SetMapper(self.selectedMapper)
    self.selected_pt_color = [1.0, 0.0, 0.0]
    self.saved_selection_points = []
    self.saved_num_pts = 0

  def leftButtonReleasedEvent(self,obj,event):
    self.OnLeftButtonUp()

    frustum = self.GetInteractor().GetPicker().GetFrustum()

    extractGeometry = vtk.vtkExtractGeometry()
    extractGeometry.SetImplicitFunction(frustum)
    extractGeometry.SetInput(0, self.Data)
    extractGeometry.Update()

    # reduce extracted geometry to points only
    glyphFilter = vtk.vtkVertexGlyphFilter()
    glyphFilter.SetInputConnection(extractGeometry.GetOutputPort());
    glyphFilter.Update();

    selected = glyphFilter.GetOutput();
    
    if selected.GetNumberOfPoints() == 0:
      print 'debug: none selected'
      pass
    elif selected.GetNumberOfPoints() == self.saved_num_pts:
      print 'debug: same points'
      pass
    else:
      print 'debug: new points'
      self.saved_selection_points = []
      for i in xrange(0, selected.GetNumberOfPoints()):
        self.saved_selection_points.append(list(selected.GetPoint(i)))
        self.saved_num_pts = selected.GetNumberOfPoints()
    #   print selected.GetPoint(i)
    # print "Selected ", selected.GetNumberOfPoints(), " points."
    # print "Selected ", selected.GetNumberOfCells(), " cells."
      # fitter = RANSACFitter()
      # model, inliers, projected_inliers, min_sd_point, max_sd_point = fitter.fitLine(self.saved_selection_points, 0.0025)

      # self.Widget.remove_all_displays()
      # self.Widget.display_cloud("inlier", projected_inliers, 10, [0,0,1])

      # display cylinder
      # cyl_center = np.asarray(max_sd_point) + 0.5 * (np.asarray(min_sd_point) - np.asarray(max_sd_point))
      # cyl_height = np.linalg.norm((np.asarray(min_sd_point) - np.asarray(max_sd_point)))
      # self.Widget.display_cylinder("cyl", list(cyl_center), [model[3], model[4], model[5]], cyl_height, model[6], [0,0,1])

      # display line

      # self.Widget.display_sphere("start", min_sd_point, 0.005, [0,0,1])
      # self.Widget.display_sphere("end", max_sd_point, 0.005, [0,1,0])
      # self.Widget.display_arrow("arrow", min_sd_point, [model[3], model[4], model[5]], 0.3, 0.001, [1,0,0])
      # self.Widget.display_arrow("arrow", center, [model[0], model[1], model[2]], 0.05, 0.001,[0,0,1])

    self.selectedMapper.SetInput(selected);
    self.selectedMapper.ScalarVisibilityOff();

    self.selectedActor.GetProperty().SetColor(self.selected_pt_color[0], self.selected_pt_color[1], self.selected_pt_color[2]); # (R,G,B)
    self.selectedActor.GetProperty().SetPointSize(5);

    self.GetInteractor().GetRenderWindow().GetRenderers().GetFirstRenderer().AddActor(self.selectedActor);
    self.GetInteractor().GetRenderWindow().Render();

    return

  def set_selected_pt_color(self,c):
    self.selected_pt_color = c
    self.selectedActor.GetProperty().SetColor(self.selected_pt_color[0], self.selected_pt_color[1], self.selected_pt_color[2]); # (R,G,B)

  def get_selected_vertices(self):
    return self.saved_selection_points


# TrackBallFacePicking
# from http://www.cmake.org/Wiki/VTK/Examples/Cxx/Picking/CellPicking
#
#
class TrackBallCameraFacePicker(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self,parent=None):
            self.AddObserver("LeftButtonPressEvent",self.leftButtonPressEvent)
            self.selectedMapper = vtk.vtkDataSetMapper()
            self.selectedActor = vtk.vtkActor()

    def leftButtonPressEvent(self,obj,event):
        # obj is a vtkInteractorStyleTrackballCamera
        # event is just a string
        pos = self.GetInteractor().GetEventPosition()
        print pos

        picker = vtk.vtkCellPicker()
        picker.SetTolerance(0.0005)
        picker.Pick(pos[0], pos[1], 0, self.GetDefaultRenderer())
        worldPosition = picker.GetPickPosition()

        if(picker.GetCellId() != -1):
            # clicked on object
            # print worldPosition
            ids = vtk.vtkIdTypeArray()
            ids.SetNumberOfComponents(1)
            ids.InsertNextValue(picker.GetCellId())

            selectionNode = vtk.vtkSelectionNode()
            selectionNode.SetFieldType(vtk.vtkSelectionNode.CELL)
            selectionNode.SetContentType(vtk.vtkSelectionNode.INDICES)
            selectionNode.SetSelectionList(ids)
            
            selection = vtk.vtkSelection()
            selection.AddNode(selectionNode)

            extractSelection = vtk.vtkExtractSelection()
            extractSelection.SetInput(0, self.Data)
            extractSelection.SetInput(1, selection)
            extractSelection.Update()

            selected = vtk.vtkUnstructuredGrid()
            selected = extractSelection.GetOutput()

            # points in object frame
            # print "NumP points: ", selected.GetNumberOfPoints()
            # for i in xrange(0, selected.GetNumberOfPoints()):
            #     print selected.GetPoint(i)

            self.selectedMapper.SetInputConnection(selected.GetProducerPort())

            self.selectedActor.SetMapper(self.selectedMapper)
            self.selectedActor.GetProperty().EdgeVisibilityOn()
            self.selectedActor.GetProperty().SetEdgeColor(1,0,0);
            self.selectedActor.GetProperty().SetLineWidth(3);

            self.GetInteractor().GetRenderWindow().GetRenderers().GetFirstRenderer().AddActor(self.selectedActor);
            self.GetInteractor().GetRenderWindow().Render();

        self.OnLeftButtonDown()
        return

class PickWidget(QWidget): 
  def __init__(self,parent = None):
    QWidget.__init__(self, parent)
    self.setStyleSheet('background-color:#333333')
    self.vtk_layout = QtGui.QGridLayout(self)
    self.vtk_widget = QVTKRenderWindowInteractor(self)
    self.vtk_layout.addWidget(self.vtk_widget)
    self.ren = vtk.vtkRenderer()
    self.vtk_widget.GetRenderWindow().AddRenderer(self.ren)
    self.meshMapper = vtk.vtkPolyDataMapper()    
    self.meshActor = vtk.vtkActor()
    self.meshActor.SetMapper(self.meshMapper)
    self.ren.AddActor(self.meshActor)
    self.interactionStyle = RubberBandFacePicker()
    self.interactionStyle.SetDefaultRenderer(self.ren)
    self.vtk_widget.SetInteractorStyle(self.interactionStyle)
    self.vtk_widget.SetPicker(vtk.vtkAreaPicker())
    self.iren = self.vtk_widget.GetRenderWindow().GetInteractor()
    self.show()
    self.iren.Initialize()

    # name2actor map for calls to display_XXXX
    self.name2actor = {}

  def start(self):
    self.reader = vtk.vtkOBJReader()
    self.reader.SetFileName(self.filename)
    self.meshMapper.SetInputConnection(self.reader.GetOutputPort())
    self.interactionStyle.Data = self.reader.GetOutput()
    self.interactionStyle.Widget = self

  def set_source(self,filename):
    self.filename = filename

  def set_selected_color(self,color):
    self.interactionStyle.set_selected_pt_color(color)

  def get_selected_vertices(self):
    V = self.interactionStyle.get_selected_vertices()
    return V

  def display_cloud(self, actorName, list_points, size, vec3_color):
    # extract inlier points
    vtk_points = vtk.vtkPoints()
    for p in list_points:
      vtk_points.InsertNextPoint(p[0], p[1], p[2])

    temp_cloud = vtk.vtkPolyData()
    temp_cloud.SetPoints(vtk_points)

    # add 0D topology to every point
    glfilter = vtk.vtkVertexGlyphFilter()
    glfilter.SetInput(temp_cloud)
    glfilter.Update()
    cloud = glfilter.GetOutput()

    cloudMapper = vtk.vtkPolyDataMapper()
    cloudMapper.SetInput(cloud)

    actor = vtk.vtkActor()
    actor.SetMapper(cloudMapper)
    actor.GetProperty().SetColor(vec3_color[0],vec3_color[1],vec3_color[2])
    actor.GetProperty().SetPointSize(size);
    self.ren.AddActor(actor);
    self.name2actor[actorName] = actor
    pass

  def display_sphere(self, actorName, vec3_center, radius, vec3_color):
    sphere = vtk.vtkSphereSource()
    sphere.SetRadius(radius)
    sphere.SetCenter(vec3_center)
    sphere.SetThetaResolution(40)
    sphere.SetPhiResolution(40)
    sphere.SetCenter(vec3_center[0], vec3_center[1], vec3_center[2])
    sphere.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(sphere.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(vec3_color[0],vec3_color[1],vec3_color[2])
    self.ren.AddActor(actor);
    self.name2actor[actorName] = actor

    pass

  def display_cylinder(self, actorName, vec3_origin, vec3_direction, height, radius, vec3_color):
    cyl = vtk.vtkCylinderSource()
    cyl.SetRadius(radius)
    cyl.SetHeight(height)
    cyl.SetCapping(1)
    cyl.Update()

    # transform to center with orientation according to normal
    axis1 = [vec3_direction[0], vec3_direction[1], vec3_direction[2]]
    axis2 = np.cross(axis1, [1, 1, 1])
    axis2 = axis2 / np.linalg.norm(axis2)
    axis3 = np.cross(axis2, axis1)
    # print axis1
    # print axis2
    # print axis3
    trans = np.eye(4)
    trans[0,0] = axis2[0];     trans[0,1] = axis1[0];      trans[0,2] = axis3[0];      trans[0,3] = vec3_origin[0];
    trans[1,0] = axis2[1];     trans[1,1] = axis1[1];      trans[1,2] = axis3[1];      trans[1,3] = vec3_origin[1];
    trans[2,0] = axis2[2];     trans[2,1] = axis1[2];      trans[2,2] = axis3[2];      trans[2,3] = vec3_origin[2];
    trans[3,0] = 0;            trans[3,1] = 0;             trans[3,2] = 0;             trans[3,3] = 1;

    vtk_trans = vtk.vtkMatrix4x4()
    for i in range(0,4):
      for j in range(0,4):
          vtk_trans.SetElement(i,j,trans[i,j])

    ar_trans = vtk.vtkTransform()
    ar_trans.SetMatrix(vtk_trans)
    ar_trans_filter = vtk.vtkTransformPolyDataFilter()
    ar_trans_filter.SetTransform(ar_trans)
    ar_trans_filter.SetInputConnection(cyl.GetOutputPort())

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(ar_trans_filter.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(vec3_color[0],vec3_color[1],vec3_color[2])
    self.ren.AddActor(actor);
    self.name2actor[actorName] = actor
    return 

  def display_disk(self, actorName, vec3_origin, vec3_direction, outer_radius, inner_radius, vec3_color):
    disk = vtk.vtkDiskSource()
    disk.SetInnerRadius(inner_radius)
    disk.SetOuterRadius(outer_radius)
    disk.SetRadialResolution(30)
    disk.SetCircumferentialResolution(30)
    disk.Update()

    # transform to center with orientation according to normal
    axis1 = [vec3_direction[0], vec3_direction[1], vec3_direction[2]]
    axis2 = np.cross(axis1, [1, 1, 1])
    axis2 = axis2 / np.linalg.norm(axis2)
    axis3 = np.cross(axis1, axis2)
    # print axis1
    # print axis2
    # print axis3
    trans = np.eye(4)
    trans[0,0] = axis3[0];     trans[0,1] = axis2[0];      trans[0,2] = axis1[0];      trans[0,3] = vec3_origin[0];
    trans[1,0] = axis3[1];     trans[1,1] = axis2[1];      trans[1,2] = axis1[1];      trans[1,3] = vec3_origin[1];
    trans[2,0] = axis3[2];     trans[2,1] = axis2[2];      trans[2,2] = axis1[2];      trans[2,3] = vec3_origin[2];
    trans[3,0] = 0;            trans[3,1] = 0;             trans[3,2] = 0;             trans[3,3] = 1;

    vtk_trans = vtk.vtkMatrix4x4()
    for i in range(0,4):
      for j in range(0,4):
          vtk_trans.SetElement(i,j,trans[i,j])

    ar_trans = vtk.vtkTransform()
    ar_trans.SetMatrix(vtk_trans)
    ar_trans_filter = vtk.vtkTransformPolyDataFilter()
    ar_trans_filter.SetTransform(ar_trans)
    ar_trans_filter.SetInputConnection(disk.GetOutputPort())

    diskMapper = vtk.vtkPolyDataMapper()
    diskMapper.SetInputConnection(ar_trans_filter.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(diskMapper)
    actor.GetProperty().SetColor(vec3_color[0],vec3_color[1],vec3_color[2])
    self.ren.AddActor(actor);
    self.name2actor[actorName] = actor
    return 

  def display_arrow(self, actorName, vec3_origin, vec3_direction, length_in_m, shaft_radius, vec3_color):
    # visualize direction arrow
    arrow = vtk.vtkArrowSource()
    arrow.SetTipLength(0.25)
    arrow.SetTipRadius((1.5 * shaft_radius) / length_in_m)
    arrow.SetTipResolution(20)
    arrow.SetShaftRadius(shaft_radius / length_in_m) # account for scaling
    arrow.SetShaftResolution(20)
    arrow.Update()

    # scale 
    scale = vtk.vtkTransform()
    scale.Scale(length_in_m,length_in_m,length_in_m)
    scale_filter = vtk.vtkTransformPolyDataFilter()
    scale_filter.SetTransform(scale)
    scale_filter.SetInputConnection(arrow.GetOutputPort())

    # transform to center with orientation according to normal
    axis1 = [vec3_direction[0], vec3_direction[1], vec3_direction[2]]
    axis2 = np.cross(axis1, [1, 1, 1])
    axis2 = axis2 / np.linalg.norm(axis2)
    axis3 = np.cross(axis1, axis2)
    # print axis1
    # print axis2
    # print axis3
    trans = np.eye(4)
    trans[0,0] = axis1[0];     trans[0,1] = axis2[0];      trans[0,2] = axis3[0];      trans[0,3] = vec3_origin[0];
    trans[1,0] = axis1[1];     trans[1,1] = axis2[1];      trans[1,2] = axis3[1];      trans[1,3] = vec3_origin[1];
    trans[2,0] = axis1[2];     trans[2,1] = axis2[2];      trans[2,2] = axis3[2];      trans[2,3] = vec3_origin[2];
    trans[3,0] = 0;            trans[3,1] = 0;             trans[3,2] = 0;             trans[3,3] = 1;

    vtk_trans = vtk.vtkMatrix4x4()
    for i in range(0,4):
      for j in range(0,4):
          vtk_trans.SetElement(i,j,trans[i,j])

    ar_trans = vtk.vtkTransform()
    ar_trans.SetMatrix(vtk_trans)
    ar_trans_filter = vtk.vtkTransformPolyDataFilter()
    ar_trans_filter.SetTransform(ar_trans)
    ar_trans_filter.SetInputConnection(scale_filter.GetOutputPort())
    
    arrowMapper = vtk.vtkPolyDataMapper()
    arrowMapper.SetInputConnection(ar_trans_filter.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(arrowMapper)
    actor.GetProperty().SetColor(vec3_color[0],vec3_color[1],vec3_color[2])
    self.ren.AddActor(actor);
    self.name2actor[actorName] = actor
    pass

  def display_affordance_primitive(self,g,type_):
    self.remove_all_displays()
    if type_ == 'axis':
      self.display_arrow('p',g.base,g.direction,g.length,g.radius,[1,.2,0])
    elif type_ == 'plane':
      self.display_disk('p',g.center,g.normal,g.radius,0,[1,.2,0])
      self.display_arrow('p',g.center,g.normal,.1,.002,[1,.3,0])


  def remove_display(self, actorName):
    actor = self.name2actor.get(actorName)
    if(actor != None):
      self.ren.RemoveActor(actor)
      self.name2actor.pop(actorName)
    pass

  def remove_all_displays(self):
    for actor in self.name2actor.viewvalues():
      self.ren.RemoveActor(actor)
    self.name2actor.clear()
    pass

if __name__ == '__main__':
    # sac = RANSACFitter()
    # sac.testPlaneFit()

    # every QT app needs an app
    app = QtGui.QApplication(['QVTKRenderWindowInteractor'])

    pickWidget = PickWidget()
    pickWidget.set_source('/home/sebastian/hydro_rosbuild_overlay/scene_parser_models/models/drill_w_collar/drill_w_collar.obj')
    pickWidget.start()

    pickWidget.display_arrow("arrow", [1,0,0], [0,1,0], 0.3, 0.005, [0,1,0])
    pickWidget.display_cylinder("cyl", [1,0,0], [0,1,0], 0.3, 0.01, [0,0,1])

    # start event processing
    sys.exit(app.exec_())