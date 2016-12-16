#!/usr/bin/env python

import sys
from PyQt4 import QtGui
from vtk import *
from vtk.qt4 import QVTKRenderWindowInteractor  

class Viewer(QtGui.QMainWindow):
    def __init__(self):
        super(Viewer, self).__init__()
        
        self.initUI()
        
    def initUI(self):
        
        self.textEdit = QtGui.QTextEdit()
        #self.setCentralWidget(self.textEdit)
        self.statusBar()
        
        #This will be a screenshot button and I need to find a .png for it soon..
        exitAction = QtGui.QAction(QtGui.QIcon('icons/exit24.png'), 'Exit', self)
        exitAction.setShortcut('Ctrl+Q')
        exitAction.setStatusTip('Exit application')
        exitAction.triggered.connect(self.close)
        #exitAction.triggered.connect(QtGui.qApp.quit)
        
        #self.statusBar()
        openFile = QtGui.QAction(QtGui.QIcon('icons/open.png'), 'Open', self)
        openFile.setShortcut('Ctrl+O')
        openFile.setStatusTip('Open STL File')
        openFile.triggered.connect(self.showDialog)
        
        menubar = self.menuBar()
        fileMenu = menubar.addMenu('&File')
        fileMenu.addAction(exitAction)
        fileMenu.addAction(openFile)
        
        #Graphically, toolbars are nice but we don't need them right now..
        toolbar = self.addToolBar('Exit')
        toolbar.addAction(exitAction)
        toolbar.addAction(openFile)
        
        self.setGeometry(300, 300, 450, 350)
        self.setWindowTitle ('STL Viewer')
        self.show()
        
    def showDialog(self):

        fname = QtGui.QFileDialog.getOpenFileName(self, 'Open file', '/home', 'STL *.stl')
        
        f = open(fname, 'r')
        
        with f:
            data = f.read()
            #self.textEdit.setText(data)
            self.vtkView(str(fname))
            
    def vtkView(self, filename):
        #filename = "STL_Viewer/motor_mount_v2.STL"
         
        reader = vtk.vtkSTLReader()
        reader.SetFileName(filename)
         
        polydata = reader.GetOutput()
         
        # Setup actor and mapper
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(polydata)
        else:
            mapper.SetInputData(polydata)
         
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
         
        # Setup render window, renderer, and interactor
        renderer = vtk.vtkRenderer()
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        renderer.AddActor(actor)
        renderWindow.Render()
        renderWindowInteractor.Start()

def main():
    
    app = QtGui.QApplication(sys.argv)
    ex = Viewer()
    sys.exit(app.exec_())
        
if __name__ == '__main__':
    main()