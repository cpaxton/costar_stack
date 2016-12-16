#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('instructor_core')
import rospy
import os, sys, yaml
# import copy
# import rviz
import rospkg
from librarian_msgs.msg import *
from librarian_msgs.srv import *
# Qt
from PyQt4 import QtGui, QtCore, uic
from PyQt4.QtGui import *
from PyQt4.QtCore import *
# CV
import optparse
from predicator_msgs.msg import *
from pyKinectTools.utils.pointcloud_conversions import *
from pyKinectTools.utils.transformations import *
from sensor_msgs.msg import PointCloud2
import cv2
# 
import numpy as np
import scipy.ndimage as nd



class SnapshotState:

    def __init__(self, max_depth, mask_thresh, pct_in_goal=0.5):

        # Params
        self.max_depth = max_depth
        self.mask_thresh = mask_thresh
        self.pct_in_goal = pct_in_goal
        # Predicates
        self.matching_goal = False

        # Bounding box (only look at mask in this region)
        self.bbox_topleft = None
        self.bbox_bottomright = None            

        self.is_fit = False

    def update_threshold(self, new_thresh):
        self.mask_thresh = new_thresh

    def set_depth(self,d):
        self.max_depth = d

    def set_thresh(self,t):
        self.mask_thresh = t

    def set_pct_in_goal(self,p):
        self.pct_in_goal = p

    def fit(self, im_reference, im_goal):
        max_depth = self.max_depth
        mask_thresh = self.mask_thresh

        # np.save(os.path.expanduser("~/im_reference"), im_reference)
        # np.save(os.path.expanduser("~/im_goal"), im_goal)

        # Set NaNs and things far away to max distance
        im_reference = np.nan_to_num(im_reference)
        im_goal = np.nan_to_num(im_goal)

        mask_bg_reference = -np.logical_or(im_reference[:,:,2]>=max_depth, im_reference[:,:,2]==0)
        mask_bg_goal = -np.logical_or(im_goal[:,:,2]>=max_depth, im_goal[:,:,2]==0)

        im_reference = im_reference*mask_bg_reference[:,:,None] + max_depth*(-mask_bg_reference[:,:,None])
        im_goal = im_goal*mask_bg_goal[:,:,None] + max_depth*(-mask_bg_goal[:,:,None])

        # Find difference between states/images
        diff = np.sqrt(np.sum((im_reference-im_goal)**2, -1))
        mask = diff > mask_thresh
        mask *= -np.logical_or(-mask_bg_reference, -mask_bg_goal)
        
        # Filter out background noise
        mask = nd.median_filter(mask, 15)

        # Determine which parts of the difference masks belong to reference/goal state
        # mask_reference_foreground = (im_reference[:,:,2] < im_goal[:,:,2])*mask
        mask_goal_foreground = (im_reference[:,:,2] >= im_goal[:,:,2])*mask

        # Set class vars
        # self.im_reference = im_reference
        self.im_goal = im_goal        
        self.mask = mask
        # self.mask_reference_foreground = mask_reference_foreground
        self.mask_goal_foreground = mask_goal_foreground
        self.is_fit = True

    def transform(self, im_xyz, im_rgb=None):
        pct_in_goal = self.pct_in_goal
        max_depth = self.max_depth
        mask_thresh = self.mask_thresh
        # im_reference = self.im_reference
        im_goal = self.im_goal

        # Set NaNs and things far away to max distance
        im_pos = np.nan_to_num(im_xyz)
        mask_bg_current = -np.logical_or(im_pos[:,:,2]>=max_depth, im_pos[:,:,2]==0)
        im_pos = im_pos*mask_bg_current[:,:,None] + max_depth*(-mask_bg_current[:,:,None])

        # Distance from new image to 'reference' state
        # diff_reference = np.sqrt(np.sum((im_pos-im_reference)**2, -1))
        # mask_reference = (diff_reference < mask_thresh)

        # Distance from new image to 'After' state
        diff_goal = np.sqrt(np.sum((im_pos-im_goal)**2, -1))
        mask_goal = (diff_goal < mask_thresh) 

        # Only show the part of the mask that belongs to the corresonding state
        # mask_reference *= self.mask_reference_foreground*mask_bg_current
        mask_goal *= self.mask_goal_foreground*mask_bg_current

        # Apply bounding box region if available
        if self.bbox_topleft is not None:
            y1, x1 = self.bbox_topleft
            y2, x2 = self.bbox_bottomright
            inside_score = mask_goal[y1:y2, x1:x2].sum()
            total_score = self.mask_goal_foreground[y1:y2, x1:x2].sum()

            mask_bbox = np.zeros_like(mask_goal)
            mask_bbox[y1:y2, x1:x2] = 1

        else:
            inside_score = mask_goal.sum()
            total_score = self.mask_goal_foreground.sum()
        score = float(inside_score) / total_score

        # Set predicates
        self.matching_goal = False
        # if np.sum(mask_goal) > np.sum(mask_reference): 
        if score > pct_in_goal:
            self.matching_goal = True

        # Only visualize if an RGB image is input
        if im_rgb is None:
            return
        im_out = im_rgb

        # Viz: faint overlay of goal state
        im_out[self.mask_goal_foreground] /= 3.
        im_out[:,:,1][self.mask_goal_foreground] += 255*1/3
        if self.bbox_topleft:
            im_out[-mask_bbox] /= 2

        # Viz: highlight the activated component of the goal state
        if self.matching_goal:
            # Turn highlighted region green
            if self.bbox_topleft is not None:
                im_out[mask_goal*mask_bbox] = [0,255,0]
            else:           
                im_out[mask_goal] = [0,255,0]

            # Add border around image
            im_out[:10,:] = [0, 255, 0]
            im_out[-10:,:] = [0, 255, 0]
            im_out[:,-10:] = [0, 255, 0]
            im_out[:,:10] = [0, 255, 0]

        # Ensure contiguous and uint8
        im_out = np.ascontiguousarray(im_out[:,:,[2,1,0]].astype(np.uint8))

        return im_out


class SnapshotEventCreator(QWidget):
    pop_status = QtCore.pyqtSignal(str,str)

    # Bounding box information
    bbox_topleft = None
    bbox_bottomright = None

    def mousePressEvent(self, QMouseEvent):
        cursor = QtGui.QCursor()
        relative_pos = self.ui.difference_vis.mapFromGlobal(cursor.pos())
        # image_size = [self.ui.difference_vis.height(), self.ui.difference_vis.width()]
        # mouse_point = [float(relative_pos.y()), float(relative_pos.x())]
        mouse_point = [relative_pos.y()*2, relative_pos.x()*2]
        # rospy.logwarn(image_size)
        # mouse_point[0] =  mouse_point[0]/image_size[0]
        # mouse_point[1] =  mouse_point[1]/image_size[1]
        # rospy.logwarn(mouse_point)

        # Reset the bounding box and fill the top left position
        self.bbox_topleft = mouse_point
        self.bbox_bottomright = None

    def mouseReleaseEvent(self, QMouseEvent):
        cursor = QtGui.QCursor()
        relative_pos = self.ui.difference_vis.mapFromGlobal(cursor.pos())
        # image_size = [self.ui.difference_vis.height(), self.ui.difference_vis.width()]
        # mouse_point = [float(relative_pos.y()), float(relative_pos.x())]
        mouse_point = [relative_pos.y()*2, relative_pos.x()*2]
        # mouse_point[0] =  mouse_point[0]/image_size[0]
        # mouse_point[1] =  mouse_point[1]/image_size[1]
        rospy.logwarn("TODO: [colin or kel] convert current mouse position to relative position within the (scaled) image")
        # rospy.logwarn("TODO: [colin] create mask within Snapshot")
        # rospy.logwarn("TODO: [kel] create button/interface in QT. And sliders for max depth + difference threshold")

        # fill the bottom right position
        self.bbox_bottomright = mouse_point

    def pointcloud_callback(self,data):
        max_distance = 3
        img_pos_raw = pointcloud2_to_array(data, split_rgb=True)
        im_xyz = np.dstack([img_pos_raw['x'], img_pos_raw['y'], img_pos_raw['z']]).copy()
        self.im_xyz = np.nan_to_num(im_xyz)

        # Convert depth to RGB-like image
        im_pos = self.im_xyz[:,:,2].copy()
        im_pos = np.clip(im_pos/float(max_distance)*255., 0, 255)
        im_pos = im_pos.astype(np.uint8)
        im_pos = np.repeat(im_pos[:,:,None], 3, 2)
        self.im_pos = im_pos

        im_rgb = np.dstack([img_pos_raw['r'], img_pos_raw['g'], img_pos_raw['b']])
        self.im_rgb = cv2.cvtColor(im_rgb, cv2.COLOR_BGR2RGB)

        height, width, byteValue = self.im_rgb.shape
        byteValue = byteValue * width

        # self.reference_image = QImage(self.im_rgb, width, height, byteValue, QImage.Format_RGB888).rgbSwapped()
        self.current_image = QImage(self.im_pos, width, height, byteValue, QImage.Format_RGB888)
        self.reference_image = QImage(self.im_pos, width, height, byteValue, QImage.Format_RGB888)
        self.goal_image = QImage(self.reference_image)

    def __init__(self,app):
        QWidget.__init__(self)
        rospy.init_node('snapshot_event',anonymous=True)
        self.app_ = app

        self.rp = rospkg.RosPack()
        i_path = self.rp.get_path('instructor_core')
        self.started_event = True
        self.events = {}
        self.reference_paused = False
        self.goal_paused = False
        self.test_timer = QTimer()
        self.reference_image = QImage()
        self.goal_image = QImage()
        self.reference_image_paused = QImage()
        self.goal_image_paused = QImage()
        self.difference_image = QImage()

        # Predicate values
        self.matching_goal = False

        # Matching threshold (in meters)
        # TODO: ADD SLIDER IN QT
        self.matching_threshold = .1
        self.max_depth = 3.0
        self.pct_in_goal = 0.5
        self.snapshot = None
        self.snapshots = {}
        self.index = 0
        
        ### Set up librarian
        rospy.logwarn('Snapshot Event Creator Waiting for Librarian Services')
        rospy.wait_for_service('/librarian/add_type',5)
        self.set_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)
        self.set_type_service('instructor_snapshot_events')
        rospy.logwarn('Librarian Proxies Created')

        ### Set Up Camera
        parser = optparse.OptionParser()
        parser.add_option("-c", "--camera", dest="camera",
                          help="name of camera", default="camera")
        parser.add_option("-n", "--namespace", dest="namespace",
                          help="namespace for occupancy data", default="")    
        (options, args) = parser.parse_args()
        camera_name = options.camera
        namespace = options.namespace
        cloud_uri = "/{}/depth_registered/points".format(camera_name)
        rospy.Subscriber(cloud_uri, PointCloud2, self.pointcloud_callback, queue_size=10)

        ### Set up predicator
        self.pub_list = rospy.Publisher('/predicator/input', PredicateList)
        self.pub_valid = rospy.Publisher('/predicator/valid_input', ValidPredicates)
        rospy.sleep(.5)
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = []
        pval.assignments = ['snapshot_events']
        self.pub_valid.publish(pval)

        ### Create UI
        self.create_ui()

        ### Start OK Check Timer
        self.ok_timer = QTimer(self)
        self.connect(self.ok_timer, SIGNAL("timeout()"), self.check_ok)
        self.ok_timer.start(100)

        # IPython.embed()



    def create_ui(self):        
        ### CREATE UI AND LAYOUT
        self.ui_path = self.rp.get_path('instructor_core') + '/ui/snapshot.ui'
        self.layout = QVBoxLayout()
        self.ui = QWidget()
        uic.loadUi(self.ui_path, self.ui)
        self.layout.addWidget(self.ui)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)
        self.ui.show()

        ### CONNECTIONS AND DEFAULTS
        self.ui.capture_before_btn.setText('CLICK TO CAPTURE')
        self.ui.capture_after_btn.setText('CLICK TO CAPTURE')
        self.ui.capture_before_btn.setCheckable(True)
        self.ui.capture_after_btn.setCheckable(True)
        self.pop_status.connect(self.pop_status_label)
        self.ui.create_event_btn.clicked.connect(self.create_snapshot)
        self.ui.save_snapshot_btn.clicked.connect(self.save_snapshot)
        self.ui.capture_before_btn.toggled.connect(self.capture_reference)
        self.ui.capture_after_btn.toggled.connect(self.capture_goal)
        self.ui.depth_slider.valueChanged.connect(self.depth_changed)
        self.ui.thresh_slider.valueChanged.connect(self.pct_in_goal_changed)
        self.ui.select_existing_combo.activated.connect(self.event_selected)
        self.ui.delete_snapshot_btn.hide()
        self.ui.delete_snapshot_btn.clicked.connect(self.delete_snapshot)

        self.pop_status_label('STARTED SUCCESSFULLY')
        self.show()
        self.reset()
        self.hide_capture()

    def event_selected(self,sid):

        if sid is not 0:
            self.current_event_index = sid
            self.current_event_label = self.ui.select_existing_combo.itemText(sid)
            self.current_event_name = str(self.current_event_label.split('(')[0]).strip(' ').lower()
            rospy.logwarn(self.current_event_name)
            self.ui.delete_snapshot_btn.show()

    def delete_snapshot(self):
        if self.current_event_name:
            rospy.logwarn(self.snapshots)
            del(self.snapshots[self.current_event_name])
            self.ui.select_existing_combo.removeItem(self.current_event_index)
            rospy.logwarn('Deleting snapshot '+ self.current_event_name)
            self.reset()
            self.hide_capture()
            self.ui.delete_snapshot_btn.hide()

    def hide_capture(self):
        self.ui.create_widget.hide()
        self.ui.capture_before_btn.hide()
        self.ui.capture_after_btn.hide()
        self.ui.save_snapshot_btn.hide()

    def show_capture(self):
        self.ui.create_widget.show()
        self.ui.capture_before_btn.show()
        self.ui.capture_after_btn.show()
        self.ui.save_snapshot_btn.show()

    def depth_changed(self,t):
        self.max_depth = float(t)/100 * 6
        self.ui.depth_field.setText(str(float(self.max_depth)))
        self.ui.event_depth_field.setText(str(float(self.max_depth))+'m')
        if self.snapshot:
            self.snapshot.set_depth(self.max_depth)
        # rospy.logwarn('depth changed to ['+str(self.max_depth)+']')

    def pct_in_goal_changed(self,t):
        self.ui.thresh_field.setText(str(float(t)))
        self.pct_in_goal = float(t)/100 * 1
        self.ui.event_thresh_field.setText(str(100*float(self.pct_in_goal))+'%')
        if self.snapshot:
            self.snapshot.set_pct_in_goal(self.pct_in_goal)
        # rospy.logwarn('Percent-in-goal changed to ['+str(self.pct_in_goal)+']')

    def update(self):
        if not self.reference_paused:
            self.ui.before_image_label.setPixmap(QPixmap.fromImage(self.reference_image))
        else:
            self.ui.before_image_label.setPixmap(QPixmap.fromImage(self.reference_image_paused))

        if not self.goal_paused:
            self.ui.after_image_label.setPixmap(QPixmap.fromImage(self.goal_image))
        else:
            self.ui.after_image_label.setPixmap(QPixmap.fromImage(self.goal_image_paused))

        if self.reference_paused == True and self.goal_paused == True:
            self.calc_diff()
        else:
            if self.snapshot is not None:
                self.snapshot.is_fit = False

        # Create Predicate List
        ps = PredicateList()
        ps.pheader.source = rospy.get_name()
        ps.statements = []

        predicates = self.snapshots.keys()
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = predicates
        pval.assignments = ['snapshot_events']
        self.pub_valid.publish(pval)

        # Always check saved snapshots and update predicates
        for snapshot_name, snapshot in self.snapshots.items():
            snapshot.transform(self.im_xyz)
            goal = snapshot.matching_goal
            # rospy.logwarn('Snapshot ['+snapshot_name+'] goal is ['+str(goal)+']')

            if goal == True:
                # Send true predicate
                statement = PredicateStatement( predicate='region_occupied',
                                          confidence=1,
                                          value=True,
                                          num_params=1,
                                          params=[str(snapshot_name), '', ''])
                ps.statements += [statement]
            else:
                # Do nothing
                pass

        self.pub_list.publish(ps)
        
    def reset(self):
        # self.ui.result_widget.hide()
        self.reference_paused = False
        self.goal_paused = False
        self.ui.capture_before_btn.setText('CLICK TO CAPTURE')
        self.ui.capture_after_btn.setText('CLICK TO CAPTURE')
        self.ui.capture_before_btn.setChecked(False)
        self.ui.capture_after_btn.setChecked(False)
        self.ui.difference_image_label.setPixmap(QPixmap.fromImage(QImage()))
        self.snapshot = None
        rospy.logwarn('Resetting')

    def create_snapshot(self):
        self.reset()
        self.show_capture()
        self.snapshot = SnapshotState(self.max_depth,self.matching_threshold, self.pct_in_goal)
        self.ui.event_depth_field.setText(str(float(self.max_depth))+'m')
        self.ui.event_thresh_field.setText(str(100*float(self.pct_in_goal))+'%')

    def save_snapshot(self):
        if self.reference_paused and self.goal_paused:

            name = str(self.ui.name_field.text())
            if name == '':
                rospy.logerr('You must specify a name for the snapshot.')
                self.pop_status.emit('No Name Specified','background-color:#F52C16; color:#ffffff')
                return

            self.snapshots[str(name).lower()] = self.snapshot
            self.ui.select_existing_combo.addItem(name.upper())
            rospy.logwarn('Creating snapshot '+ str(name).upper())
            self.reset()
            self.hide_capture()


    def calc_diff(self):
        # Check for current snapshot creation
        if self.snapshot is None:
            return

        if self.bbox_bottomright is not None:
            self.snapshot.bbox_topleft = self.bbox_topleft
            self.snapshot.bbox_bottomright = self.bbox_bottomright
        else:
            self.snapshot.bbox_topleft = None
            self.snapshot.bbox_bottomright = None            

        # Update current snapshot
        if not self.snapshot.is_fit:
            self.snapshot.fit(self.reference_image_raw, self.goal_image_raw)
        
        im_pos = self.snapshot.transform(self.im_xyz, self.im_rgb)

        # Display bounding box
        if self.bbox_topleft is not None:
            y1, x1 = self.bbox_topleft
            if self.bbox_bottomright is not None:
                y2, x2 = self.bbox_bottomright
            else:
                cursor = QtGui.QCursor()
                relative_pos = self.ui.difference_vis.mapFromGlobal(cursor.pos())
                y2, x2 = [relative_pos.y()*2, relative_pos.x()*2]

            cv2.rectangle(im_pos,(x1,y1),(x2,y2),(255,255,255),2)

        # Update QT
        height, width, byteValue = im_pos.shape
        byteValue = byteValue * width
        self.current_image = QImage(im_pos, width, height, byteValue, QImage.Format_RGB888)
        self.difference_image = QImage(self.current_image)
        self.ui.difference_image_label.setPixmap(QPixmap.fromImage(self.difference_image))

    def capture_reference(self,val):
        if val == True:
            self.reference_image_paused = QImage(self.reference_image.copy())
            # self.reference_image_raw = self.im_rgb.copy()
            self.reference_image_raw = self.im_xyz.copy()
            self.reference_paused = True
            self.ui.capture_before_btn.setText('CLICK TO RE-CAPTURE')
            if self.goal_paused == True:
                self.calc_diff()
        else:
            self.ui.capture_before_btn.setText('CLICK TO CAPTURE')
            self.reference_paused = False
            
    def capture_goal(self,val):
        if val == True:
            self.goal_image_paused = QImage(self.goal_image.copy())
            # self.goal_image_raw = self.im_rgb.copy()
            self.goal_image_raw = self.im_xyz.copy()
            self.goal_paused = True
            self.ui.capture_after_btn.setText('CLICK TO RE-CAPTURE')
            if self.reference_paused == True:
                self.calc_diff()
        else:
            self.ui.capture_after_btn.setText('CLICK TO CAPTURE')
            self.goal_paused = False

    ### STATUS LABEL
    @QtCore.pyqtSlot(str,str)
    def pop_status_label(self,text,style='None'):
        if style == None:
            self.ui.status_label.setStyleSheet('background-color:#2574A9; color:#ffffff')
        else:
            self.ui.status_label.setStyleSheet(style)
        self.ui.status_label.setText(text)
        self.ui.status_label.show()
        QTimer.singleShot(3000, self.hide_status_label)

    def hide_status_label(self):
        self.ui.status_label.hide()
        pass





    # def save(self):
    #     # print yaml.dump(self.events)
    #     rospy.logwarn('Saving Events...')
    #     try:
    #         self.save_service(id='force_events',type='instructor_force_events',text=yaml.dump(self.events))
    #     except (rospy.service.ServiceException) as e:
    #         pass

    #     rospy.sleep(.5)
    #     rospy.logwarn('Done.')
        

    # def load(self):
    #     events = yaml.load(self.load_service(id='force_events',type='instructor_force_events').text)
    #     if events is not None:
    #         self.events = events
    #         for e in self.events.values():
    #             self.ui.select_event_combo.addItem(e.name.upper() + ' ('+str(e.force)+'N, '+e.direction+')')
    #     else:
    #         rospy.logwarn('No Events Found')


    # def new_event(self):
    #     self.cancel_selected()
    #     self.ui.event_widget.show()
    #     self.started_event = True
    #     self.ui.name_field.setText('')
    #     self.ui.force_spin.setValue(0)
    #     self.current_direction = None
    #     print 'Starting New Event'

    # def event_selected(self,sid):
    #     self.cancel_selected()
    #     if sid is not 0:
    #         self.current_event_label = self.ui.select_event_combo.itemText(sid)
    #         self.current_event_name = str(self.current_event_label.split('(')[0]).strip(' ').lower()
    #         self.current_event = self.events[self.current_event_name]
    #         self.ui.event_widget.show()
    #         self.ui.manage_widget.show()
    #         self.ui.name_field.setText(self.current_event_name)
    #         self.ui.force_spin.setValue(int(self.current_event.force))
    #         self.interactionStyle.set_current_direction(self.current_event.direction)

    # def update_selected(self):
    #     pass

    # def delete_selected(self):
    #     sid = self.ui.select_event_combo.findText(self.current_event_label)
    #     self.ui.select_event_combo.removeItem(sid)
    #     self.events.pop(self.current_event_name)
    #     self.pop_status.emit('DELETED EVENT: '+ str(self.current_event_name).upper(),'background-color:#5AD132; color:#ffffff')
    #     self.current_event_name = ''
    #     self.current_event = None
    #     self.current_event_label = ''
    #     self.cancel_selected()
    #     pass

    # def cancel_selected(self):
    #     self.ui.manage_widget.hide()
    #     self.ui.event_widget.hide()
    #     self.ui.select_event_combo.setCurrentIndex(0)
    #     self.interactionStyle.set_current_direction(None)

    # def create_event(self):
    #     if self.started_event == True:
    #         name = str(self.ui.name_field.text())
    #         force = float(self.ui.force_spin.value())

    #         if name == '':
    #             rospy.logerr('You must specify a name for the event.')
    #             self.pop_status.emit('No Name Specified','background-color:#B52A2A; color:#ffffff')
    #             return
    #         if force == 0:
    #             rospy.logerr('You must specify a force value.')
    #             self.pop_status.emit('No Force Specified','background-color:#B52A2A; color:#ffffff')
    #             return
    #         if self.current_direction == None:
    #             rospy.logerr('You must select a force direction.')
    #             self.pop_status.emit('No Direction Selected','background-color:#B52A2A; color:#ffffff')
    #             return

    #         ### Create force event
    #         print 'Crating Force Event'
    #         print '-- Name: '+ str(name)
    #         print '-- Force: '+ str(force)
    #         print '-- Direction: '+ str(self.current_direction)

    #         self.events[name] = ForceEvent(name.lower(),force,self.current_direction)
    #         self.ui.select_event_combo.addItem(name.upper() + ' ('+str(force)+'N, '+self.current_direction+')')
    #         self.pop_status.emit('CREATED EVENT: '+ str(name).upper(),'background-color:#5AD132; color:#ffffff')
    #         self.ui.event_widget.hide()
    #         self.stop_test()
    #     else:
    #         print 'No Event Started'

    def check_ok(self):
        self.update()
        if rospy.is_shutdown():
          self.close()
          self.app_.exit()

    def closeEvent(self, event):
        print 'Closing...'
        # self.save()
        event.accept()
        print 'Finished.'

if __name__ == '__main__':
    app = QApplication( sys.argv )
    w = SnapshotEventCreator(app)
    w.show()
    app.exec_()
