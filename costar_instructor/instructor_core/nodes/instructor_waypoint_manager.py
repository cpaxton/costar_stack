#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy
import rospkg
import tf; 
import tf_conversions as tf_c
from instructor_core.srv import *
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import yaml
import time
import PyKDL
# Interactive Markers #
from std_msgs.msg import * 
from sensor_msgs.msg import * 
from geometry_msgs.msg import *
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *


### TIMER ###################################################
global tic_time
def tic():
    global tic_time
    tic_time = time.time()
def toc():
    global tic_time
    elapsed = time.time() - tic_time ############## TIMER
    rospy.logerr(elapsed) ################## TIMER
### TIMER ###################################################

class WaypointManager(object):
    def __init__(self):
        rospy.init_node('instructor_waypoint_manager',anonymous=True)
        rospy.loginfo('Waypoint Manager Starting Up...')
        self.broadcaster_ = tf.TransformBroadcaster()
        self.listener_ = tf.TransformListener()
        self.add_waypoint_service = rospy.Service('instructor_core/AddWaypoint', AddWaypoint, self.add_waypoint)
        self.remove_waypoint_service = rospy.Service('instructor_core/RemoveWaypoint', RemoveWaypoint, self.remove_waypoint)
        self.get_waypoint_list_service = rospy.Service('instructor_core/GetWaypointList', GetWaypointList, self.get_waypoint_list)
        self.get_landmark_list_service = rospy.Service('instructor_core/GetLandmarkList', GetLandmarkList, self.get_landmark_list)
        self.get_relative_waypoint_list_service = rospy.Service('instructor_core/GetRelativeWaypointList', GetRelativeWaypointList, self.get_relative_waypoint_list)
        self.waypoints = {}
        self.relative_waypoints = {}
        self.landmarks = {}
        self.saved_landmark_frames = {}
        # Set up librarian
        rospy.loginfo('Waypoint Manager Waiting for Librarian Services')
        rospy.wait_for_service('/librarian/add_type',5)
        self.set_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)
        self.set_type_service('instructor_waypoint')
        self.set_type_service('instructor_relative_waypoint')
        self.stale_landmarks = {}

        # TEST LANDMARK #
        #self.delete_initial_landmarks()
        # rospy.set_param('/instructor_landmark/test_landmark','test_landmark')
        self.F_relative_waypoint_last = None

        self.ms = InteractiveMarkerServer("waypoint_manager_markers")

        # load existing waypoints if any
        self.load()
        rospy.loginfo('Waypoint Manager Running')
        rospy.sleep(1.0)

        self.ms.applyChanges()

        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(.5)


    def save_all(self):
        # rospy.logwarn("### SAVING ALL ###")
        for waypoint_name, pose in self.waypoints.items():
            self.save_service(id=waypoint_name,type='instructor_waypoint',text=yaml.dump(pose))
        # rospy.logwarn("### DONE ###")

    def load(self):
        waypoint_names = self.list_service('instructor_waypoint').entries
        for name in waypoint_names:
            if name[0] == '.':
                # Ignore invalid waypoints
                continue
            pose = yaml.load(self.load_service(id=name,type='instructor_waypoint').text)
            self.waypoints['/' + name] = pose
            self.add_waypoint_marker('/' + name)

        relative_waypoint_names = self.list_service('instructor_relative_waypoint').entries
        rospy.loginfo('Found Relative Waypoints:')
        rospy.loginfo(relative_waypoint_names)
        for name in relative_waypoint_names:
            if name[0] == '.':
                # Ignore invalid waypoints
                continue
            rospy.logwarn("loading " + str(name))
            data = yaml.load(self.load_service(id=name,type='instructor_relative_waypoint').text)
            self.relative_waypoints['/' + name] = data
            self.add_relative_waypoint_marker('/' + name)

    def update(self):
        for name,waypoint in self.waypoints.items():
            F = tf_c.fromMsg(waypoint)
            self.broadcaster_.sendTransform(tuple(F.p),tuple(F.M.GetQuaternion()),rospy.Time.now(), name, '/world')

        for name,waypoint_data in self.relative_waypoints.items():
            try:
                F_relative_frame_waypoint = tf_c.fromMsg(waypoint_data[0])
                relative_frame_name = waypoint_data[1]
                F_world_relative_frame = tf_c.fromTf(self.listener_.lookupTransform('/world', relative_frame_name, rospy.Time(0)))
                F_relative_waypoint = F_world_relative_frame*F_relative_frame_waypoint
                self.broadcaster_.sendTransform(tuple(F_relative_waypoint.p),tuple(F_relative_waypoint.M.GetQuaternion()),rospy.Time.now(), name, '/world')
                self.F_relative_waypoint_last = F_relative_waypoint
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                if self.F_relative_waypoint_last == None:
                    pass
                else:
                    self.broadcaster_.sendTransform(tuple(self.F_relative_waypoint_last.p),tuple(self.F_relative_waypoint_last.M.GetQuaternion()),rospy.Time.now(), name, '/world')

        self.check_landmarks()

    def delete_initial_landmarks(self):
        params = rospy.get_param_names()
        landmarks = [p for p in params if p.find('instructor_landmark')>=0]
        for L in landmarks:
            #print L
            rospy.delete_param(L)

    def check_landmarks(self):
        params = rospy.get_param_names()
        landmarks = [p for p in params if p.find('instructor_landmark')>=0]
        #rospy.logwarn("landmarks: " + str(landmarks))
        all_landmark_names =[]

        for L in landmarks:
            all_landmark_names.append(L.replace('/instructor_landmark/',''))

        for L in landmarks:
            landmark_name = L.replace('/instructor_landmark/','')
            #rospy.logwarn(landmark_name)
            try:
                landmark_tf_name = rospy.get_param(L)
            except KeyError:
                rospy.logerr("Could not find landmark %s!"%(L))
                continue
            #rospy.logwarn(landmark_tf_name)

            try:
              self.listener_.waitForTransform('/world',landmark_tf_name, rospy.Time(), rospy.Duration(4.0))
              try:
                  F = tf_c.fromTf(self.listener_.lookupTransform('/world',landmark_tf_name,rospy.Time(0)))
                  self.saved_landmark_frames[landmark_name] = F
              except (tf.LookupException, tf.ConnectivityException) as e:
                  rospy.logerr('Frame ['+landmark_tf_name+'] not found')
                  return

              if landmark_name not in self.landmarks.keys():
                  self.landmarks[landmark_name] = '/'+landmark_name
                  # short_name = 'landmark_' + landmark_name.split('_')[len(landmark_name.split('_'))-1:][0]
                  self.add_landmark_marker('/'+landmark_name,landmark_name)
              else:
                  if landmark_name in self.stale_landmarks.keys():
                      self.stale_landmarks.pop(landmark_name)
                      self.add_landmark_marker('/'+landmark_name,landmark_name)
            except tf.Exception, e:
                rospy.logerr('Frame ['+landmark_tf_name+'] not found (TIMEOUT)')
                if rospy.has_param(L):
                    rospy.delete_param(L)
        
        # Rebroadcast frame of landmark
        for N,F in self.saved_landmark_frames.items():
            self.broadcaster_.sendTransform(tuple(F.p),tuple(F.M.GetQuaternion()),rospy.Time.now(), '/'+N, '/world')

        # rospy.logwarn('Landmark Names')
        # rospy.logwarn(all_landmark_names)

        # TODO REMOVE LANDMARKS THAT ARE NOT FOUND ON THE PARAMETER SERVER

        # rospy.logwarn('Landmark Keys')
        # rospy.logwarn(self.landmarks.keys())
        for L in self.landmarks.keys():
            if L not in all_landmark_names:
                # rospy.logwarn('Landmark ['+L+'] no longer broadcasting')
                if L not in self.stale_landmarks.keys():
                    self.stale_landmarks[L] = True
                    self.stale_landmark_marker('/'+L,L + ' (stale)')
                    
            #     self.landmarks.pop(L)
            #     short_name = 'landmark_' + landmark_name.split('_')[len(landmark_name.split('_'))-1:][0]
            #     self.remove_landmark_marker(short_name)

        pass

    def add_waypoint(self,msg):
        name = str(msg.name).strip('/')
        if msg.relative_frame_name == '': # frame in world coordinates
            rospy.logwarn('Adding Waypoint: ['+name+']')
            if msg.name not in self.waypoints.keys():
                self.waypoints[msg.name] = msg.world_pose
                self.add_waypoint_marker(msg.name)
                self.save_service(id=str(name),type='instructor_waypoint',text=yaml.dump(msg.world_pose))
                # rospy.logwarn("### DONE ###")
                return 'Added Waypoint ['+name+']'
            else:
                # rospy.logwarn("### DONE ###")
                return 'Waypoint ['+name+'] already exists!'
                
        else: # frame in relative coordinates
            if msg.name not in self.relative_waypoints.keys():
                rospy.logwarn('Adding Relative Waypoint: ['+name+'] with landmark ['+msg.relative_frame_name+']')
                self.relative_waypoints[msg.name] = [msg.relative_pose, msg.relative_frame_name]
                self.add_relative_waypoint_marker(msg.name)
                self.save_service(id=name,type='instructor_relative_waypoint',text=yaml.dump([msg.relative_pose, msg.relative_frame_name]))
                # rospy.logwarn("### DONE ###")
                return 'Added Waypoint ['+name+'] relative to  ['+msg.relative_frame_name+']'
            else:
                # rospy.logwarn("### DONE ###")
                return 'Waypoint ['+name+'] already exists!'

    def remove_waypoint(self,msg):
        # rospy.logwarn("### REMOVING WAYPOINT ###")
        name = str(msg.name).strip('/')
        if msg.name in self.waypoints.keys():
            self.waypoints.pop(msg.name)
            self.remove_waypoint_marker(msg.name)
            self.delete_service(id=name,type='instructor_waypoint')
            # rospy.logwarn("### DONE ###")
            return 'Removed Waypoint ['+name+']'
        elif msg.name in self.relative_waypoints.keys():
            self.relative_waypoints.pop(msg.name)
            self.remove_relative_waypoint_marker(msg.name)
            self.delete_service(id=name,type='instructor_relative_waypoint')
            # rospy.logwarn("### DONE ###")
            return 'Removed Relative Waypoint ['+name+']'
        else:
            # rospy.logwarn("### DONE ###")
            return 'Waypoint ['+name+'] does not exist!'

    def get_landmark_list(self,msg):
        # rospy.logwarn("### GETTING LANDMARKS ###")
        names = []
        frame_names = []
        for n,f in self.landmarks.items():
            names.append(n)
            frame_names.append(f)
        resp = GetLandmarkListResponse()
        resp.names = names
        resp.frame_names = frame_names
        # rospy.logwarn("### DONE ###")
        return resp

    def get_waypoint_list(self,msg):
        # rospy.logwarn("### GETTING WAYPOINTS ###")
        waypoints = []
        for n in self.waypoints.keys():
            waypoints.append(n)
        resp = GetWaypointListResponse()
        resp.names = waypoints
        # rospy.logwarn("### DONE ###")
        return resp

    def get_relative_waypoint_list(self,msg):
        # rospy.logwarn("### GETTING RELATIVE WAYPOINTS ###")
        waypoints = []
        for n in self.relative_waypoints.keys():
            waypoints.append(n)
        resp = GetRelativeWaypointListResponse()
        resp.names = waypoints
        # rospy.logwarn("### DONE ###")
        return resp

    ### MARKERS ###
    def add_waypoint_marker(self,name):
        self.ms.erase(name+'marker')
        i_marker = InteractiveMarker()
        i_marker.header.frame_id = name
        i_marker.pose = Pose(Point(0,0,0),Quaternion())
        i_marker.scale = .1
        i_marker.name = str(name+'marker').replace('/','')
        # Arrow Marker
        marker = Marker()
        marker.type = Marker.ARROW
        marker.color.r = 24.0/255.0
        marker.color.g = 167.0/255.0
        marker.color.b = 240.0/255.0
        marker.color.a = .75
        marker.scale = Point(.075,.015,.015)
        R = PyKDL.Rotation.RPY(0,-1.5707,0)
        P = Point(0,0,0)
        Q = Quaternion(*R.GetQuaternion())
        marker.pose = Pose(P,Q)
        # Text Marker
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.color.r = 25.0/255.0
        text_marker.color.g = 181.0/255.0
        text_marker.color.b = 219.0/255.0
        text_marker.color.a = 1.0
        text_marker.scale = Point(.02,.02,.02)
        text_marker.text = str(name).replace('/','').upper()
        text_marker.pose = Pose(Point(0,-.05,0),Quaternion())
        # Marker Control
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        marker_control.markers.append( marker )
        marker_control.markers.append( text_marker )
        i_marker.controls.append( marker_control )
        self.ms.insert(i_marker, self.processFeedback)
        self.ms.applyChanges()
        rospy.sleep(.1)

    def remove_waypoint_marker(self,name):
        self.ms.erase(str(name+'marker').replace('/',''))
        self.ms.applyChanges()
        rospy.sleep(.1)

    def add_relative_waypoint_marker(self,name):
        self.ms.erase(name+'rel_marker')
        i_marker = InteractiveMarker()
        i_marker.header.frame_id = name
        i_marker.pose = Pose(Point(0,0,0),Quaternion())
        i_marker.scale = .1
        i_marker.name = str(name+'rel_marker').replace('/','')
        # Arrow Marker
        marker = Marker()
        marker.type = Marker.ARROW
        marker.color.r = 135.0/255.0
        marker.color.g = 211.0/255.0
        marker.color.b = 124.0/255.0
        marker.color.a = .75
        marker.scale = Point(.075,.015,.015)
        R = PyKDL.Rotation.RPY(0,-1.5707,0)
        P = Point(0,0,0)
        Q = Quaternion(*R.GetQuaternion())
        marker.pose = Pose(P,Q)
        # Text Marker
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.color.r = 135.0/255.0
        text_marker.color.g = 211.0/255.0
        text_marker.color.b = 124.0/255.0
        text_marker.color.a = 1.0
        text_marker.scale = Point(.02,.02,.02)
        text_marker.text = str(name).replace('/','').upper()
        text_marker.pose = Pose(Point(0,-.05,0),Quaternion())
        # Marker Control
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        marker_control.markers.append( marker )
        marker_control.markers.append( text_marker )
        i_marker.controls.append( marker_control )
        self.ms.insert(i_marker, self.processFeedback)
        self.ms.applyChanges()
        rospy.sleep(.1)

    def remove_relative_waypoint_marker(self,name):
        self.ms.erase(str(name+'rel_marker').replace('/',''))
        self.ms.applyChanges()
        rospy.sleep(.1)

    def add_landmark_marker(self,name,show_name):
        self.ms.erase(name+'landmark_marker')
        i_marker = InteractiveMarker()
        i_marker.header.frame_id = name
        i_marker.pose = Pose(Point(0,0,0),Quaternion())
        i_marker.scale = .1
        i_marker.name = str(name+'landmark_marker').replace('/','')
        # Arrow Marker
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.color.r = 246.0/255.0
        marker.color.g = 36.0/255.0
        marker.color.b = 89.0/255.0
        marker.color.a = .75
        marker.scale = Point(.05,.05,.025)
        # Text Marker
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.color.r = 246.0/255.0
        text_marker.color.g = 36.0/255.0
        text_marker.color.b = 89.0/255.0
        text_marker.color.a = 1.0
        text_marker.scale = Point(.02,.02,.02)
        text_marker.text = show_name.upper()
        text_marker.pose = Pose(Point(0,0,.04),Quaternion())
        # Marker Control
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        marker_control.markers.append( marker )
        marker_control.markers.append( text_marker )
        i_marker.controls.append( marker_control )
        self.ms.insert(i_marker, self.processFeedback)
        self.ms.applyChanges()
        rospy.sleep(.1)
    
    def stale_landmark_marker(self,name,show_name):
        self.ms.erase(name+'landmark_marker')
        i_marker = InteractiveMarker()
        i_marker.header.frame_id = name
        i_marker.pose = Pose(Point(0,0,0),Quaternion())
        i_marker.scale = .1
        i_marker.name = str(name+'landmark_marker').replace('/','')
        # Arrow Marker
        marker = Marker()
        marker.type = Marker.CYLINDER
        marker.color.r = 155.0/255.0
        marker.color.g = 155.0/255.0
        marker.color.b = 155.0/255.0
        marker.color.a = .75
        marker.scale = Point(.05,.05,.025)
        # Text Marker
        text_marker = Marker()
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.color.r = 200.0/255.0
        text_marker.color.g = 200.0/255.0
        text_marker.color.b = 200.0/255.0
        text_marker.color.a = 1.0
        text_marker.scale = Point(.02,.02,.02)
        text_marker.text = show_name.upper()
        text_marker.pose = Pose(Point(0,0,.04),Quaternion())
        # Marker Control
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.interaction_mode = InteractiveMarkerControl.NONE
        marker_control.markers.append( marker )
        marker_control.markers.append( text_marker )
        i_marker.controls.append( marker_control )
        self.ms.insert(i_marker, self.processFeedback)
        self.ms.applyChanges()
        rospy.sleep(.1)

    def remove_landmark_marker(self,name):
        self.ms.erase(str(name+'landmark_marker').replace('/',''))
        self.ms.applyChanges()
        rospy.sleep(.1)

    def processFeedback(self, feedback):
        pass

# MAIN #########################################################################
if __name__ == '__main__':
    W = WaypointManager()
