import rospy
import yaml
import tf
import tf_conversions.posemath as pm

from predicator_msgs.srv import *
from predicator_msgs.msg import *

from librarian_msgs.srv import *
from librarian_msgs.msg import *

from costar_objrec_msgs.msg import *
from costar_robot_msgs.srv import *

from predicator_landmark import GetWaypointsService

import numpy as np
import PyKDL as kdl

'''
SmartWaypointManager
This class will create and load smart waypoints from the $LIBRARIAN_HOME/smart_waypoints directory
It also provides ways to use and access these methods
- stores smartmove waypoints with associated class info
- stores joint space waypoints
- stores a set of 6dof cartesian waypoints (for now in the world frame)
'''
class SmartWaypointManager:


    def __init__(self,
        world="world",
        ns="",
        endpoint="/endpoint",
        gripper_center="/gripper_center",
        listener=None, broadcaster=None):
        self.get_waypoints_srv = GetWaypointsService(world=world,service=False)

        rospy.loginfo("[SmartMove] Waiting for LIBRARIAN to handle file I/O...")
        rospy.wait_for_service('/librarian/add_type',5)
        self.add_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)

        self.get_assignment_service = rospy.ServiceProxy('/predicator/get_assignment', predicator_msgs.srv.GetAssignment)

        self.detected_objects = rospy.Subscriber(ns + '/detected_object_list', DetectedObjectList, self.detected_objects_cb)

        if not broadcaster is None:
            self.broadcaster = broadcaster
        else:
            self.broadcaster = tf.TransformBroadcaster()

        if not listener is None:
          self.listener = listener
        else:
          self.listener = tf.TransformListener()

        self.world = world
        self.endpoint = endpoint
        self.gripper_center = gripper_center

        self.folder = 'smartmove_waypoint'
        self.info_folder = 'smartmove_info'
        self.add_type_service(self.folder)

        self.waypoints = {}
        self.waypoint_names = {}

        self.all_moves = []

        self._reset_objs()
        self.obj_class = {}
        
        self.available_obj_classes = rospy.get_param("/costar/smartmove/available_objects")
        self.available_regions = rospy.get_param("/costar/smartmove/regions")
        self.available_references = rospy.get_param("/costar/smartmove/references")

    def _reset_objs(self):
        self.objs = []
        self.obj_classes = []
        
    '''
    reads in costar object detection messages
    this includes symmetry information produced by the vision pipeline
    '''
    def detected_objects_cb(self,msg):

        for obj in msg.objects:
          if not obj.id in self.objs:
            self.objs.append(obj.id)
          if not obj.object_class in self.obj_classes:
                self.objs.append(obj.object_class)
          self.obj_class[obj.id] = obj.object_class

    '''
    get all waypoints from the disk
    now including some joint space waypoints
    '''
    def load_all(self):

        self.waypoints = {}
        self.waypoint_names = {}
        self.all_moves = []

        '''
        this section loads class and pose information for computing smartmoves
        '''

        waypoint_filenames = self.list_service(self.folder).entries
    
        for name in waypoint_filenames:
          data = yaml.load(self.load_service(id=name,type=self.folder).text)
          if not data[1] in self.waypoints.keys():
            self.waypoints[data[1]] = []
            self.waypoint_names[data[1]] = []

          self.waypoints[data[1]].append(data[0])
          self.waypoint_names[data[1]].append(name)
          self.all_moves.append(data[1] + "/" + name)


    def lookup_waypoint(self,obj_class,name):
      rospy.logwarn("Smart Waypoint Manager looking for %s with class %s"%(name,obj_class))
      if obj_class in self.waypoints:
        return self.waypoints[obj_class][self.waypoint_names[obj_class].index(name)]
      else:
        rospy.logerr("There is no smartmove waypoint %s with class %s"%(name,obj_class))
        return None

    def get_reference_frames(self):
      return self.available_references

    def get_available_predicates(self):
      return self.available_regions

    def get_all_moves(self):
        return self.all_moves

    def get_detected_objects(self):
        self._reset_objs()

        if not self.available_obj_classes is None:
            for oc in self.available_obj_classes:
                resp = self.get_assignment_service(PredicateStatement(predicate=oc,params=["*","",""]))
                oc_objs = [p.params[0] for p in resp.values]
                if len(oc_objs) > 0:
                  self.obj_classes.append(oc)
                for obj in oc_objs:
                    self.obj_class[obj] = oc
                    rospy.loginfo("%s = %s"%(obj,oc))
                self.objs += oc_objs

        return self.objs

    def delete(self,move):
        self.delete_service(id=move.strip('/'),type=self.folder)

    def get_detected_object_classes(self):
        self.get_detected_objects()
        return self.obj_classes

    def get_available_object_classes(self):
        return self.available_obj_classes

    def save_new_waypoint(self,obj,name):
        pose = self.get_new_waypoint(obj)
        if not pose is None:
            self.save_service(id=name.strip('/'),type=self.folder,text=yaml.dump([pose,self.obj_class[obj]]))

    def get_new_waypoint(self,obj):
        try:
            # TODO: make the snap configurable
            #(trans,rot) = self.listener.lookupTransform(obj,self.endpoint,rospy.Time(0))
            (eg_trans,eg_rot) = self.listener.lookupTransform(self.gripper_center,self.endpoint,rospy.Time(0))
            (og_trans,og_rot) = self.listener.lookupTransform(obj,self.gripper_center,rospy.Time(0))

            rospy.logwarn("gripper obj:" + str((og_trans, og_rot)))
            rospy.logwarn("endpoint gripper:" + str((eg_trans, eg_rot)))

            xyz, rpy = [], []
            for dim in og_trans:
                if abs(dim) < 0.025:
                    xyz.append(0.)
                else:
                    xyz.append(dim)
            Rog = kdl.Rotation.Quaternion(*og_rot)
            for dim in Rog.GetRPY():
                rpy.append(np.round(dim / np.pi * 8.) * np.pi/8.)
            Rog_corrected = kdl.Rotation.RPY(*rpy)
            Vog_corrected = kdl.Vector(*xyz)
            Tog_corrected = kdl.Frame(Rog_corrected, Vog_corrected)

            rospy.logwarn(str(Tog_corrected) + ", " + str(Rog_corrected.GetRPY()))

            Teg = pm.fromTf((eg_trans, eg_rot))
            return pm.toMsg(Tog_corrected * Teg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Failed to lookup transform from %s to %s'%(obj,self.endpoint))
        return None

    def get_moves_for_class(self,obj_class):
        wpts = []
        try:
            wpts = self.waypoint_names[obj_class]
        except KeyError:
            wpts = []
        return wpts

    def get_moves_for_object(self,obj):
        try:
            print "obj class = %s"%self.obj_class[obj]
        except KeyError:
            rospy.logerr("Could not find object %s! Are you sure this is a valid object?"%obj)
        
        wpts = []
        try:
            wpts = self.waypoint_names[self.obj_class[obj]]
        except KeyError:
            wpts = []
        return wpts

    def publish_tf(self):
        for key in self.waypoints.keys():
            (poses,names) = self.get_waypoints_srv.get_waypoints(key,[],self.waypoints[key],self.waypoint_names[key])
            for (pose,name) in zip(poses,names):
                (trans,rot) = pm.toTf(pm.fromMsg(pose))
                self.broadcaster.sendTransform(trans,rot,rospy.Time.now(),name,self.world)

    '''
    Send any non-smart waypoints we might be managing.
    '''
    def publish_cartesian_waypoints(self):

        for (name,pose) in self.cart_waypoints:
            (trans,rot) = pm.toTf(pm.fromMsg(pose))
            self.broadcaster.sendTransform(trans,rot,rospy.Time.now(),name,self.world)

