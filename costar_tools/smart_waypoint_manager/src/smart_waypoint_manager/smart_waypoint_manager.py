import rospy
import yaml
import tf
import tf_conversions.posemath as pm

from predicator_msgs.srv import *
from predicator_msgs.msg import *

from librarian_msgs.srv import *
from librarian_msgs.msg import *

from costar_objrec_msgs.msg import *

from predicator_landmark import GetWaypointsService

'''
SmartWaypointManager
This class will create and load smart waypoints from the $LIBRARIAN_HOME/smart_waypoints directory
It also provides ways to use and access these methods
'''
class SmartWaypointManager:


    def __init__(self,world="world",ns="",endpoint="/endpoint"):
        self.get_waypoints_srv = GetWaypointsService(world=world,service=False)

        rospy.loginfo("[SmartMove] Waiting for LIBRARIAN to handle file I/O...")
        rospy.wait_for_service('/librarian/add_type',5)
        self.add_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)

        self.get_assignment_service = rospy.ServiceProxy('/predicator/get_assignment', predicator_msgs.srv.GetAssignment)

        #self.detected_objects = rospy.Subscriber(ns + '/detected_object_list', DetectedObjectList, self.detected_objects_cb)


        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.world = world
        self.endpoint = endpoint

        self.folder = 'smartmove_waypoint'

        self.add_type_service(self.folder)

        self.waypoints = {}
        self.waypoint_names = {}

        self.all_moves = []

        self.objs = []
        self.obj_classes = []
        self.obj_class = {}

        
        self.add_type_service("smartmove_info")
        self.available_obj_classes = yaml.load(self.load_service(type="smartmove_info",id="obj_classes").text)
        self.available_regions = yaml.load(self.load_service(type="smartmove_info",id="regions").text)
        self.available_references = yaml.load(self.load_service(type="smartmove_info",id="references").text)
        print "Available classes = " + str(self.available_obj_classes)

    def detected_objects_cb(self,msg):
        self.objs = [obj.id for obj in msg.objects]
        self.obj_classes = [obj.object_class for obj in msg.objects]
        for (o,c) in zip(self.objs,self.obj_classes):
            self.obj_class[o] = c

        #print self.objs
        #print self.obj_classes

    '''
    get all waypoints from the disk
    '''
    def load_all(self):

        self.waypoints = {}
        self.all_moves = []

        waypoint_filenames = self.list_service(self.folder).entries
    
        for name in waypoint_filenames:
            data = yaml.load(self.load_service(id=name,type=self.folder).text)
            if not data[1] in self.waypoints.keys():
                self.waypoints[data[1]] = []
                self.waypoint_names[data[1]] = []

            self.waypoints[data[1]].append(data[0])
            self.waypoint_names[data[1]].append(name)
            self.all_moves.append(data[1] + "/" + name)

        print " === LOADING === "
        print self.waypoint_names
        print self.waypoints

    def get_all_moves(self):
        return self.all_moves

    def get_detected_objects(self):
        #print self.available_obj_classes
        self.objs = []
        for oc in self.available_obj_classes:
            resp = self.get_assignment_service(PredicateStatement(predicate=oc,params=["*","",""]))
            oc_objs = [p.params[0] for p in resp.values]
            for obj in oc_objs:
                self.obj_class[obj] = oc
            self.objs += oc_objs

        return self.objs

    def delete(self,move):
        self.delete_service(id=move.strip('/'),type=self.folder)

    def get_detected_object_classes(self):
        return self.obj_classes

    def save_new_waypoint(self,obj,name):
        pose = self.get_new_waypoint(obj)
        if not pose is None:
            self.save_service(id=name.strip('/'),type=self.folder,text=yaml.dump([pose,self.obj_class[obj]]))

    def get_new_waypoint(self,obj):
        try:
            (trans,rot) = self.listener.lookupTransform(obj,self.endpoint,rospy.Time(0))
            return pm.toMsg(pm.fromTf((trans,rot)))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr('Failed to lookup transform from %s to %s'%(obj,self.endpoint))
        return None

    def get_moves_for_object(self,obj):
        print "obj class = %s"%self.obj_class[obj]
        
        wpts = []
        try:
            wpts = self.waypoint_names[self.obj_class[obj]]
        except KeyError:
            wpts = []
        return wpts

    def update_tf(self):
        
        for key in self.waypoints.keys():
            (poses,names) = self.get_waypoints_srv.get_waypoints(key,[],self.waypoints[key],self.waypoint_names[key])
            for (pose,name) in zip(poses,names):
                (trans,rot) = pm.toTf(pm.fromMsg(pose))
                self.broadcaster.sendTransform(trans,rot,rospy.Time.now(),name,self.world)
                #print (trans, rot, name)

    def add_waypoint(self):
        pass
        

