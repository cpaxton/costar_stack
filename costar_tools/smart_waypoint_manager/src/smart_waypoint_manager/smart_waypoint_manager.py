import rospy
import yaml
import tf
import tf_conversions.posemath as pm

from predicator_msgs.srv import *
from predicator_msgs.msg import *

from librarian_msgs.srv import *
from librarian_msgs.msg import *

from predicator_landmark import GetWaypointsService

'''
SmartWaypointManager
This class will create and load smart waypoints from the $LIBRARIAN_HOME/smart_waypoints directory
It also provides ways to use and access these methods
'''
class SmartWaypointManager:


    def __init__(self,world="world"):
        self.get_waypoints_srv = GetWaypointsService(world=world,service=False)

        rospy.loginfo("[SmartMove] Waiting for LIBRARIAN to handle file I/O...")
        rospy.wait_for_service('/librarian/add_type',5)
        self.add_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)


        self.broadcaster = tf.TransformBroadcaster()
        self.world = world

        self.folder = 'smartmove_waypoint'

        self.add_type_service(self.folder)

        self.waypoints = {}
        self.waypoint_names = {}

    '''
    get all waypoints from the disk
    '''
    def load_all(self):
        self.waypoints = {}
        waypoint_filenames = self.list_service(self.folder).entries
    
        for name in waypoint_filenames:
            data = yaml.load(self.load_service(id=name,type=self.folder).text)
            if not data[1] in self.waypoints.keys():
                self.waypoints[data[1]] = []
                self.waypoint_names[data[1]] = []

            self.waypoints[data[1]].append(data[0])
            self.waypoint_names[data[1]].append(name)

        print self.waypoint_names
        print self.waypoints

    def update_tf(self):
        
        for key in self.waypoints.keys():
            (poses,names) = self.get_waypoints_srv.get_waypoints(key,[],self.waypoints[key],self.waypoint_names[key])
            for (pose,name) in zip(poses,names):
                (trans,rot) = pm.toTf(pm.fromMsg(pose))
                self.broadcaster.sendTransform(trans,rot,rospy.Time.now(),name,self.world)
                print (trans, rot, name)

    def add_waypoint(self):
        pass
        

