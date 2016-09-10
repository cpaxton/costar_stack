import rospy

from predicator_msgs.msg import *
from predicator_msgs.srv import *

import tf
import tf_conversions.posemath as pm

from costar_objrec_msgs.msg import *

'''
convert detected objects into predicate messages we can query later on
'''
class GetWaypointsService:

    valid_msg = None

    def __init__(self,world="world",service=True,ns=""):
        #self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        #self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        #self.valid_msg = ValidPredicates()
        #self.valid_msg.pheader.source = rospy.get_name()
        #self.valid_msg.predicates.append('class_detected')
        #self.valid_msg.predicates.append('member_detected')

        #self.sub = rospy.Subscriber('/landmark_definitions', LandmarkDefinition, self.callback)
        self.world = world
        self.and_srv = rospy.ServiceProxy('/predicator/match_AND',Query)
        if service:
            self.service = rospy.Service('/predicator/get_waypoints',GetWaypoints,self.get_waypoints_srv)
        self.listener = tf.TransformListener()

        self.detected_objects = rospy.Subscriber(ns + '/detected_object_list',
                                                 DetectedObjectList,
                                                 self.detected_objects_cb)
        self.obj_symmetries = {}


    def detected_objects_cb(self,msg):
        for obj in msg.objects:
            self.obj_symmetries[obj.object_class] = obj.symmetry
        
    def get_waypoints_srv(self,req):
        resp = GetWaypointsResponse()
        print req
        (poses, names) = self.get_waypoints(req.frame_type,req.predicates,req.transforms,req.names)

        if poses is None or len(poses) < 1:
            resp.msg = 'FAILURE -- message not found!'
            resp.success = False
        else:
            resp.waypoints.poses = poses
            resp.names = names
            resp.msg = 'SUCCESS -- done!'
            resp.success = True

        return resp

    def get_waypoints(self,frame_type,predicates,transforms,names):
        self.and_srv.wait_for_service()

        type_predicate = PredicateStatement()
        type_predicate.predicate = frame_type
        type_predicate.params = ['*','','']

        print "----"
        print "predicate(s) to match:"
        print predicates
        print "----"
        print "type predicate to match:"
        print type_predicate

        res = self.and_srv([type_predicate]+predicates)

        print "Found matches: " + str(res.matching)
        #print res

        if (not res.found) or len(res.matching) < 1:
          return None

        poses = []
        for tform in transforms:
            poses.append(pm.fromMsg(tform))

        #print poses
        new_poses = []
        new_names = []

        for match in res.matching:
            try:
                (trans,rot) = self.listener.lookupTransform(self.world,match,rospy.Time(0))
                for (pose, name) in zip(poses,names):
                    #resp.waypoints.poses.append(pm.toMsg(pose * pm.fromTf((trans,rot))))
                    new_poses.append(pm.toMsg(pm.fromTf((trans,rot)) * pose))
                    new_names.append(match + "/" + name)

                    # Create extra poses for symmetries around the Z axis
                    if frame_type in self.obj_symmetries:
                        if self.obj_symmetries[frame_type].z_symmetries > 1:
                            for i in xrange(1, self.obj_symmetries[frame_type].z_symmetries):
                                theta = i * self.obj_symmetries[frame_type].z_rotation
                                #print "... "  + str(theta)
                                tform = pm.Frame(pm.Rotation.RotZ(theta))
                                #print tform
                                new_poses.append(pm.toMsg(pm.fromTf((trans,rot)) * tform * pose))
                                new_names.append(match + "/" + name + "/" + str(i))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('Could not find transform from %s to %s!'%(self.world,match))
        
        return (new_poses, new_names)
        
