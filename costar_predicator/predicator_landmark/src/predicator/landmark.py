import rospy
from predicator_msgs.msg import *
from predicator_msgs.srv import *
import tf
import tf_conversions.posemath as pm

'''
convert detected objects into predicate messages we can query later on
'''
class GetWaypointsService:

    valid_msg = None

    def __init__(self,world="world",service=True):
        #self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        #self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        #self.valid_msg = ValidPredicates()
        #self.valid_msg.pheader.source = rospy.get_name()
        #self.valid_msg.predicates.append('class_detected')
        #self.valid_msg.predicates.append('member_detected')

        #self.sub = rospy.Subscriber('/landmark_definitions', LandmarkDefinition, self.callback)
        self.world = world
        self.and_srv = rospy.ServiceProxy('/predicator/match_AND',Query)
        self.service = rospy.Service('/predicator/get_waypoints',GetWaypoints,self.get_waypoints_srv)
        self.listener = tf.TransformListener()

    def get_waypoints_srv(self,req):
        resp = GetWaypointsResponse()
        poses = self.get_waypoints(req.frame_type,req.predicates,req.transforms)

        if poses is None or len(poses) < 1:
            resp.msg = 'FAILURE -- message not found!'
            resp.success = False
        else:
            resp.waypoints.poses = poses
            resp.msg = 'SUCCESS -- done!'
            resp.success = True

        return resp

    def get_waypoints(self,frame_type,predicates,transforms):
        self.and_srv.wait_for_service()

        type_predicate = PredicateStatement()
        type_predicate.predicate = frame_type
        type_predicate.params = ['*','','']

        res = self.and_srv([type_predicate]+predicates)

        print "Found matches: " + str(res.matching)

        if (not res.found) or len(res.matching) < 1:
          return None

        poses = []
        for tform in transforms:
            poses.append(pm.fromMsg(tform))

        print poses
        new_poses = []

        for match in res.matching:
            try:
                (trans,rot) = self.listener.lookupTransform(self.world,match,rospy.Time(0))
                for pose in poses:
                    #resp.waypoints.poses.append(pm.toMsg(pose * pm.fromTf((trans,rot))))
                    new_poses.append(pm.toMsg(pose * pm.fromTf((trans,rot))))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('Could not find transform from %s to %s!'%(self.world,match))
        
        return new_poses
        
