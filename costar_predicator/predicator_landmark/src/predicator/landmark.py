import rospy
from predicator_msgs.msg import *
import tf
import tf_conversions as tf_c

'''
convert detected objects into predicate messages we can query later on
'''
class GetWaypointsService:

    valid_msg = None

    def __init__(self):
        #self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        #self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        #self.valid_msg = ValidPredicates()
        #self.valid_msg.pheader.source = rospy.get_name()
        #self.valid_msg.predicates.append('class_detected')
        #self.valid_msg.predicates.append('member_detected')

        #self.sub = rospy.Subscriber('/landmark_definitions', LandmarkDefinition, self.callback)
        self.and_srv = rospy.ServiceProxy('/predicator/match_AND',Query)
        self.service = rospy.Service('/predicator/get_waypoints',GetWaypoints,self.get_waypoints)

    def get_waypoints(self,req):
        resp = GetWaypointsResponse()

        self.and_srv.wait_for_service()

        type_predicate = PredicateStatement()
        type_predicate.predicate = req.frame_type
        type_predicate.params = ['*','','']

        res = self.and_srv(type_predicate+predicates)

        print "Found matches: " + str(res.matches)


        if (not res.found) or len(res.matches) < 1:
            resp.msg = 'FAILURE -- message not found!'
            resp.success = False

        for match in res.matches:
            pass

        return resp
        
