import rospy
from predicator_msgs.msg import *
import tf

'''
convert detected objects into predicate messages we can query later on
'''
class DetectedObjectsPublisher:

    valid_msg = None

    def __init__(self):
        #self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        #self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        #self.valid_msg = ValidPredicates()
        #self.valid_msg.pheader.source = rospy.get_name()
        #self.valid_msg.predicates.append('class_detected')
        #self.valid_msg.predicates.append('member_detected')

        self.sub = rospy.Subscriber('/landmark_definitions', LandmarkDefinition, self.callback)
        self.service = rospy.Service('/predicator/get_possible_waypoints',)

    def callback(self,msg):

        self.pub.publish(true_msg)
        self.vpub.publish(self.valid_msg)

        print self.valid_msg
        print true_msg
