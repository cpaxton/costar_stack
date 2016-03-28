import rospy
from costar_objrec_msgs.msg import *
from predicator_msgs.msg import *

'''
convert detected objects into predicate messages we can query later on
'''
class DetectedObjectsPublisher:

    def __init__(self):
        self.pub = rospy.Publisher('/predicator/input', PredicateList)
        self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates)

    def callback(self,msg):
        true_msg = PredicateList()
        valid_msg = ValidPredicates()

        

        self.pub.publish(true_msg)
        self.vpub.publish(valid_msg)
