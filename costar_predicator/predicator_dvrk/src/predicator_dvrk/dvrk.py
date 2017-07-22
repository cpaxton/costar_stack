import rospy
from predicator_msgs.msg import *

class SModelPredicator:

    def __init__(self,publish_predicates=True,start_subscriber=True,gripper_name='s_model'):

        self.valid_predicates = ValidPredicates(
		assignments=["clutch"], # list all pedals here
		predicates=['pressed'])
        self.predicate_msg = PredicateList()

        if publish_predicates:
            # create predicator things
            self.pub = rospy.Publisher("predicator/input",PredicateList,queue_size=1000)
            self.vpub = rospy.Publisher("predicator/valid_predicates",PredicateList,queue_size=1000)

        if start_subscriber:
            self.sub = rospy.Subscriber("LISTEN ON PEDAL TOPIC",inputMsg,self.callback)

        self.name = rospy.get_name()

    def callback(self, msg):
        self.handle(msg)

    def handle(self,status):
        self.predicate_msg = PredicateList()
        self.predicate_msg.pheader.source = self.name
	# add all pressed pedals to the list by calling add predicate ("pressed", pedal name)
    '''
    add a single message
    '''
    def addPredicate(self,predicate, pedal_name):
        p = PredicateStatement(predicate=predicate,params=[pedal_name,'',''])
        self.predicate_msg.statements.append(p)

    '''
    publish current predicate messages
    '''
    def tick(self):
        self.pub.publish(self.predicate_msg)
        self.vpub.publish(self.valid_predicates)

    '''
    update and spin
    '''
    def spin(self,rate=10):
        spin_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.tick()
            spin_rate.sleep()
            
