import rospy
from predicator_msgs.msg import *
from sensor_msgs.msg import Joy

class dvrkPredicator:

    def __init__(self,publish_predicates=True,start_subscriber=True,gripper_name='psm_gripper'):

        self.valid_predicates = ValidPredicates(
		assignments=["clutch","coag","camera_plus","camera_minus","camera"], 
		predicates=['pressed'])
        self.valid_predicates.pheader.source = rospy.get_name()
        self.predicate_msg = PredicateList()

        self.clutch_pressed = False
        self.camera_pressed = False

        if publish_predicates:
            # create predicator things
            self.pub = rospy.Publisher("predicator/input",PredicateList,queue_size=1000)
            self.vpub = rospy.Publisher("predicator/valid_input",ValidPredicates,queue_size=1000)

        if start_subscriber:
            self.sub = rospy.Subscriber("/dvrk/footpedals/clutch",Joy,self.callbackClutch)
            self.sub2 = rospy.Subscriber("/dvrk/footpedals/camera",Joy,self.callbackCamera)
            #self.sub2 = rospy.Subscriber("/dvrk/footpedals/coag",Joy,self.callback)
            #self.sub3 = rospy.Subscriber("/dvrk/footpedals/camera_plus",Joy,self.callback)
            #self.sub4 = rospy.Subscriber("/dvrk/footpedals/camera_minus",Joy,self.callback)
            #self.sub5 = rospy.Subscriber("/dvrk/footpedals/camera",Joy,self.callback)

        self.name = rospy.get_name()

    def callbackClutch(self, msg):
        # if this is for the clutch -- set clutch pressed
        if len(msg.buttons) > 0:
            if msg.buttons[0] == 1:
                # clutch pressed
                self.clutch_pressed = True
            elif msg.buttons[0] == 0:
                self.clutch_pressed = False
        print "clutch pedal status:", self.clutch_pressed

    def callbackCamera(self, msg):
        # if this is for the clutch -- set clutch pressed
        if len(msg.buttons) > 0:
            if msg.buttons[0] == 1:
                # clutch pressed
                self.camera_pressed = True
            elif msg.buttons[0] == 0:
                self.camera_pressed = False
        print "camera pedal status:", self.camera_pressed
	
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
        self.predicate_msg = PredicateList()
        self.predicate_msg.pheader.source = self.name
        if self.clutch_pressed:
            self.addPredicate("pressed","clutch")
        if self.camera_pressed:
            self.addPredicate("pressed","camera")
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
            
