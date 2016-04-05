import rospy
from costar_objrec_msgs.msg import *
from predicator_msgs.msg import *

'''
convert detected objects into predicate messages we can query later on
'''
class DetectedObjectsPublisher:

    valid_msg = None

    def __init__(self):
        self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        self.valid_msg = ValidPredicates()
        self.valid_msg.pheader.source = rospy.get_name()
        self.valid_msg.predicates.append('class_detected')
        self.valid_msg.predicates.append('member_detected')

        self.sub = rospy.Subscriber('/detected_object_list', DetectedObjectList, self.callback)

    def callback(self,msg):
        print msg
        true_msg = PredicateList()
        true_msg.pheader.source = rospy.get_name()

        for obj in msg.objects:
            ps = PredicateStatement()
            ps.predicate = obj.object_class
            ps.num_params = 1
            ps.confidence = 1.0
            ps.params[0] = obj.id
            ps.param_classes.append('frame')
            true_msg.statements.append(ps)

            ps = PredicateStatement()
            ps.predicate = 'member_detected'
            ps.num_params = 2
            ps.confidence = 1.0
            ps.params[0] = obj.id
            ps.params[1] = obj.object_class
            true_msg.statements.append(ps)

            ps = PredicateStatement()
            is_det_pred = 'is_detected_%s'%(obj.object_class)
            ps.predicate = is_det_pred
            ps.num_params = 1
            ps.confidence = 1.0
            ps.params[0] = obj.id
            true_msg.statements.append(ps)

            ps = PredicateStatement()
            ps.predicate = 'class_detected'
            ps.num_params = 1
            ps.confidence = 1.0
            ps.params[0] = obj.object_class
            true_msg.statements.append(ps)

            if not obj.object_class in self.valid_msg.predicates:
                self.valid_msg.predicates.append(obj.object_class)
            if not obj.object_class in self.valid_msg.assignments:
                self.valid_msg.assignments.append(obj.object_class)
            if not obj.id in self.valid_msg.assignments:
                self.valid_msg.assignments.append(obj.id)
            if not is_det_pred in self.valid_msg.predicates:
                self.valid_msg.predicates.append(is_det_pred)

        self.pub.publish(true_msg)
        self.vpub.publish(self.valid_msg)

        #print self.valid_msg
        #print true_msg
