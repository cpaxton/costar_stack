#!/usr/bin/env python
# ROS IMPORTS
import roslib; roslib.load_manifest('instructor_predicate')
import rospy
# MSGS and SERVICES
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from predicator_msgs.msg import *
from std_msgs.msg import *

class GripperAnalyzer():

    def __init__(self):
        self.start = False  
        rospy.init_node('gripper_analyzer',anonymous=True)
        # PUBLISHERS AND SUBSCRIBERS
        self.gripper_state_sub = rospy.Subscriber('/gripper_state',String,self.gripper_cb)
        # PREDICATOR INTERFACE
        self.pub_list = rospy.Publisher('/predicator/input', PredicateList)
        self.pub_valid = rospy.Publisher('/predicator/valid_input', ValidPredicates)
        rospy.logwarn('GRIPPER ANALYZER INITIALIZE')   
        rospy.sleep(.5)
        pval = ValidPredicates()
        pval.pheader.source = rospy.get_name()
        pval.predicates = ['opening', 'closing', 'open', 'closed']
        pval.assignments = ['gripper']
        self.pub_valid.publish(pval) 
        rospy.sleep(1.0)  

        rospy.logwarn('GRIPPER ANALYZER RUNNING')   
        self.start = True  

        while not rospy.is_shutdown():
            rospy.sleep(.1)

        rospy.logwarn('GRIPPER ANALYZER FINISHED')

    def gripper_cb(self,val):
        if self.start == True:
            self.gripper_reported_state = str(val.data)

            ps = PredicateList()
            ps.pheader.source = rospy.get_name()
            ps.statements = []

            if self.gripper_reported_state == 'OPEN':
                statement = PredicateStatement( 'open', 1, PredicateStatement.TRUE, 1, ['gripper', '', ''],[]) 
                ps.statements += [statement]

            elif self.gripper_reported_state == 'CLOSED':
                statement = PredicateStatement( 'closed', 1, PredicateStatement.TRUE, 1, ['gripper', '', ''],[]) 
                ps.statements += [statement]

            elif self.gripper_reported_state == 'OPENING':
                statement = PredicateStatement( 'opening', 1, PredicateStatement.TRUE, 1, ['gripper', '', ''],[]) 
                ps.statements += [statement]

            elif self.gripper_reported_state == 'CLOSING':
                statement = PredicateStatement( 'closing', 1, PredicateStatement.TRUE, 1, ['gripper', '', ''],[]) 
                ps.statements += [statement]

            self.pub_list.publish(ps)

if __name__ == "__main__":
    analyzer = GripperAnalyzer()