#!/usr/bin/env python

import rospy
import copy
from predicator_msgs.msg import *
from predicator_core.srv import *

import sets

def predicate_to_tuple(predicate):
    if predicate.num_params == 1:
        return (predicate.predicate, predicate.params[0])
    elif predicate.num_params == 2:
        return (predicate.predicate, predicate.params[0], predicate.params[1])
    elif predicate.num_params == 3:
        return (predicate.predicate, predicate.params[0], predicate.params[1], predicate.params[2])
    else:
        return (predicate.predicate)
    #return (predicate.predicate, predicate.params[0], predicate.params[1], predicate.params[2])

'''
return a key based on a predicate
'''
def get_key(predicate, params):
    return "(%s,%s,%s,%s)"%(predicate,
            params[0],
            params[1],
            params[2])

'''
get a predicate out of a string
'''
def parse_key(key):
    ps = PredicateStatement()

    elems = [word for word in key[1:-1].split(',') if len(word) > 0]
    ps.predicate = elems[0];
    if len(elems) > 1:
        ps.params = ['', '', '']
        ps.params[0:(len(elems)-1)] = elems[1:len(elems)]
        ps.num_params = len(ps.params)

    return ps

'''
Predicator()
Class containing functions to process and access the different predicator functions.
Aggregates lists of predicates arriving on a list topic, and publishes them.
'''
class Predicator(object):

    def __init__(self): #, sub_topic, pub_topic, list_pub_topic, test_srv, get_srv, valid_srv, preds_srv, assgn_srv):
        self._subscriber = rospy.Subscriber('predicator/input', PredicateList, self.callback)
        self._validSubscriber = rospy.Subscriber('predicator/valid_input', ValidPredicates, self.validCallback)
        self._publisher = rospy.Publisher('predicator/all', PredicateSet)
        self._list_publisher = rospy.Publisher('predicator/list', PredicateList)
        self._testService = rospy.Service('predicator/test_predicate', TestPredicate, self.test_predicate)
        self._getService = rospy.Service('predicator/get_assignment', GetAssignment, self.get_assignment)
        self._valuePredicatesService = rospy.Service('predicator/get_value_predicates', GetList, self.get_value_predicates)
        self._predicatesService = rospy.Service('predicator/get_predicates', GetList, self.get_predicates)
        self._assignmentsService = rospy.Service('predicator/get_possible_assignment', GetTypedList, self.get_assignments)
        self._latest = {}
        self._predicates = {}

        self._all_predicates = sets.Set()
        self._all_value_predicates = sets.Set()
        self._all_assignments = sets.Set()

    '''
    get_value_predicates()
    returns what the input modules say is a list of valid value predicates
    '''
    def get_value_predicates(self, req):
        msg = GetListResponse()
        msg.data = self._all_value_predicates
        return msg

    '''
    get_predicates()
    returns what the input modules say is a list of valid nonvalue predicates
    '''
    def get_predicates(self, req):
        msg = GetListResponse()
        msg.data = self._all_predicates
        return msg

    def get_assignments(self, req):

        msg = GetTypedListResponse()

        if len(req.id) == 0:
            # get a list of all possible assignments
            msg.data = self._all_assignments
        else:
            # get a list of all possible assignments received for that predicate as of the last message
            pass

        return msg

    '''
    callback()
    read in predicate messages and record their source
    '''
    def callback(self, msg):
        self._latest[msg.header.frame_id] = msg.statements

    '''
    validCallback()
    read in sets of valid predicates from various sources
    '''
    def validCallback(self, msg):
        for item in msg.predicates:
            self._all_predicates.add(item)
        for item in msg.value_predicates:
            self._all_value_predicates.add(item)
        for item in msg.assignments:
            self._all_assignments.add(item)

    def aggregate(self):
        d = {}
        for source, lst in self._latest.items():
            for predicate in lst:
                key = get_key(predicate.predicate, predicate.params)
                d[key] = []

                for j in range(predicate.num_params):
                    new_params = copy.deepcopy(predicate.params)
                    #free_vars = ['','','']

                    new_params[j] = '*'
                    #free_vars[j] = predicate.params[j]

                    new_key = get_key(predicate.predicate, new_params)

                    if not new_key in d:
                        d[new_key] = []
                    d[new_key].append(copy.deepcopy(predicate.params))

                if(predicate.num_params > 1):
                    '''
                    NOTE: I changed this to let asterisks (*) mark free variables.
                    '''
                    tmp = ['', '', '']
                    tmp[1:predicate.num_params] = ['*'] * predicate.num_params
                    pred_key = get_key(predicate.predicate, ['','',''])
                    if not pred_key in d:
                        d[pred_key] = []
                    d[pred_key].append(copy.deepcopy(predicate.params))
                
        self._predicates = d

    '''
    publish()
    publish a message with all true things
    '''
    def publish(self):

        # create a list of all received predicates and republish it
        list_msg = PredicateList()
        list_msg.header.frame_id = rospy.get_name()
        list_msg.statements = []
        for source, lst in self._latest.items():
            for item in lst:
                list_msg.statements.append(item)

        self._list_publisher.publish(list_msg)

        # loop over all values 
        # create PredicateStatements for keys
        # create PredicateAssignemnts with keys, set of other predicates
        # set of PredicateStatements from list of values goes into each Assignment
        set_msg = PredicateSet()
        set_msg.header.frame_id = rospy.get_name()
        for key, values in self._predicates.items():
            pa = PredicateAssignment()
            pa.statement = parse_key(key)

            pa.values = []
            for val in values:
                ps = PredicateStatement()
                ps.predicate = pa.statement.predicate
                ps.num_params = pa.statement.num_params
                ps.params = val
                pa.values.append(ps)

            set_msg.assignments.append(pa)

        self._publisher.publish(set_msg)

    '''
    test_predicate (SERVICE)
    Check to see if a certain predicate exists on the server
    '''
    def test_predicate(self, req):
        if get_key(req.statement.predicate, req.statement.params) in self._predicates:
            print "Found predicate in response to request:"
            print req
            return TestPredicateResponse(found=True)
        else:
            print "Did not find predicate in response to request:"
            print req
            return TestPredicateResponse(found=False)

    '''
    get_assignment (SERVICE)
    get possible assignments of free variables
    '''
    def get_assignment(self, req):
        key = get_key(req.statement.predicate, req.statement.params) 
        if key in self._predicates:
            vals = []
            for stored_params in self._predicates[key]:
                s = PredicateStatement(predicate=req.statement.predicate,
                        num_params=req.statement.num_params,
                        params=stored_params)
                vals.append(s)
            return GetAssignmentResponse(found=True, values=vals)
        else:
            print "Did not find predicate in response to request:"
            print req
            return GetAssignmentResponse(found=False, values=[])

if __name__ == '__main__':
    rospy.init_node('predicator_core')
    
    spin_rate = rospy.get_param('rate',10)

    rate = rospy.Rate(spin_rate)

    try:

        pc = Predicator()
        #        'predicator/input',
        #        'predicator/all',
        #        'predicator/list',
        #        'predicator/test_predicate',
        #        'predicator/get_assignment')

        while not rospy.is_shutdown():
            pc.aggregate()
            pc.publish()
            rate.sleep()

    except rospy.ROSInterruptException: pass
