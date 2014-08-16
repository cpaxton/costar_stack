#!/usr/bin/env python

import roslib; roslib.load_manifest('beetree')
import rospy 
from std_msgs.msg import *
from collections import OrderedDict
from copy import deepcopy

class Node(object):
    """ Beetree Node
    The core class for beetree nodes.  Beetree nodes are connected together in a tree 
    using the add_child and related functions.  A beetree node can generate a graphviz
    dot file for visualization purposes.
    """

    def __init__(self, name, label, color='', shape='box', flag=False, alt_label=None):
        """ Beetree Node Constructor
        @type name: String
        @param name: the name of the node
        @type label: String
        @param label: the label that will appear for the node in the generated dotcode
        @type color: String
        @param color: the color the node will appear as in dotcode
        @type shape: String
        @param shape: the shape the node will appear as in dotcode
        @type flag: Bool
        @param flag: a boolean which determines whether this node will be drawn in dotcode as highlighted 
        """
        self.num_children_ = 0
        self.highlighted_ = False
        self.node_status_ = 'NODE_ERROR'
        self.child_status_ = 'NODE_ERROR'
        self.name_ = name
        self.label_ = label
        self.color_ = color
        self.shape_ = shape
        self.flag_ = flag
        self.children_ = []
        self.children_names_ = []
        self.exec_index = None
        self.parent_ = None
        self.alt_view = False
        self.alt_label_ = alt_label
        self.alt_shape_ = 'record'

    def set_alt_view(self,v):
        self.alt_view = v
                
    def set_flag(self,flag):
        """ Sets a flag for whether this node will be highlighted in the dot code
        """
        self.flag_ = flag

    def generate_dot(self):
        """ Generates dot code for this node and its connection to its children
        Also recursively calls the children's generate_dot() functions
        """

        # if parent generate front end of dotcode string
        if self.parent_ == None:
            dot = 'digraph behavior_tree { splines=false; '
        else:
            dot = ''
        # generate this node's dot code
        if self.alt_view == True:
            shape = self.alt_shape_
            label = self.alt_label_
        else:
            shape = self.shape_
            label = self.label_

        rospy.logerr(shape)
        rospy.logerr(label)

        if self.flag_ == False:
            if self.color_ == '':
                dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][label="'+label+'"]; '
            else:
                dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][style="filled" fillcolor="'+self.color_+'"][label="'+label+'"]; '
        else:
            if self.color_ == '':
                dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][style="bold" color="red"][label="'+label+'"]; '
            else:
                dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][style="filled, bold" fillcolor="'+self.color_+'" color="red"][label="'+label+'"]; '   
        # recursively generate the node's child dot code
        if self.num_children_ > 0:
            for C in self.children_:
                # call the current child's generate_dot function
                dot += C.generate_dot()
                # generate the dot for the connection between self and the current child
                dot += self.name_ + ':s->' + C.name_ + ':n; '

        # if parent generate tail end of dotcode string
        if self.parent_ == None:        
            return dot + '}'
        else:
            return dot

    def reset(self):
        """ Resets the children of this node. useful when an execution is finished and the subtree needs to be reset
        """
        print "Node " + self.name_ + " resetting."
        self.reset_self()
        for C in self.children_:
            C.reset()

    def reset_self(self):
        # To be implemented by child class
        pass

    def get_position(self):
        position = self.parent_.children_.index(self)
        return position

    def add_child(self, child_to_add):
        """ Adds a child to this node
        @type child_to_add: Node
        @param child_to_add: the node to add as a child
        """
        child_to_add.parent_ = self
        self.children_.append(child_to_add)
        self.children_names_.append(child_to_add.name_)
        self.num_children_+=1

    def insert_child(self,index,child_to_insert):
        """ Insert a child at a position in the existing children of this node
        @type index: Int
        @param index: the position to insert the child
        @type child_to_insert: Node
        @param child_to_insert: the node to insert as a child
        """
        child_to_insert.parent_ = self
        self.children_.insert(index,child_to_insert)
        self.children_names_.insert(index,child_to_insert.name_)
        self.num_children_+=1

    def replace_child(self,existing_child,replacement_child):
        if existing_child in self.children_:
            replacement_child.parent_ = self
            existing_child_index = self.children_.index(existing_child)
            existing_children = existing_child.children_
            existing_children_names = existing_child.children_names_
            for e in existing_children:
                e.parent_ = replacement_child
            replacement_child.set_num_children(len(existing_children))
            replacement_child.children_ = existing_children
            replacement_child.children_names_ = existing_children_names
            self.children_[existing_child_index] = replacement_child
            return True
        else:
            return False

    def add_sibling_after(self,child_to_add):
        """ Add a sibling after this node as a child of this nodes parent
        @type child_to_add: Node
        @param child_to_add: the node to add as a sibling
        """
        my_index = self.parent_.children_.index(self)
        self.parent_.insert_child(my_index + 1, child_to_add)

    def add_sibling_before(self,child_to_add):
        """ Add a sibling before this node as a child of this nodes parent
        @type child_to_add: Node
        @param child_to_add: the node to add as a sibling
        """
        my_index = self.parent_.children_.index(self)
        self.parent_.insert_child(my_index, child_to_add)

    def remove_child_by_name(self,child_name):
        """ Remove a child by name
        @type child_name: String
        @param child_name: the name of the child to remove
        """
        index = self.children_names_.index(child_name)
        self.children_.pop(index)
        self.children_names_.pop(index)
        self.num_children_ -= 1    

    def remove_child(self, child_to_remove):
        """ Remove a child by reference
        @type child_to_remove: Node
        @param child_to_remove: reference to the child to remove
        """
        index = self.children_.index(child_to_remove)
        self.children_.pop(index)
        self.children_names_.pop(index)
        self.num_children_ -= 1    

    def remove_all_children(self):
        """ Remove all children of this node
        """
        self.children_ = []
        self.children_names_ = []
        self.num_children_ = 0

    def remove_self(self):
        """ Remove this node from its parents list of children and all connections to the tree
        """
        self.parent_.remove_child(self)
        return True

    def set_num_children(self,number):
        """ Set the number of children this node has
        """
        self.num_children_ = number

    def set_status(self,status):
        """ Set the status of this node as SUCCESS, RUNNING, FAILURE or NODE_ERROR
        @type status: String
        @param status: the status value
        """
        self.node_status_ = status
        # print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_

    def get_parent(self):
        return self.parent_

    def get_status(self):
        return self.node_status_

    def execute(self):
        """ Virtual function that each node runs when that node gets ticked
        """
        pass

### CORE LOGICAL NODES ----------------------------------------------------------------------------------------

class NodeSelector(Node):
    ''' Selector Node
    Runs children in order until one succeeds then 
    returns SUCCESS, if all fail, returns FAILURE.
    '''
    def __init__(self,name,label):
        L = '( * )\\n ' + label.upper()
        L_alt = '{SELECTOR | '+label.upper()+'}'
        super(NodeSelector,self).__init__(name,label=L,alt_label=L_alt)
    def get_node_type(self):
        return 'SELECTOR'
    def get_node_name(self):
        return 'Selector'
    def execute(self):

        for child in self.children_:
            status = child.execute()

            if status != 'FAILURE':
                return self.set_status(status)

        return self.set_status('FAILURE')


class NodeSequence(Node):
    """ Sequence Node
    The sequence node executes its children in order or insertion.  If a child
    fails, the node will return FAILURE. If a child succeeds, the sequence will
    then execute the next child until all children are executed, then return
    SUCCESS.
    """
    def __init__(self,name,label):
        L = '->'
        L_alt = '{SEQUENCE | '+label.upper()+'}'
        super(NodeSequence,self).__init__(name,label=L,alt_label=L_alt)
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):

        for child in self.children_:
            status = child.execute()

            if status != 'SUCCESS':
                return self.set_status(status)

        return self.set_status('SUCCESS')

class NodeIterator(Node):
    """ Iterator Node
    The iterator node executes its children in order or insertion, ignoring
    failure, and will return SUCCESS when all children have returned either 
    SUCCESS or FAILURE.
    """
    def __init__(self,name,label):
        L = '1...'
        L_alt = '{ITERATOR | '+label.upper()+'}'
        super(NodeIterator,self).__init__(name,label=L,alt_label=L_alt)
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):

        for child in self.children_:
            status = child.execute()

            if status != 'SUCCESS' and status != 'FAILURE':
                return self.set_status(status)
            
        return self.set_status('SUCCESS')       

class NodeParallelAll(Node):
    ''' Parallel All Node
    Executes all children close to simultaneously.  If any fail, returns
    FAILURE. Once all succeed, returns SUCCESS.  Until that point returns
    RUNNING.
    '''
    def __init__(self,name,label):
        L = '|A|'
        L_alt = '{PARALLEL ALL | '+label.upper()+'}'
        super(NodeParallelAll,self).__init__(name,label=L,alt_label=L_alt)
        self.num_success = None
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):
        # print 'Executing Parallel: (' + self.name_ + ')'
        if self.num_success == None:
            self.num_success = 0

        for C in self.children_:
            self.child_status_ = C.execute()
            if self.child_status_ == 'NODE_ERROR':
                self.num_success = None
                return self.set_status('NODE_ERROR')
            elif self.child_status_ == 'FAILURE':
                self.num_success = None
                return self.set_status('FAILURE')
            elif self.child_status_ == 'RUNNING':
                pass
            elif self.child_status_ == 'SUCCESS':
                self.num_success += 1

        # Only return if all children succeed
        if self.num_success == self.num_children_:
            self.num_success = None
            return self.set_status('SUCCESS')
        else:
            return self.set_status('RUNNING')

class NodeParallelRemove(Node):
    ''' Parallel Remove Node
    Executes all children close to simultaneously.  If any fail, returns
    FAILURE. Once all succeed, returns SUCCESS.  Until that point returns
    RUNNING. As nodes succeed, they are prevented from being ticked again
    '''
    def __init__(self,name,label):
        L = '|R|'# + label.upper()
        L_alt = '{PARALLEL REMOVE | '+label.upper()+'}'
        super(NodeParallelRemove,self).__init__(name,label=L,alt_label=L_alt)
        self.exec_list_ = None
        self.num_success = None
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):

        if self.exec_list_ == None:
            self.exec_list_ = deepcopy(self.children_)
            self.num_success = 0

        # print 'Executing Parallel: (' + self.name_ + ')'

        for C in self.exec_list_:
            self.child_status_ = C.execute()
            if self.child_status_ == 'NODE_ERROR':
                return self.set_status('NODE_ERROR')
            elif self.child_status_ == 'FAILURE':
                return self.set_status('FAILURE')
            elif self.child_status_ == 'RUNNING':
                pass
            elif self.child_status_ == 'SUCCESS':
                self.num_success += 1
                self.exec_list_.remove(C) # remove child that succeeds

        # only return if all children succeed
        if self.num_success == self.num_children_:
            return self.set_status('SUCCESS')
        else:
            return self.set_status('RUNNING')

class NodeParallelOne(Node):
    ''' Parallel One Node
    Executes all children close to simultaneously.  If any fail, returns
    FAILURE. Once ONE succeeds, returns SUCCESS.  Until that point returns
    RUNNING.
    '''
    def __init__(self,name,label):
        L = '|1|'
        L_alt = '{PARALLEL ONE | '+label.upper()+'}'
        super(NodeParallelOne,self).__init__(name,label=L,alt_label=L_alt)
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):

        # print 'Executing Parallel: (' + self.name_ + ')'

        for C in self.children_:
            self.child_status_ = C.execute()
            if self.child_status_ == 'NODE_ERROR':
                return self.set_status('NODE_ERROR')
            elif self.child_status_ == 'FAILURE':
                return self.set_status('FAILURE')
            elif self.child_status_ == 'RUNNING':
                pass
            elif self.child_status_ == 'SUCCESS':
                return self.set_status('SUCCESS')

        return self.set_status('RUNNING')

class NodeDecoratorRepeat(Node):
    ''' Decorator Repeat Node
    Executes child node N times. If N = -1, will repeat forever, ignoring failures
    '''
    def __init__(self,name,label,runs=1):
        if runs == -1:
            L = '(runs)\\n['+'INF'+']'
            L_alt = '{REPEAT FOREVER | '+label.lower()+'}'
        else:
            L = '(runs)\\n['+str(runs)+']'
            L_alt = '{REPEAT '+str(runs)+'| '+label.lower()+'}'
        
        super(NodeDecoratorRepeat,self).__init__(name,L,shape='pentagon',alt_label=L_alt)
        self.runs_ = runs
        self.num_runs_ = 0
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        if self.runs_ == -1: # run forever
            self.child_status_ = self.children_[0].execute()
            if self.child_status_ == 'SUCCESS':
                rospy.logwarn('REPEAT DECORATOR ['+self.name_+']: SUCCEEDED, RESET')
                self.children_[0].reset()
                return self.set_status('RUNNING')
            elif self.child_status_ == 'RUNNING':
                return self.set_status('RUNNING')
            elif self.child_status_ == 'FAILURE':
                rospy.logwarn('REPEAT DECORATOR ['+self.name_+']: FAILED, RESET')
                self.children_[0].reset()
                return self.set_status('RUNNING')
        else:
            if self.num_runs_ < self.runs_:
                self.child_status_ = self.children_[0].execute()
                if self.child_status_ == 'SUCCESS':
                    self.num_runs_ += 1
                elif self.child_status_ == 'RUNNING':
                    return self.set_status('RUNNING')
                elif self.child_status_ == 'FAILURE':
                    return self.set_status('FAILURE')
            else:
                return self.set_status('SUCCESS')

class NodeDecoratorIgnoreFail(Node):
    ''' Decorator Ignore Fail
    Returns success regardless of outcome of child
    '''
    def __init__(self,name,label):
        L = 'ignore\\nfailure'
        L_alt = '{IGNORE FAILURE | '+label.lower()+'}'
        super(NodeDecoratorIgnoreFail,self).__init__(name,L,shape='pentagon',alt_label=L_alt)
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        self.child_status_ = self.children_[0].execute()            
        if self.child_status_ == 'RUNNING':
            return self.set_status('RUNNING')
        else:
            return self.set_status('SUCCESS')

class NodeDecoratorWaitForSuccess(Node):
    ''' Decorator Wait for Success
    Returns running while waiting for child node to be true
    '''
    def __init__(self,name,label,timeout):
        L = 'wait\\nsuccess'
        L_alt = '{WAIT SUCCESS | '+label.lower()+'}'
        super(NodeDecoratorWaitForSuccess,self).__init__(name,L,shape='pentagon',alt_label=L_alt)
        self.timeout = timeout # seconds
        self.timer = None
        self.started = False
        self.finished = False
        self.needs_reset = False
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        if self.needs_reset:
            rospy.logwarn('Success Decorator needs already returned '+ self.get_status())
            return self.get_status()

        if not self.finished:
            if not self.started:
                if self.timeout == -1: # dont start timer, wait forever
                    self.started = True
                else:
                    self.timer = rospy.Timer(rospy.Duration(1.0*self.timeout), self.timed_out, oneshot=True)
                    self.started = True
                self.child_status_ = self.children_[0].execute()
                if self.child_status_ == 'SUCCESS':
                    rospy.logwarn('WAIT SUCCESS DECORATOR ['+self.name_+']: REPORTED SUCCESS')
                    self.needs_reset = True
                    return self.set_status('SUCCESS')
                elif self.child_status_ == 'RUNNING':
                    rospy.logwarn('WAIT SUCCESS DECORATOR ['+self.name_+']: CHILD RUNNING, WAITING')
                    return self.set_status('RUNNING')
                else:
                    rospy.logwarn('WAIT SUCCESS DECORATOR ['+self.name_+']: CHILD FAILED, RESET, WAITING')
                    self.children_[0].reset()
                    return self.set_status('RUNNING')
            else: # started
                self.child_status_ = self.children_[0].execute()
                if self.child_status_ == 'SUCCESS':
                    rospy.logwarn('WAIT SUCCESS DECORATOR ['+self.name_+']: REPORTED SUCCESS')
                    self.needs_reset = True
                    return self.set_status('SUCCESS')
                elif self.child_status_ == 'RUNNING':
                    rospy.logwarn('WAIT SUCCESS DECORATOR ['+self.name_+']: CHILD RUNNING, WAITING')
                    return self.set_status('RUNNING')
                else:
                    rospy.logwarn('WAIT SUCCESS DECORATOR ['+self.name_+']: CHILD FAILED, RESET, WAITING')
                    self.children_[0].reset()
                    return self.set_status('RUNNING')
        else: # finished
            rospy.logwarn('Success Decorator timed out')
            return self.set_status('FAILURE')

    def reset_self(self):
        self.started = False
        self.timer = None
        self.finished = False
        self.needs_reset = False
    def timed_out(self,event):
        self.finished = True

class NodeRoot(Node):
    ''' Root Node
    The root node can have only one child, and typically is used as the root of 
    the tree
    '''
    def __init__(self, name, label):
        L = '(/)'
        L_alt = '{ROOT | '+name+'}'
        super(NodeRoot,self).__init__(name,L,alt_label=L_alt)
    def get_node_type(self):
        return 'ROOT'
    def get_node_name(self):
        return 'Root'
    def execute(self):
        # print 'Executing Root: (' + self.name_ + ')'
        if self.num_children_ > 0:
            return self.children_[0].execute()
        # print 'ROOT: Child returned status: ' + self.child_status_

class NodeAction(Node):
    ''' Action Node
    Placeholder action node
    '''
    def __init__(self,name,label):
        L = 'Action\\n' + label.upper()
        L_alt = '{ACTION|' + label.lower()+'}'
        color='#92D665'
        super(NodeAction,self).__init__(name,L,color,alt_label=L_alt)
        self.name_ = name
    def get_node_type(self):
        return 'ACTION'
    def get_node_name(self):
        return 'Action'
    def execute(self):
        pass

class NodeService(Node):
    ''' Service Node
    Placeholder service node
    '''
    def __init__(self,name,label):
        color='#92D665'
        super(NodeService,self).__init__(name,label,color)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        pass

class NodeCondition(Node):
    ''' Condition Node
    Placeholder condition node
    '''
    def __init__(self,name,label,param_name=None,desired_value=None):
        L = '(condition)\\n' + label.upper()
        L_alt = '{CONDITION | ' + label.lower()+'}'
        color = '#FAE364'
        super(NodeCondition,self).__init__(name,L,color,'ellipse',alt_label=L_alt)
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'
    def execute(self):
        pass

