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

    def __init__(self, name, label, color='', shape='box', flag=False):
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
        if self.flag_ == False:
            if self.color_ == '':
                dot = dot + self.name_ + ' [shape='+self.shape_+'][URL="' +self.name_+'"][label="'+self.label_+'"]; '
            else:
                dot = dot + self.name_ + ' [shape='+self.shape_+'][URL="' +self.name_+'"][style="filled" fillcolor="'+self.color_+'"][label="'+self.label_+'"]; '
        else:
            if self.color_ == '':
                dot = dot + self.name_ + ' [shape='+self.shape_+'][URL="' +self.name_+'"][style="bold" color="red"][label="'+self.label_+'"]; '
            else:
                dot = dot + self.name_ + ' [shape='+self.shape_+'][URL="' +self.name_+'"][style="filled, bold" fillcolor="'+self.color_+'" color="red"][label="'+self.label_+'"]; '   
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
        for C in self.children_:
            C.reset()

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

    def set_children_number(self,number):
        """ Set the number of children this node has
        """
        self.children_number_ = number

    def set_status(self,status):
        """ Set the status of this node as SUCCESS, RUNNING, FAILURE or NODE_ERROR
        @type status: String
        @param status: the status value
        """
        self.node_status_ = status
        # print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_

    def execute(self):
        """ Virtual function that each node runs when that node gets ticked
        """
        pass

### CORE LOGICAL NODES ----------------------------------------------------------------------------------------

class NodeSelector(Node):
    ''' Runs children in order until one succeeds then 
       returns SUCCESS, if all fail, returns FAILURE.
    '''
    def __init__(self,name,label):
        L = '( * )\\n ' + label.upper()
        super(NodeSelector,self).__init__(name,L)
    def get_node_type(self):
        return 'SELECTOR'
    def get_node_name(self):
        return 'Selector'
    def execute(self):
        print 'Executing selector: (' + self.name_ + ')'

        if self.exec_index == None:
            self.exec_index = 0

        self.child_status_ = self.children_[self.exec_index].execute()

        if self.child_status_ == 'FAILURE':
            self.exec_index += 1
            if self.exec_index == self.num_children_: # Last Child
                self.exec_index = None 
                return self.set_status('FAILURE')
            else:
                return self.set_status('RUNNING')
        elif self.child_status_ == 'SUCCESS':
            self.exec_index = None 
            return self.set_status('SUCCESS')
        else:
            return self.set_status(self.child_status_)

class NodeSequence(Node):
    """ Sequence Node
        The sequence node executes its children in order or insertion.  If a child
        fails, the node will return FAILURE. If a child succeeds, the sequence will
        then execute the next child until all children are executed, then return
        SUCCESS.
    """
    def __init__(self,name,label):
        L = '->'
        # L = '( --> )\\n ' + label.upper()
        super(NodeSequence,self).__init__(name,L)
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):
        print 'Executing sequence: (' + self.name_ + ')'

        if self.exec_index == None:
            self.exec_index = 0

        self.child_status_ = self.children_[self.exec_index].execute()
        if self.child_status_ == 'SUCCESS':
            print 'got success'
            self.exec_index += 1
            print 'execution index =' + str(self.exec_index)
            print 'num_children_ =' + str(self.num_children_)
            if self.exec_index == self.num_children_: # Last Child
                self.exec_index = None 
                return self.set_status('SUCCESS')
            else:
                return self.set_status('RUNNING')
        elif self.child_status_ == 'FAILURE':
            self.exec_index = None 
            return self.set_status('FAILURE')
        else:
            return self.set_status(self.child_status_)

class NodeIterator(Node):
    """ Iterator Node
        The iterator node executes its children in order or insertion, ignoring
        failure, and will return SUCCESS when all children have returned either 
        SUCCESS or FAILURE.
    """
    def __init__(self,name,label):
        L = '-|'
        # L = '( --> )\\n ' + label.upper()
        super(NodeIterator,self).__init__(name,L)
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):
        print 'Executing selector: (' + self.name_ + ')'

        if self.exec_index == None:
            self.exec_index = 0

        self.child_status_ = self.children_[self.exec_index].execute()
        # If child reports success or failure, execute next child
        if self.child_status_ == 'SUCCESS':
            self.exec_index += 1
            if self.exec_index == self.num_children_: # Last Child
                return self.set_status('SUCCESS')
                self.exec_index = None 
            else:
                return self.set_status('RUNNING')
        elif self.child_status_ == 'FAILURE':
            self.exec_index += 1
            if self.exec_index == self.num_children_: # Last Child
                return self.set_status('SUCCESS')
                self.exec_index = None 
            else:
                return self.set_status('RUNNING')
        else:
            return self.set_status(self.child_status_)        

class NodeParallelAll(Node):
    ''' 
        Executes all children close to simultaneously.  If any fail, returns
        FAILURE. Once all succeed, returns SUCCESS.  Until that point returns
        RUNNING.
    '''
    def __init__(self,name,label):
        L = '|A|'# + label.upper()
        super(NodeParallelAll,self).__init__(name,L)
        self.num_success = None
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):
        print 'Executing Parallel: (' + self.name_ + ')'
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
    ''' 
        Executes all children close to simultaneously.  If any fail, returns
        FAILURE. Once all succeed, returns SUCCESS.  Until that point returns
        RUNNING. As nodes succeed, they are prevented from being ticked again
    '''
    def __init__(self,name,label):
        L = '|R|'# + label.upper()
        super(NodeParallelRemove,self).__init__(name,L)
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

        print 'Executing Parallel: (' + self.name_ + ')'

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
    ''' 
        Executes all children close to simultaneously.  If any fail, returns
        FAILURE. Once ONE succeeds, returns SUCCESS.  Until that point returns
        RUNNING.
    '''
    def __init__(self,name,label):
        L = '|1|'# + label.upper()
        super(NodeParallelOne,self).__init__(name,L)
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):

        print 'Executing Parallel: (' + self.name_ + ')'

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
    def __init__(self,name,label,runs=1):
        L = '(runs)\\n['+str(runs)+']'
        super(NodeDecoratorRepeat,self).__init__(name,L,shape='diamond')
        self.runs_ = runs
        self.num_runs_ = 0
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        print 'Executing Repeat Decorator: (' + self.name_ + ')'
        if self.num_runs_ < self.runs_:
            self.child_status_ = self.children_[0].execute()
            if self.child_status_ == 'SUCCESS':
                self.num_runs_ += 1
            elif self.child_status_ == 'RUNNING':
                returnself.set_status('RUNNING')
            elif self.child_status_ == 'FAILURE':
                return self.set_status('FAILURE')
        else:
            return self.set_status('SUCCESS')

class NodeDecoratorIgnoreFail(Node):
    def __init__(self,name,label):
        L = '(ignore)\\n['+str(runs)+']'
        super(NodeDecoratorIgnoreFail,self).__init__(name,L,shape='diamond')
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        print 'Executing Repeat Decorator: (' + self.name_ + ')'
        self.child_status_ = self.children_[0].execute()            
        if self.child_status_ == 'RUNNING':
            return self.set_status('RUNNING')
        else:
            return self.set_status('SUCCESS')

class NodeRoot(Node):
    def __init__(self, name, label):
        L = '(/)'
        super(NodeRoot,self).__init__(name,L)
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
    def __init__(self,name,label):
        L = '( action )\\n' + label.upper()
        color='#92D665'
        super(NodeAction,self).__init__(name,L,color)
        self.name_ = name
    def get_node_type(self):
        return 'ACTION'
    def get_node_name(self):
        return 'Action'
    def execute(self):
        print 'Executing Action: (' + self.name_ + ')'
        return self.set_status('SUCCESS')

class NodeService(Node):
    def __init__(self,name,label):
        color='#92D665'
        super(NodeService,self).__init__(name,label,color)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        print 'Executing Service: ' + self.name_
        return self.set_status('SUCCESS')

class NodeCondition(Node):
    def __init__(self,name,label,param_name=None,desired_value=None):
        L = '( condition )\\n' + label.upper()
        color = '#FAE364'
        super(NodeCondition,self).__init__(name,L,color,'ellipse')
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'
    def execute(self):
        print 'Executing Condition: (' + self.name_ + ')'
        return self.set_status('SUCCESS')


