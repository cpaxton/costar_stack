#!/usr/bin/env python

import roslib; roslib.load_manifest('beetree')
import rospy 
from std_msgs.msg import *
from collections import OrderedDict
from copy import deepcopy
import random

class Node(object):
    """ Beetree Node
    The core class for beetree nodes.  Beetree nodes are connected together in a tree 
    using the add_child and related functions.  A beetree node can generate a graphviz
    dot file for visualization purposes.
    """

    def __init__(self, name, label, color='', shape='box', flag=False, alt_label=None, attach=True, view_mode='sequential',subtree_label='Subtree: Collapsed Nodes',size=None):
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
        self.attach = attach
        self.collapsed = False
        self.view_mode = view_mode
        self.subtree_label = subtree_label
        self.size = size

    def set_label(self, label):
        self.label_ = label

    def set_color(self,c):
        self.color_ = c

    def set_alt_view(self,v):
        self.alt_view = v

    def set_collapsed(self,val):
        self.collapsed = val
                
    def set_flag(self,flag):
        """ Sets a flag for whether this node will be highlighted in the dot code
        """
        self.flag_ = flag

    def generate_dot(self,run=False):
        """ Generates dot code for this node and its connection to its children
        Also recursively calls the children's generate_dot() functions
        """
        # rospy.logwarn(run)
        if run == True:
            if self.node_status_ == 'RUNNING':
                self.flag_ = True
            else:
                self.flag_ = False 

        # if parent generate front end of dotcode string
        if self.parent_ == None:
            dot = 'digraph behavior_tree {bgcolor="#222222" nodesep=.5 ranksep=.5 rankdir=LR splines=false; '
        else:
            dot = ''
        # generate this node's dot code
        if self.alt_view == True:
            shape = self.alt_shape_
            label = self.alt_label_
        else:
            shape = self.shape_
            label = self.label_

        if self.size != None:
            sz = str(self.size)
        else:
            sz = str(16)

        # Change shape for collapsed nodes
        if self.collapsed:
            # shape = 'diamond'
            color = '#34495E'
            style = 'dashed'
            if self.flag_ == False:
                dot = dot + self.name_ + ' [shape=box][URL="' +self.name_+'"][fontsize=13 fontname="times 13 bold" style="filled, '+style+'" fontcolor="#bbbbbb" color="#bbbbbb" fillcolor="'+color+'"][label="'+self.alt_label_+'"];'
            else:
                dot = dot + self.name_ + ' [shape=box][URL="' +self.name_+'"][fontsize=13 fontname="times 13 bold" style="filled, '+style+'" fontcolor="#bbbbbb" color="red" fillcolor="'+color+'"][label="'+self.alt_label_+'"];'

        else:
            color = self.color_
            style = 'bold'

            if self.flag_ == False:
                if color == '':
                    dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][fontsize='+sz+' fontname="times '+sz+' bold" style="'+style+'" fontcolor="#ffffff" color="#ffffff" label="'+label+'"]; '
                else:
                    dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][fontsize='+sz+' fontname="times '+sz+' bold" style="filled, '+style+'" fontcolor="#ffffff" color="#ffffff" fillcolor="'+color+'"][label="'+label+'"]; '
            else:
                if color == '':
                    dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][fontsize='+sz+' fontname="times '+sz+' bold" style="'+style+'" color="red"][label="'+label+'"]; '
                else:
                    dot = dot + self.name_ + ' [shape='+shape+'][URL="' +self.name_+'"][fontsize='+sz+' fontname="times '+sz+' bold" style="filled, '+style+'" fontcolor="#ffffff" fillcolor="'+color+'" color="red"][label="'+label+'"]; '   

        # recursively generate the node's child dot code
        if self.collapsed == False:
            if self.num_children_ > 0:
                first = True
                for C in self.children_:
                    dot += C.generate_dot(run)
                    if C.attach:
                        # dot += self.name_ + ':s->' + C.name_ + ':n; '
                        if self.view_mode == 'sequential':
                            dot += self.name_ + ':e->' + C.name_ + ':w [color="#ffffff"]; '
                        elif self.view_mode == 'first':
                            if first:
                                dot += self.name_ + ':e->' + C.name_ + ':w [color="#ffffff"]; '
                                first = False
                            else:
                                dot += self.name_ + ':e->' + C.name_ + ':w [style="dashed" color="#888888"]; '
                    else:
                        print "NOT ATTACHED"

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
            self.children_names_.pop(existing_child_index)
            self.children_names_.append(replacement_child.name_)
            return True
        else:
            return False

    def replace_self(self,replacement_child):
        self.parent_.replace_child(self,replacement_child)

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
        if child_to_remove in self.children_:
            index = self.children_.index(child_to_remove)
            child = self.children_.pop(index)
            child.remove_all_children()
            self.children_names_.pop(index)
            self.num_children_ -= 1    
            return True
        else:
            return False

    def remove_all_children(self):
        """ Remove all children of this node
        """
        self.children_ = []
        self.children_names_ = []
        self.num_children_ = 0

    def remove_self(self):
        """ Remove this node from its parents list of children and all connections to the tree
        """
        return self.parent_.remove_child(self)

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

    def get_child_names(self):
        return deepcopy(self.children_names_)

    def execute(self):
        """ Virtual function that each node runs when that node gets ticked
        """
        pass

### CORE LOGICAL NODES ###
#----------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------
class NodeSelector(Node):
    ''' Selector Node
    Runs children in order until one succeeds then 
    returns SUCCESS, if all fail, returns FAILURE.
    '''
    def __init__(self,name,label):
        L = '?'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeSelector,self).__init__(name,L,color,alt_label=L_alt,view_mode='first',shape='circle')
    def get_node_type(self):
        return 'SELECTOR'
    def get_node_name(self):
        return 'Selector'
    def execute(self):

        if len(self.children_) == 0:
            rospy.logwarn("sequence ["+self.name_+"] has no children, returning success.")
            return self.set_status('SUCCESS')

        for child in self.children_:
            status = child.execute()

            if status[:7] != 'FAILURE':
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
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeSequence,self).__init__(name,L,color,alt_label=L_alt,shape='diamond')
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):

        if len(self.children_) == 0:
            rospy.logwarn("sequence ["+self.name_+"] has no children, returning success.")
            return self.set_status('SUCCESS')

        for child in self.children_:
            status = child.execute()

            if status is None:
                return self.set_status('FAILURE')
            elif status[:7] != 'SUCCESS':
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
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeIterator,self).__init__(name,L,color,alt_label=L_alt)
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):

        for child in self.children_:
            status = child.execute()
            
            if status is None:
                return self.set_status('FAILURE')
            if status[:7] != 'SUCCESS' and status[:7] != 'FAILURE':
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
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeParallelAll,self).__init__(name,L,color,alt_label=L_alt)
        self.num_success = 0
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):
        # print 'Executing Parallel: (' + self.name_ + ')'
        self.num_success = 0

        for C in self.children_:
            self.child_status_ = C.execute()
            if self.child_status_ == 'NODE_ERROR':
                self.num_success = None
                return self.set_status('NODE_ERROR')
            elif self.child_status_[:7] == 'FAILURE':
                self.num_success = None
                return self.set_status('FAILURE')
            elif self.child_status_[:7] == 'RUNNING':
                pass
            elif self.child_status_[:7] == 'SUCCESS':
                self.num_success += 1

        # Only return if all children succeed
        if self.num_success == self.num_children_:
            return self.set_status('SUCCESS')
        else:
            return self.set_status('RUNNING')
    def reset(self):
        super(NodeParallelAll, self).reset()
        self.num_success = 0

class NodeParallelRemove(Node):
    ''' Parallel Remove Node
    Executes all children close to simultaneously.  If any fail, returns
    FAILURE. Once all succeed, returns SUCCESS.  Until that point returns
    RUNNING. As nodes succeed, they are prevented from being ticked again
    '''
    def __init__(self,name,label):
        L = '|R|'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeParallelRemove,self).__init__(name,L,color,alt_label=L_alt)
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
            elif self.child_status_[:7] == 'FAILURE':
                return self.set_status('FAILURE')
            elif self.child_status_[:7] == 'RUNNING':
                pass
            elif self.child_status_[:7] == 'SUCCESS':
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
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeParallelOne,self).__init__(name,L,color,alt_label=L_alt)
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
            elif self.child_status_[:7] == 'FAILURE':
                return self.set_status('FAILURE')
            elif self.child_status_ == 'RUNNING':
                pass
            elif self.child_status_[:7] == 'SUCCESS':
                return self.set_status('SUCCESS')

        return self.set_status('RUNNING')

class NodeDecoratorRepeat(Node):
    ''' Decorator Repeat Node
    Executes child node N times. If N = -1, will repeat forever, ignoring failures
    '''
    def __init__(self,name,label,runs=1):
        if runs == -1:
            L = 'REPEAT\\nFOREVER'
        else:
            L = 'REPEAT [0/'+str(runs)+']'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        
        super(NodeDecoratorRepeat,self).__init__(name,L,color,shape='invhouse',alt_label=L_alt,size=12)
        self.runs_ = runs
        self.num_runs_ = 0
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        if self.runs_ == -1: # run forever
            self.child_status_ = self.children_[0].execute()
            if self.child_status_[:7] == 'SUCCESS':
                rospy.logwarn('REPEAT DECORATOR ['+self.name_+']: SUCCEEDED, RESET')
                self.children_[0].reset()
                return self.set_status('RUNNING')
            elif self.child_status_[:7] == 'RUNNING':
                return self.set_status('RUNNING')
            elif self.child_status_[:7] == 'FAILURE':
                return self.set_status('FAILURE')
        else:
            # TODO(cpaxton) or TODO(fjonath) figure out why this does not work
            # right. Counter increments when it should not.
            if self.num_runs_ < self.runs_:
                self.child_status_ = self.children_[0].execute()
                if self.child_status_[:7] == 'SUCCESS':
                    self.num_runs_ += 1
                elif self.child_status_[:7] == 'RUNNING':
                    return self.set_status('RUNNING')
                elif self.child_status_[:7] == 'FAILURE':
                    return self.set_status('FAILURE')
            else:
                return self.set_status('SUCCESS')
            L = 'REPEAT [%d/%d]'%(self.num_runs_,self.runs_)
            self.set_label(L)

    def reset(self):
        super(NodeDecoratorRepeat, self).reset()
        if not self.runs_ == -1:
            self.num_runs_ = 0

class NodeDecoratorIgnoreFail(Node):
    ''' Decorator Ignore Fail
    Returns success regardless of outcome of child
    '''
    def __init__(self,name,label):
        L = 'IGNORE\\nFAILURE'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeDecoratorIgnoreFail,self).__init__(name,L,color,shape='invhouse',alt_label=L_alt,size=12)
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
        L = 'WAIT\\nSUCCESS'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        super(NodeDecoratorWaitForSuccess,self).__init__(name,L,color,shape='invhouse',alt_label=L_alt,size=12)
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
                if self.child_status_[:7] == 'SUCCESS':
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
                if self.child_status_[:7] == 'SUCCESS':
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

class NodeDecoratorReset(Node):
    ''' Decorator Reset
    Resets a node and returns its return value
    '''
    def __init__(self,name,label,runs=-1):
        L = 'RESET'
        if label != '':
            L_alt = label
        else:
            L_alt = name.upper()+' Subtree'
        color='#22A7F0'
        self.runs_ = runs
        self.num_runs_ = 0 
        super(NodeDecoratorReset,self).__init__(name,L,color,shape='house',alt_label=L_alt,size=12)
    def get_node_type(self):
        return 'DECORATOR_RESET'
    def get_node_name(self):
        return 'Decorator Reset'
    def execute(self):
        if self.runs_ == -1: # resets forever
            self.child_status_ = self.children_[0].execute()
            if self.child_status_[:7] == 'SUCCESS':
                # self.children_[0].reset()
                return self.set_status('SUCCESS')
            elif self.child_status_[:7] == 'FAILURE':
                self.children_[0].reset()
                return self.set_status('FAILURE')
            elif self.child_status_[:7] == 'RUNNING':
                return self.set_status('RUNNING')
        else:
            if self.num_runs_ < self.runs_:
                self.num_runs_ += 1 # always increment  
                self.child_status_ = self.children_[0].execute()
                if self.child_status_[:7] == 'SUCCESS':
                    self.children_[0].reset()
                    return self.set_status('SUCCESS')
                elif self.child_status_[:7] == 'FAILURE':
                    self.children_[0].reset()
                    return self.set_status('FAILURE')
                elif self.child_status_ == 'RUNNING':
                    return self.set_status('RUNNING')

class NodeRoot(Node):
    ''' Root Node
    The root node can have only one child, and typically is used as the root of 
    the tree
    '''
    def __init__(self, name, label):
        L = 'ROOT'
        L_alt = "I am ROOT"
        color='#22A7F0'
        super(NodeRoot,self).__init__(name,L,color,alt_label=L_alt)
    def get_node_type(self):
        return 'ROOT'
    def get_node_name(self):
        return 'Root'
    def execute(self):
        # if self.num_children_ > 0:
            # return self.children_[0].execute()
        ### Solution to non attached children like variables... needs work
        if self.num_children_ > 0:
            for c in self.children_:
                if c.attach == True:
                    return c.execute()

class NodeAction(Node):
    ''' Action Node
    Placeholder action node
    '''
    def __init__(self,name,label):
        L = 'Action\\n' + label
        L_alt = '{ACTION|' + label.lower()+'}'
        color='#26A65B'
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
        color='#26A65B'
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
        L = label
        L_alt = '{CONDITION | ' + label.lower()+'}'
        color = '#DB0A5B'
        # super(NodeCondition,self).__init__(name,L,color,'ellipse',alt_label=L_alt)
        super(NodeCondition,self).__init__(name,L,color,alt_label=L_alt)
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'
    def execute(self):
        pass


class NodeQuery(Node):
    ''' Query Node
    Placeholder Query node
    '''
    def __init__(self,name,label,param_name=None,desired_value=None,attach=True):
        L = name.upper()+'\\n[\\"' + label.upper() + '\\"]'
        L_alt = '{QUERY | ' + label.lower()+'}'
        color = '#E87E04'
        att = attach
        super(NodeQuery,self).__init__(name,L,color,alt_label=L_alt,attach=att)
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'QUERY'
    def get_node_name(self):
        return 'Query'
    def execute(self):
        pass

class NodeCalculation(Node):
    ''' Calculation Node
    Placeholder Calculation node
    '''
    def __init__(self,name,label,param_name=None,desired_value=None):
        L = 'Calculation\\n' + label
        L_alt = '{CALCULATION | ' + label.lower()+'}'
        color = '#BC83DE'
        super(NodeCalculation,self).__init__(name,L,color,alt_label=L_alt)
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'CALCULATION'
    def get_node_name(self):
        return 'Calculation'
    def execute(self):
        pass


class NodeVariable(Node):
    ''' Variable Node
    Placeholder Variable node
    '''
    def __init__(self,name,label,attach=False):
        L = 'VARIABLE\\n[' + label + ']'
        L_alt = '{VARIABLE | ' + label.lower()+'}'
        color = '#BC83DE'
        att = attach
        super(NodeVariable,self).__init__(name,L,color,alt_label=L_alt,attach=att)
        # self.desired_value_ = desired_value
        # self.param_name_ = param_name
    def get_node_type(self):
        return 'VARIABLE'
    def get_node_name(self):
        return 'Variable'
    def execute(self):
        pass
