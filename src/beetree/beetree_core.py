#!/usr/bin/env python
import roslib; roslib.load_manifest('beetree')
import rospy 
from std_msgs.msg import *

class Node(object):
    """Base class for beetree behavior_tree nodes.

    This node has the basic structure for a node, as well as functions for recursively printing information, generating graphviz dot code, adding child nodes and resetting variables.
    """
    def __init__(self,is_root=False,parent=None,name='',label='',color='',shape='box',flag=False):
        """Node constructor
        @type is_root: bool
        @param is_root: determines whether the node is root or not.

        @type parent: string
        @param parent: the parent of the node if it is not root

        @type name: string
        @param name: the name of the node. This is also its graphviz URL.

        @type label: string
        @param label: the label of the node, which will be displayed in the dot output as its icon label

        @todo Make it an option to show or hide text labels on logical nodes such as parallel or selector
        @todo Visualize the tick with graphviz color change?
        """
        self.number_children_ = 0
        self.children_number_ = 0
        self.highlighted_ = False
        self.overwritten_ = False
        self.overwritten_result_ = 'NODE_ERROR'
        self.node_status_ = 'NODE_ERROR'
        self.child_status_ = 'NODE_ERROR'
        self.first_child_ = None
        self.curr_child_ = None
        self.exec_child_ = None
        self.next_brother_ = None
        self.prev_brother_ = None
        self.name_ = name
        self.label_ = label
        self.color_ = color
        self.shape_ = shape
        self.flag_ = flag
                
        if is_root:
            self.depth_ = 0
            self.parent_ = None
        else:
            if parent != None:
                self.depth_ = parent.depth_ + 1
                self.parent_ = parent
                self.parent_.add_child(self)
            else:
                # No Parent assigned, wont be added to the tree
                pass

    def set_flag(self,flag):
        """ sets a flag for whether this node will be highlighted in the dot code
        """
        self.flag_ = flag

    def generate_dot(self):
        """generates dot code for this node
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
        if self.number_children_ > 0:
            current = self.first_child_
            for n in range(self.number_children_):
                # call the current child's generate_dot function (for recursion)
                dot += current.generate_dot()
                # generate the dot for the connection between self and the current child
                dot += self.name_ + ':s->' + current.name_ + ':n; '
                current = current.next_brother_
                if current == None:
                    break

        # if parent generate tail end of dotcode string
        if self.parent_ == None:        
            return dot + '}'
        else:
            return dot

    def execute_reset_status(self):
        """resets the node's status
        resets the node's status
        """
        self.node_status_ = 'NODE_ERROR'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            self.exec_child_.execute_reset_status()
            self.exec_child_ = self.exec_child_.next_brother_

    def add_child(self, my_child):
        """adds a child to the node, and adds it as a brother if there are existing NodeSelector

        @type my_child: beetree.Node
        @param my_child: the child Node to be added to this node
        """
        if self.number_children_ == 0:
            print self.name_ + ': adding first child -> ' + my_child.name_
            self.first_child_ = self.curr_child_ = my_child
            self.number_children_+=1
        else:
            print self.name_ + ': adding brother -> ' + my_child.name_
            self.curr_child_ = self.curr_child_.add_brother(my_child, self.number_children_)
            self.number_children_+=1
        return self.curr_child_

    def add_sibling(self,sibling):
        # Assign sibling depth and parent
        sibling.depth_ = self.depth_
        sibling.parent_ = self.parent_
        # Increment parent's number of children
        self.parent_.number_children_+=1
        # Add sibling to the right of this node
        next = self.next_brother_
        self.next_brother_ = sibling
        if next != None:
            self.next_brother_.next_brother_ = next

    def add_brother(self,my_brother,children_number):
        """adds a brother to the node
        explicitely adds a node as a brother of existing child NodeSelector

        @type my_brother: beetree.Node
        @param my_brother: the node to insert as a brother

        @type children_number: int
        @param children_number: the number of children in the node
        """
        self.next_brother_ = my_brother
        self.next_brother_.set_prev_brother(self)
        self.next_brother_.set_children_number(self.children_number_+1)
        return self.next_brother_

    def remove_child(self,child_name):
        if self.number_children_ == 0:
            print 'node has no childern'
        elif self.number_children_ == 1:
            self.first_child_.remove_all_children()
            self.first_child_ = None
            self.curr_child_ = None
            self.number_children_ = 0
        else:
            current = self.first_child_
            for n in range(self.number_children_):
                if child_name == current.name_:
                    if current.prev_brother_ == None: 
                        # This is the first child of several so we should point 
                        # to the its brother as the first one, and remove it
                        current.remove_all_children()
                        current.next_brother_.prev_brother_ = None
                        self.first_child_ = self.curr_child_ = current.next_brother_
                        break
                    else:
                        # This is the brother of a previous child
                        current.remove_all_children()
                        if current.next_brother_ == None:
                            # This is the last brother
                            current.prev_brother_.next_brother_ = None
                        else:
                            # This has a next brother
                            current.next_brother_.prev_brother_ = current.prev_brother_
                            current.prev_brother_.next_brother_ = current.next_brother_
                        break
                else: 
                    # get next sibling if this one doesnt match
                    current = current.next_brother_
            self.number_children_ -= 1

    def remove_all_children(self):
        current = self.first_child_
        for n in range(self.number_children_):
            current.remove_all_children()
            current = current.next_brother_
        self.first_child_ = self.curr_child_ = None
        self.number_children_ = 0

    def remove_self(self):
        if self.parent_ != None:
            self.parent_.remove_child(self.name_)
        else:
            print 'You cannot remove the root node'

    def print_info(self):
        """prints the nodes info
        prints the nodes info
        """
        print 'Depth: ' + str(self.depth_)
        print '-- Name: ' + self.name_
        print '-- Status: ' + self.node_status_
        print '-- Number of Children: ' + str(self.number_children_)

    def print_subtree(self):
        """prints the info recursively for the tree under this node
        prints the info recursively for the tree under this node
        """
        self.print_info()
        if self.number_children_ > 0:
            self.first_child_.print_subtree()

        if self.next_brother_ != None:
            self.next_brother_.print_subtree()

    def set_children_number(self,number):
        self.children_number_ = number

    def set_next_brother(self,brother):
        self.next_brother_ = brother

    def set_prev_brother(self,brother):
        self.prev_brother_ = brother

    def set_highlighted(self,highlight):
        self.highlighted_ = highlight

    def set_overwrite(self,overwritten):
        self.overwritten_result_ = state
        self.overwritten_ = overwritten

    def set_status(self,status):
        self.node_status_ = status
        print '  -  Node: ' + self.name_ + ' returned status: ' + self.node_status_
        return self.node_status_


## Specialized Nodes
class NodeSelector(Node):

    def __init__(self,parent,name,label):
        L = '( * )\\n ' + label.upper()
        super(NodeSelector,self).__init__(False,parent,name,L)
    def get_node_type(self):
        return 'SELECTOR'
    def get_node_name(self):
        return 'Selector'
    def execute(self):
        print 'executing selector: (' + self.name_ + ')'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            print 'ticking child' + str(i)
            self.child_status_ = self.exec_child_.execute()

            print 'checking child status: ' + self.child_status_
            if self.child_status_ == 'NODE_ERROR':
                return self.set_status('NODE_ERROR')
            elif self.child_status_ == 'RUNNING':
                return self.set_status('RUNNING')
            elif self.child_status_ == 'SUCCESS':
                return self.set_status('SUCCESS')

            print 'pointing self.exec_child_ to next brother'
            self.exec_child_ = self.exec_child_.next_brother_

        return self.set_status('FAILURE')

class NodeSequence(Node):
    """sequence type node

    This node runs its children in order acording to the order of insertion.  If any child fails, this node will return FAILURE.  If a child succeeds this node will call the next child in order. When the last child succeeds this node will return SUCCESS.
    """
    def __init__(self,parent,name,label):
        L = '->'
        # L = '( --> )\\n ' + label.upper()
        super(NodeSequence,self).__init__(False,parent,name,L)
    def get_node_type(self):
        return 'SEQUENCE'
    def get_node_name(self):
        return 'Sequence'
    def execute(self):
        self.exec_child_ = self.first_child_
        print 'Executing Sequence: (' + self.name_ + '): current child: ' + self.exec_child_.name_

        for i in range(self.number_children_):
            self.child_status_ = self.exec_child_.execute()
            if self.child_status_ == 'NODE_ERROR':
                return self.set_status('NODE_ERROR')
            elif self.child_status_ == 'RUNNING':
                return self.set_status('RUNNING')
            elif self.child_status_ == 'FAILURE':
                return self.set_status('FAILURE')
            self.exec_child_ = self.exec_child_.next_brother_

        return self.set_status('SUCCESS')

class NodeParallel(Node):
    def __init__(self,parent,name,label):
        L = '||'# + label.upper()
        super(NodeParallel,self).__init__(False,parent,name,L)
    def get_node_type(self):
        return 'PARALLEL'
    def get_node_name(self):
        return 'Parallel'
    def execute(self):
        number_failure = 0
        number_success = 0
        number_error = 0

        print 'Executing Parallel: (' + self.name_ + ')'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            self.child_status_ = self.exec_child_.execute()
            if self.child_status_ == 'NODE_ERROR':
                number_error += 1
            elif self.child_status_ == 'FAILURE':
                number_failure  += 1
            elif self.child_status_ == 'SUCCESS':
                number_success += 1
            self.exec_child_ = self.exec_child_.next_brother_

        if number_error > 0:
            return self.set_status('NODE_ERROR')
        elif number_success>= self.number_children_/2:
            return self.set_status('SUCCESS')
        elif number_failure >= self.number_children_/2:
            return self.set_status('FAILURE')
        else:
            return self.set_status('RUNNING')

class NodeRoot(Node):
    def __init__(self, name, label):
        L = '?'
        # L = '( ? )\\n ' + label.upper()
        super(NodeRoot,self).__init__(True,None,name,L,'',)
    def get_node_type(self):
        return 'ROOT'
    def get_node_name(self):
        return 'Root'
    def execute(self):
        print 'Executing Root: (' + self.name_ + ')'
        self.child_status_ = self.first_child_.execute()
        print 'ROOT: Child returned status: ' + self.child_status_
        return self.child_status_

class NodeAction(Node):
    def __init__(self,parent,name,label):
        L = '( action )\\n' + label.upper()
        color='#92D665'
        super(NodeAction,self).__init__(False,parent,name,L,color)
        self.name_ = name
    def get_node_type(self):
        return 'ACTION'
    def get_node_name(self):
        return 'Action'
    def execute(self):
        print 'Executing Action: (' + self.name_ + ')'
        return self.set_status('SUCCESS')

class NodeService(Node):
    def __init__(self,parent,name,label):
        color='#92D665'
        super(NodeService,self).__init__(False,parent,name,label,color)
    def get_node_type(self):
        return 'SERVICE'
    def get_node_name(self):
        return 'Service'
    def execute(self):
        print 'Executing Service: ' + self.name_
        return self.set_status('SUCCESS')

class NodeCondition(Node):
    def __init__(self,parent,name,label,param_name=None,desired_value=None):
        L = '( condition )\\n' + label.upper()
        color = '#FAE364'
        super(NodeCondition,self).__init__(False,parent,name,L,color,'ellipse')
        self.desired_value_ = desired_value
        self.param_name_ = param_name
    def get_node_type(self):
        return 'CONDITION'
    def get_node_name(self):
        return 'Condition'
    def execute(self):
        print 'Executing Condition: (' + self.name_ + ')'
        return self.set_status('SUCCESS')

class NodeDecoratorRepeat(Node):
    def __init__(self,parent,name,label,runs=1):
        L = '(runs)\\n['+str(runs)+']'
        super(NodeDecoratorRepeat,self).__init__(False,parent,name,L,shape='diamond')
        self.runs_ = runs
        self.num_runs_ = 0
    def get_node_type(self):
        return 'DECORATOR_REPEAT'
    def get_node_name(self):
        return 'Decorator Repeat'
    def execute(self):
        print 'Executing Repeat Decorator: (' + self.name_ + ')'
        self.exec_child_ = self.first_child_
        if self.num_runs_ < self.runs_:
            self.child_status_ = self.exec_child_.execute()
            if self.child_status_ == 'SUCCESS':
                self.num_runs_ += 1
            elif self.child_status_ == 'RUNNING':
                pass
            elif self.child_status_ == 'FAILURE':
                return self.set_status('FAILURE')
        else:
            return self.set_status('SUCCESS')


