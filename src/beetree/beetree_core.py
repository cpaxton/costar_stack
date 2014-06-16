#!/usr/bin/env python
import roslib; roslib.load_manifest('beetree')
import rospy 
from std_msgs.msg import *

class Node(object):

    def __init__(self,is_root=False,parent=None,name='',label=''):

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
                
        if is_root:
            self.depth_ = 0
            self.parent_ = None
        else:
            self.depth_ = parent.depth_ + 1
            self.parent_ = parent
            self.parent_.add_child(self)

    def generate_dot(self):
        if self.parent_ == None:
            dot = 'digraph behavior_tree { '
        else:
            dot = ''

        if self.number_children_ != 0:
            dot = dot + self.name_ + ' [shape=box][URL="' +self.name_+'"][label="'+self.label_+'"]; '
        
        if self.number_children_ > 0:
            current = self.first_child_
            if current.next_brother_ == None:
                dot = dot + current.name_ + '[URL="' +current.name_+'"][label="'+current.label_+'"];' + self.name_ + '->' + current.name_ + '; '
                dot = dot + current.generate_dot()
            else:
                for n in range(self.number_children_):
                # while current.next_brother_ != None:
                    dot = dot + current.name_ + '[URL="' +current.name_+'"][label="'+current.label_+'"];' + self.name_ + '->' + current.name_ + '; '
                    dot = dot +  current.generate_dot()
                    current = current.next_brother_

        if self.parent_ == None:        
            return dot + '}'
        else:
            return dot

    def execute_reset_status(self):
        self.node_status_ = 'NODE_ERROR'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            self.exec_child_.execute_reset_status()
            self.exec_child_ = self.exec_child_.next_brother_

    def add_child(self, my_child):
        if self.number_children_ == 0:
            print self.name_ + ': adding first child -> ' + my_child.name_
            self.first_child_ = self.curr_child_ = my_child
        else:
            print self.name_ + ': adding brother -> ' + my_child.name_
            self.curr_child_ = self.curr_child_.add_brother(my_child, self.number_children_)

        self.number_children_+=1
        return self.curr_child_

    def add_brother(self,my_brother,children_number):
        self.next_brother_ = my_brother
        self.next_brother_.set_prev_brother(self)
        self.next_brother_.set_children_number(self.children_number_+1)
        return self.next_brother_

    def print_info(self):
        print 'Depth: ' + str(self.depth_)
        print '-- Name: ' + self.name_
        print '-- Status: ' + self.node_status_
        print '-- Number of Children: ' + str(self.number_children_)

    def print_subtree(self):
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

    def set_overwrite(state,overwritten):
        self.overwritten_result_ = state
        self.overwritten_ = overwritten

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
        print 'executing selector'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            print 'ticking child' + str(i)
            self.child_status_ = self.exec_child_.execute()

            print 'checking child status: ' + self.child_status_
            if self.child_status_ == 'NODE_ERROR':
                self.node_status_ = 'NODE_ERROR'
                return self.node_status_
            elif self.child_status_ == 'RUNNING':
                self.node_status_ = 'RUNNING'
                return self.node_status_
            elif self.child_status_ == 'SUCCESS':
                self.node_status_ = 'SUCCESS'
                return self.node_status_

            print 'pointing self.exec_child_ to next brother'
            self.exec_child_ = self.exec_child_.next_brother_

        self.node_status_ = 'FAILURE'
        return self.node_status_


class NodeSequence(Node):

    def __init__(self,parent,name,label):
        L = '( --> )\\n ' + label.upper()
        super(NodeSequence,self).__init__(False,parent,name,L)

    def get_node_type(self):
        return 'SEQUENCE'

    def get_node_name(self):
        return 'Sequence'

    def execute(self):
        print 'Executing Sequence'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            self.child_status_ = self.exec_child_.execute()
            if self.child_status_ == 'NODE_ERROR':
                self.node_status_ = 'NODE_ERROR'
                return self.node_status_
            elif self.child_status_ == 'RUNNING':
                self.node_status_ = 'RUNNING'
                return self.node_status_
            elif self.child_status_ == 'SUCCESS':
                self.node_status_ = 'SUCCESS'
                return self.node_status_
            self.exec_child_ = self.exec_child_.next_brother_

        self.node_status_ = 'SUCCESS'
        return self.node_status_


class NodeParallel(Node):

    def __init__(self,parent,name,label):
        L = '( || )'# + label.upper()
        super(NodeParallel,self).__init__(False,parent,name,L)

    def get_node_type(self):
        return 'PARALLEL'

    def get_node_name(self):
        return 'Parallel'

    def execute(self):
        number_failure = 0
        number_success = 0
        number_error = 0

        print 'Executing Parallel'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            self.child_status_ = self.exec_child_.execute()
            if self.child_status_ == 'NODE_ERROR':
                number_error += 1
            elif self.child_status_ == 'FAILURE':
                number_failure  += 1
            elif self.child_status_ == 'SUCCESS':
                number_success += 1

        if number_error > 0:
            self.node_status_ = 'NODE_ERROR'
            return self.node_status_
        elif number_success>= self.number_children_/2:
            self.node_status_ = 'SUCCESS'
            return self.node_status_
        elif number_failure >= self.number_children_/2:
            self.node_status_ = 'FAILURE'
            return self.node_status_
        else:
            self.node_status_ = 'RUNNING'
            return self.node_status_


class NodeRoot(Node):

    def __init__(self, name, label):
        L = '( / )\\n ' + label.upper()
        super(NodeRoot,self).__init__(True,None,name,L)

    def get_node_type(self):
        return 'ROOT'

    def get_node_name(self):
        return 'Root'

    def execute(self):
        print 'Executing Root'
        self.child_status_ = self.first_child_.execute()
        return self.child_status_

# TODO
class NodeAction(Node):

    def __init__(self,parent,name,label):
        L = '( act )\\n' + label.upper()
        super(NodeAction,self).__init__(False,parent,name,L)
        self.name_ = name
    def get_node_type(self):
        return 'ACTION'

    def get_node_name(self):
        return 'Action'

    def execute(self):
        print 'Executing Action: ' + self.name_

# TODO
class NodeService(Node):

    def __init__(self,parent,name,label):
        super(NodeService,self).__init__(False,parent,name,label)

    def get_node_type(self):
        return 'SERVICE'

    def get_node_name(self):
        return 'Service'

    def execute(self):
        pass


class NodeParamCondition(Node):

    def __init__(self,parent,name,label,param_name=None,desired_value=None):
        L = '( if/then )\\n' + label.upper()
        super(NodeParamCondition,self).__init__(False,parent,name,L)
        self.desired_value_ = desired_value
        self.param_name_ = param_name

    def get_node_type(self):
        return 'CONDITION'

    def get_node_name(self):
        return 'Condition'

    def execute(self):
        print 'Executing Condition'
        
        if not rospy.has_param(self.param_name_):
            self.node_status_ = 'FAILURE'
            return self.node_status_

        value = rospy.get_param(self.param_name_)

        if value == self.desired_value_:
            self.node_status_ = 'SUCCESS'
            return self.node_status_
        else:
            self.node_status_ = 'FAILURE'
            return self.node_status_


class NodeDecoratorRunNumber(Node):

    def __init__(self,parent,name,label,runs=1):
        super(NodeDecoratorRunNumber,self).__init__(False,parent,name,label)
        self.runs_ = runs
        self.num_runs_ = 0

    def get_node_type(self):
        return 'DECORATOR_RUN_NUMBER'

    def get_node_name(self):
        return 'Decorator Run Number'

    def execute(self):
        print 'Executing Run Number Decorator'
        self.exec_child_ = self.first_child_

        if self.num_runs_ < self.runs_:
            self.child_status_ = self.exec_child_.execute()
            if self.child_status_ == 'SUCCESS':
                self.num_runs_ += 1
            elif self.child_status_ == 'FAILURE':
                self.node_status_ = 'FAILURE'
                return self.node_status_
        else:
            self.node_status_ = 'SUCCESS'
            return self.node_status_


