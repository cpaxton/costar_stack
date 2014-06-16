#!/usr/bin/env python
import roslib; roslib.load_manifest('beetree')
import rospy 


class Node(object):

    def __init__(self,is_root=False,parent=None,name='unnamed'):

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

        if is_root:
            self.depth_ = 0
            self.parent_ = None
        else:
            self.depth_ = parent.depth_ + 1
            self.parent_ = parent
            self.parent_.add_child(self)

    def execute_reset_status(self):
        self.node_status_ = 'NODE_ERROR'
        self.exec_child_ = self.first_child_

        for i in range(self.number_children_):
            self.exec_child_.execute_reset_status()
            self.exec_child_ = self.exec_child_.next_brother_

    def add_child(self, my_child):
        if self.number_children_ == 0:
            print self.name_ + ' adding first child ' + my_child.name_
            self.first_child_ = self.curr_child_ = my_child
        else:
            print self.name_ + ' adding brother ' + my_child.name_
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

    def __init__(self,parent,name='unnamed'):
        super(NodeSelector,self).__init__(False,parent,name)

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

    def __init__(self,parent,name='unnamed'):
        super(NodeSequence,self).__init__(False,parent,name)

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

    def __init__(self,parent,name='unnamed'):
        super(NodeParallel,self).__init__(False,parent,name)

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

    def __init__(self):
        super(NodeRoot,self).__init__(True, None,'root')

    def get_node_type(self):
        return 'ROOT'

    def get_node_name(self):
        return 'Root'

    def execute(self):
        print 'Executing Root'
        self.child_status_ = self.first_child_.execute()
        return self.child_status_


class NodeAction(Node):

    def __init__(self,parent,name='unnamed'):
        super(NodeAction,self).__init__(False,parent,name)
        self.name_ = name
    def get_node_type(self):
        return 'ACTION'

    def get_node_name(self):
        return 'Action'

    def execute(self):
        print 'Executing Action: ' + self.name_
        # self.node_status_ = 'SUCCESS'
        self.node_status_ = 'RUNNING'
        return self.node_status_

        
class NodeService(Node):

    def __init__(self,parent,name='unnamed'):
        super(NodeService,self).__init__(False,parent,name)

    def get_node_type(self):
        return 'SERVICE'

    def get_node_name(self):
        return 'Service'

    def execute(self):
        pass

class NodeParamCondition(Node):

    def __init__(self,parent,name='unnamed',param_name=None,desired_value=None):
        super(NodeParamCondition,self).__init__(False,parent,name)
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

    def __init__(self,parent,name='unnamed',runs=1):
        super(NodeDecoratorRunNumber,self).__init__(False,parent,name)
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



# MAIN #########################################################################
if __name__ == '__main__':
    root = NodeRoot()
    sec1 = NodeSequence(root,'sequence_1')
    a1 = NodeAction(sec1,'a1')
    a2 = NodeAction(sec1,'a2')
    a3 = NodeAction(sec1,'a3')
    a4 = NodeAction(sec1,'a4')

    # root.print_subtree()
    # sec2 = NodeSequence(sec1)
    # a2 = NodeAction(sec2,'a2')
    # a3 = NodeAction(sec2,'a3')
    # a4 = NodeAction(sec2,'a4')
    print root.execute()


        




























