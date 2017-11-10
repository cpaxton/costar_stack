# predicator (c) 2014-2016, Chris Paxton
#
# based on some code taken from Robotiq's c_model_control package

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

import rospy
from robotiq_c_model_control.msg import CModel_robot_input as inputMsg
from predicator_msgs.msg import *

class CModelPredicator:

    def __init__(self,publish_predicates=True,start_subscriber=True,gripper_name='c_model'):

        self.valid_predicates = ValidPredicates()
        self.valid_predicates.assignments = assignments=[gripper_name]
        self.valid_predicates.predicates = ['gripper_open','gripper_closed','gripper_moving',
                    'gripper_basic_mode','gripper_pinch_mode','gripper_wide_mode',
                    'gripper_scissor_mode','gripper_activated',
                    'finger_a_contact','finger_b_contact','finger_c_contact',
                    'any_finger_contact']
        self.valid_predicates.pheader.source = rospy.get_name()

        self.predicate_msg = PredicateList()
        self.gripper_name = gripper_name

        self.publish_predicates = publish_predicates
        if self.publish_predicates:
            # create predicator things
            self.pub = rospy.Publisher("predicator/input",PredicateList,queue_size=1000)
            self.vpub = rospy.Publisher("predicator/valid_input",ValidPredicates,queue_size=1000)

        if start_subscriber:
            self.sub = rospy.Subscriber("CModelRobotInput",inputMsg,self.callback)

        self.name = rospy.get_name()

        self.gripper_mode = 'pinch_mode'
        self.activated = False
        self.contact = False
        self.closed = False
        self.moving = False

    def callback(self, msg):
        self.handle(msg)

    def handle(self,status):
        self.predicate_msg = PredicateList()
        self.predicate_msg.pheader.source = self.name
        if(status.gACT == 0):
            # gripper reset
            pass
        if(status.gACT == 1):
            self.addPredicate('gripper_activated')
            self.activated = False
        else:
            self.activated = True

        # this gripper is always in pinch mode
        self.addPredicate('gripper_pinch_mode')
        self.gripper_mode = 'pinch'

        #gGTO
        if(status.gGTO == 0):
            self.moving = False
            #output += 'Standby (or performing activation/automatic release)\n'
        if(status.gGTO == 1):
            self.moving = True
            #output += 'Go to Position Request\n'

        #gOBJ
        self.contact = False
        if(status.gOBJ == 0):
            self.moving = True
            #output += 'Fingers are in motion (only meaningful if gGTO = 1)\n'
        if(status.gOBJ == 1):
            self.contact = True
            self.moving = False
            #output += 'Fingers have stopped due to a contact while opening\n'
        if(status.gOBJ == 2):
            self.contact = True
            self.moving = False
            #output += 'Fingers have stopped due to a contact while closing \n'
        if(status.gOBJ == 3):
            self.contact = False
            self.moving = False
            #output += 'Fingers are at requested position\n'

        if self.contact:
            self.addPredicate('any_finger_contact')
        if self.moving:
            self.addPredicate('gripper_moving')

        if status.gFLT > 5:
          rospy.logerr("Fault!")
          self.moving = False

        pos = status.gPO
        #print "position = %s"%str(pos)
        if pos > 240 and not self.moving:
          self.closed = True
        else:
          self.closed = False

    '''
    add a single message
    '''
    def addPredicate(self,predicate):
        p = PredicateStatement(predicate=predicate,params=[self.gripper_name,'',''])
        self.predicate_msg.statements.append(p)

    '''
    publish current predicate messages
    '''
    def tick(self):
        if self.publish_predicates:
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
            
