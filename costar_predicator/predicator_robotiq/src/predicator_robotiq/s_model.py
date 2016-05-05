# predicator (c) 2014-2016, Chris Paxton
#
# based on some code taken from Robotiq's s_model_control package:

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
from robotiq_s_model_control.msg import SModel_robot_input  as inputMsg
from predicator_msgs.msg import *

class SModelPredicator:

    def __init__(self,publish_predicates=True,start_subscriber=True,gripper_name='s_model'):

        self.valid_predicates = ValidPredicates(assignments=[gripper_name],predicates=['gripper_open','gripper_closed','gripper_moving',
            'gripper_basic_mode','gripper_pinch_mode','gripper_wide_mode','gripper_scissor_mode','gripper_activated',
            'finger_a_contact','finger_b_contact','finger_c_contact','any_finger_contact'])
        self.predicate_msg = PredicateList()
        self.gripper_name = gripper_name

        self.gripper_mode = ''
        self.activated = False
        self.contact = False
        self.closed = False
        self.moving = False

        if publish_predicates:
            # create predicator things
            self.pub = rospy.Publisher("predicator/input",PredicateList,queue_size=1000)
            self.vpub = rospy.Publisher("predicator/valid_predicates",PredicateList,queue_size=1000)

        if start_subscriber:
            self.sub = rospy.Subscriber("SModelRobotInput",inputMsg,self.callback)

        self.name = rospy.get_name()

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

        if(status.gMOD == 0):
            self.addPredicate('gripper_basic_mode')
            self.gripper_mode = 'basic'
        elif(status.gMOD == 1):
            self.addPredicate('gripper_pinch_mode')
            self.gripper_mode = 'pinch'
        elif(status.gMOD == 2):
            self.addPredicate('gripper_wide_mode')
            self.gripper_mode = 'wide'
        elif(status.gMOD == 3):
            self.addPredicate('gripper_scissor_mode')
            self.gripper_mode = 'scissor'

        if ((status.gGTO == 1) # going to position (GOTO command)
            or (status.gIMC == 2) # mode change in progress
            or (status.gSTA == 0) # in motion towards position
            ):

            self.addPredicate('gripper_moving')
            self.moving = True
        else:
            self.moving = False
        
        contact = False
        if (status.gDTA == 1 or status.gDTA == 2):
            self.addPredicate('finger_a_contact')
            contact = True
        if (status.gDTB == 1 or status.gDTB == 2):
            self.addPredicate('finger_b_contact')
            contact = True
        if (status.gDTC == 1 or status.gDTC == 2):
            self.addPredicate('finger_c_contact')
            contact = True 

        self.contact = contact
        if contact:
            self.addPredicate('any_finger_contact')

        if ((status.gDTA >= 2 and status.gDTB >= 2 and status.gDTC >= 2 and status.gPRA >= 250) # fingers closed or stopped closing
            or (status.gDTS >=2 and status.gPRA >= 250) # scissor closing
            ):

            self.addPredicate('gripper_closed')
            self.closed = True
        else:
            self.closed = False

    '''
    add a single message
    '''
    def addPredicate(self,predicate):
        p = PredicateStatement(predicate=predicate,params=[self.gripper_name,'',''])
        self.predicate_msg.predicates.append(p)

    '''
    publish current predicate messages
    '''
    def tick(self):
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
            
