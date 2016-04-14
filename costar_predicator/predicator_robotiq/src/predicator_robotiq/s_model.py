# predicator (c) 2014-2016, Chris Paxton
#
# some code taken from Robotiq's s_model_control package:

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
from robotiq_s_model_control.msg import _SModel_robot_ouput as outputMsg
from robotiq_s_model_control.msg import _SModel_robot_input as inputMsg
from predicator_msgs.msg import *

class SModelPredicator:

    def __init__(self, independent_node=True,gripper_name='s_model'):

        self.valid_predicates = ValidPredicates()
        self.predicate_msg = PredicateList(assignments=[gripper_name],predicates=['gripper_open','gripper_closed','gripper_moving',
            'gripper_basic_mode','gripper_pinch_mode','gripper_wide_mode','gripper_scissor_mode','gripper_activated',
            'finger_a_contact','finger_b_contact','finger_c_contact'])
        

        if independent_node:
            # create predicator things
            self.sub = rospy.Subscriber("SModelRobotInput",inputMsg,self.callback)
            self.pub = rospy.Publisher("predicator/input",PredicateList,queue_size=1000)
            self.vpub = rospy.Publisher("predicator/valid_predicates",PredicateList,queue_size=1000)

    def callback(self, msg):
        self.handle(msg)

    '''
    publish current predicate messages
    '''
    def tick(self):
        pass

    '''
    update and spin
    '''
    def spin(self,rate=10):
        spin_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.tick()
            spin_rate.sleep()
            
