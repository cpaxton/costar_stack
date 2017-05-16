# Chris Paxton -- 2016
# Based on the Robotiq code, but modified to provide a simple service interface for use with our user interface 
# Portions of this code (c) Robotiq, Inc.:

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

from gripper_manager import CostarGripper

import rospy
from robotiq_c_model_control.msg import _CModel_robot_output  as outputMsg
from robotiq_c_model_control.msg import _CModel_robot_input  as inputMsg
from predicator_robotiq import CModelPredicator

class SimpleCModelServer(CostarGripper):

    def __init__(self,ns="/costar/gripper",verbose=False):
        super(SimpleCModelServer, self).__init__(
                "c_model",
                input_topic="CModelRobotInput",
                output_topic="CModelRobotOutput",
                InputMsgType=inputMsg.CModel_robot_input,
                OutputMsgType=outputMsg.CModel_robot_output,
                GripperPredicatorType=CModelPredicator,
                ns=ns,
                verbose=verbose)

    def getDefaultMsg(cls):
        command = outputMsg.CModel_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP = 255
        command.rFR = 150
        return command

    def activate(self,msg=None):
        self.command = self.getDefaultMsg()
        self.pub.publish(self.command)
        return []

    def reset(self, msg=None):
        self.command = outputMsg.CModel_robot_output();
        self.command.rACT = 0
        self.pub.publish(self.command)
        return []

    def open_gripper(self,msg=None):
        self.command.rPR = 0
        self.pub.publish(self.command)
        rospy.sleep(0.2)
        while self.predicator.moving: #self.predicator.closed
          rospy.sleep(0.01)
        rospy.loginfo("Done open")
        return []

    def close_gripper(self,msg=None):
        self.command.rPR = 255
        self.pub.publish(self.command)
        rospy.sleep(0.2)
        while self.predicator.moving: #not self.predicator.closed
          rospy.sleep(0.01)
        rospy.loginfo("Done close")
        return []

    def wide_mode(self,msg=None):
        return []

    def pinch_mode(self,msg=None):
        return []

    def basic_mode(self,msg=None):
        return []

    def scissor_mode(self,msg=None):
        return []

    def statusInterpreter(self,status):
        """Generate a string according to the current value of the status variables."""

        output = 'C-Model status:\n'

        #gACT
        output += 'gACT = ' + str(status.gACT) + ': '
        if(status.gACT == 0):
            output += 'Gripper reset\n'
        if(status.gACT == 1):
            output += 'Gripper activation\n'

        #gSTA
        output += 'gSTA = ' + str(status.gSTA) + ': '
        if(status.gSTA == 0):
            output += 'Gripper is in reset ( or automatic release ) state. see Fault Status if Gripper is activated\n'
        if(status.gSTA == 1):
            output += 'Activation in progress\n'
        if(status.gSTA == 2):
            output += 'Not used\n'
        if(status.gSTA == 3):
            output += 'Activation is completed\n'

        #gOBJ
        output += 'gOBJ = ' + str(status.gOBJ) + ': '
        if(status.gOBJ == 0):
            output += 'Fingers are in motion (only meaningful if gGTO = 1)\n'
        if(status.gOBJ == 1):
            output += 'Fingers have stopped due to a contact while opening\n'
        if(status.gOBJ == 2):
            output += 'Fingers have stopped due to a contact while closing \n'
        if(status.gOBJ == 3):
            output += 'Fingers are at requested position\n'
     
        #gFLT
        output += 'gFLT = ' + str(status.gFLT) + ': '
        if(status.gFLT == 0x00):
            output += 'No Fault\n'
        if(status.gFLT == 0x05):
            output += 'Priority Fault: Action delayed, initialization must be completed prior to action\n'
        if(status.gFLT == 0x07):
            output += 'Priority Fault: The activation bit must be set prior to action\n'
        if(status.gFLT == 0x09):
            output += 'Minor Fault: The communication chip is not ready (may be booting)\n'   
        if(status.gFLT == 0x0B):
            output += 'Minor Fault: Automatic release in progress\n'
        if(status.gFLT == 0x0E):
            output += 'Major Fault: Overcurrent protection triggered\n'
        if(status.gFLT == 0x0F):
            output += 'Major Fault: Automatic release completed\n'
            
        return output

