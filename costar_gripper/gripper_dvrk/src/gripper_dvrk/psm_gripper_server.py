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
import dvrk

class SimplePSMGripperServer(CostarGripper):

	def __init__(self,ns="/costar/gripper",verbose=False):

		self.dvrk_arm = dvrk.psm('PSM2')
		self.psm_initialized = False

		super(SimplePSMGripperServer, self).__init__(
				"psm_gripper",
				input_topic="PSMGripperInput",
				output_topic="PSMGripperOutput",
				InputMsgType=None,
				OutputMsgType=None,
				GripperPredicatorType=None,
				ns=ns,
				verbose=verbose)

	def init_gripper(self):
		if self.psm_initialized != True:
			self.dvrk_arm.home()
			self.dvrk_arm.insert_tool(0.1)
			self.psm_initialized = True
			print "PSM Initialized"
			return []

	def open_gripper(self,msg=None):
		self.dvrk_arm.open_jaw()
		return []

	def close_gripper(self,msg=None):
		self.dvrk_arm.close_jaw()
		return []

	def getDefaultMsg(self):
		return []
	
	def activate(self,msg=None):
		return []
	
	def reset(self, msg=None):
		return []
	
	def statusInterpreter(self,status):
		return []
