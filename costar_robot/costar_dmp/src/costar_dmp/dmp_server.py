#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty as EmptyService

class CostarDMP:

	def __init__(self):
		self.collecting = False
		start_rec_srv = rospy.Service('costar/dmp/start_rec', EmptyService, self.start_rec_cb)
		stop_rec_srv = rospy.Service('costar/dmp/stop_rec', EmptyService, self.stop_rec_cb)

	def dmp_tick(self):
		if self.collecting == True:
			print "DMP tick() method called with recording."
		else:
			print "DMP tick() method called without recording. "

	def start_rec_cb(self,req):
		self.collecting = True
		return []

	def stop_rec_cb(self,req):
		self.collecting = False
		return []




