#!/usr/bin/env python
import roslib; roslib.load_manifest('instructor_core')
import rospy
import rospkg

import tf; 
import tf_conversions as tf_c
from instructor_core.srv import *
from librarian_msgs.msg import *
from librarian_msgs.srv import *
import yaml
import numpy as np
import PyKDL
from copy import deepcopy

class InstructorARSmooth(object):
    def __init__(self):
        rospy.init_node('instructor_ar_smooth',anonymous=True)

        self.broadcaster_ = tf.TransformBroadcaster()
        self.listener_ = tf.TransformListener()
        self.buffer = 40
        self.frames = {}

        while not rospy.is_shutdown():
            self.update()
            rospy.sleep(.1)

    def update(self):

        all_frames = self.listener_.getFrameStrings()
        ar_frames = [f for f in all_frames if f.find('ar_marker')>=0]
        un_filtered = [f for f in ar_frames if f.find('filtered')<0]

        # Decay each frame count at every timestep
        for f in self.frames.keys():
            self.frames[f]['no_frames'] -= 1
            if self.frames[f]['no_frames'] <= 0:
                short_name = 'landmark_' + f.split('_')[len(f.split('_'))-1:][0]
                rospy.delete_param('instructor_landmark/'+short_name)
                rospy.logwarn('Deleted:' +short_name)
                self.frames.pop(f)

        for frame in un_filtered:
            try:
                F = tf_c.fromTf(self.listener_.lookupTransform('/world',frame,rospy.Time(0)))
            except (tf.LookupException, tf.ConnectivityException) as e:
                rospy.logerr('Frame ['+frame+'] not found')
                return
            except tf.ExtrapolationException as e:
                # The frame we're seeing is old
                return

            if frame not in self.frames.keys():
                self.frames[frame] = {'no_frames':0, 'poses':[], 'average_pose':None}
                rospy.logwarn('New Frame:' + frame)

            # rospy.logwarn(self.frames[frame]['no_frames'])
            if self.frames[frame]['no_frames'] < self.buffer:
                self.frames[frame]['no_frames'] += 2
                self.frames[frame]['poses'].append(F)
            else:
                self.frames[frame]['poses'].pop(0)
                self.frames[frame]['poses'].pop(0)
                self.frames[frame]['poses'].append(F)

        for frame in self.frames.keys():
            # Get all stored frame positions/rotations
            sum_xyz = [tf_c.toTf(f)[0] for f in self.frames[frame]['poses']]
            sum_rpy = [f.M.GetRPY() for f in self.frames[frame]['poses']]

            if len(sum_xyz) > 2:
                xyz = np.mean(sum_xyz, 0)
                rpy = np.mean(sum_rpy, 0)

                F_avg = PyKDL.Frame()
                F_avg.p = PyKDL.Vector(*xyz)
                F_avg.M = PyKDL.Rotation.RPY(*rpy)

                self.broadcaster_.sendTransform(tuple(F_avg.p),tuple(F_avg.M.GetQuaternion()),rospy.Time.now(), '/filtered/'+frame, '/world')
                # rosparam_marker_name = "instructor_landmark/{}".format(str('filtered/'+frame))
                # rospy.set_param(rosparam_marker_name, str('filtered/'+frame))
                short_name = 'landmark_' + frame.split('_')[len(frame.split('_'))-1:][0]
                rospy.set_param('instructor_landmark/'+short_name, str('filtered/'+frame))
                # rospy.logwarn(short_name)


### MAIN ########################################################
if __name__ == '__main__':
    I = InstructorARSmooth()
