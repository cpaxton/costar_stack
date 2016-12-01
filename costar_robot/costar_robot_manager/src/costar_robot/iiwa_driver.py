
import tf
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
import tf_conversions.posemath as pm
import numpy as np

import PyKDL
import urdf_parser_py
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from costar_robot import SimplePlanning
from costar_robot import CostarArm

from moveit_msgs.msg import *
from moveit_msgs.srv import *

from predicator_landmark import GetWaypointsService

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm', 'IDLE':'PauseArm'}

class CostarIIWADriver(CostarArm):

    def __init__(self,world="/world",
            listener=None,
            traj_step_t=0.1,
            max_acc=1,
            max_vel=1,
            max_goal_diff = 0.02,
            goal_rotation_weight = 0.01,
            max_q_diff = 1e-6):

        base_link = 'iiwa_link_0'
        end_link = 'iiwa_link_ee'
        planning_group = 'manipulator'

        super(CostarIIWADriver, self).__init__(base_link,end_link,planning_group, dof=7)

        self.iiwa_mode_publisher = rospy.Publisher('/interaction_mode',String,queue_size=1000)

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5,cartesian=False, linear=False):

        rate = rospy.Rate(30)
        t = rospy.Time(0)

        stamp = rospy.Time.now().to_sec()
        self.cur_stamp = stamp

        for pt in traj.points[:-1]:
          self.pt_publisher.publish(pt)
          self.set_goal(pt.positions)

          print " -- %s"%(str(pt.positions))
          start_t = rospy.Time.now()

          if self.cur_stamp > stamp:
            return 'FAILURE - preempted'

          rospy.sleep(rospy.Duration(pt.time_from_start.to_sec() - t.to_sec()))
          t = pt.time_from_start

          #while not self.near_goal:
          #  if (rospy.Time.now() - start_t).to_sec() > 10*t.to_sec():
          #      break
          #  rate.sleep()

        print " -- GOAL: %s"%(str(traj.points[-1].positions))
        self.pt_publisher.publish(traj.points[-1])
        self.set_goal(traj.points[-1].positions)
        start_t = rospy.Time.now()

        # wait until robot is at goal
        #while self.moving:
        while not self.at_goal:
            if (rospy.Time.now() - start_t).to_sec() > 3:
                return 'FAILURE - timeout'
            rate.sleep()

        if self.at_goal:
            return 'SUCCESS - moved to pose'
        else:
            return 'FAILURE - did not reach destination'

    def handle_tick(self):
        if self.driver_status in mode.keys():
            self.iiwa_mode_publisher.publish(mode[self.driver_status])
        else:
            #rospy.logwarn('IIWA mode for %s not specified!'%self.driver_status)
            pass

