
import tf
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
import tf_conversions.posemath as pm
import numpy as np

from dvrk import psm

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

class CostarPSMDriver(CostarArm):

    def __init__(self,
            name="psm1",
            world="/world",
            listener=None,
            traj_step_t=0.1,
            max_acc=1,
            max_vel=1,
            max_goal_diff = 0.02,
            goal_rotation_weight = 0.01,
            max_q_diff = 1e-6):

        # TODO: correct these
        base_link = 'PSM1_psm_base_link'
        end_link = 'PSM1_tool_tip_link'
        planning_group = 'manipulator'

        self.dvrk_arm = psm('psm1')

        super(CostarPSMDriver, self).__init__(base_link,end_link,planning_group, dof=6)

    def home(self, req):
        if self.dvrk_arm.home():
            return 'SUCCESS'
        else:
            return 'FAILURE'

    def insert_tool(self,req):
        if self.dvrk_arm.insert_tool(0.1):
            return 'SUCCESS'
        else:
            return 'FAILURE'

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5,cartesian=False, linear=False):
        traj_way_point = PyKDL.Vector(traj.points[0].positions[0],traj.points[0].positions[1],traj.points[0].positions[2])

        print "waypoint [0]: " + str(traj_way_point)
        self.dvrk_arm.move(traj_way_point)

        # rate = rospy.Rate(30)
        # t = rospy.Time(0)
        #
        # stamp = rospy.Time.now().to_sec()
        # self.cur_stamp = stamp
        #
        # for pt in traj.points[:-1]:
        #   self.pt_publisher.publish(pt)
        #   self.set_goal(pt.positions)
        #
        #   print " -- %s"%(str(pt.positions))
        #   start_t = rospy.Time.now()
        #
        #   if self.cur_stamp > stamp:
        #     return 'FAILURE - preempted'
        #
        #   rospy.sleep(rospy.Duration(pt.time_from_start.to_sec() - t.to_sec()))
        #   t = pt.time_from_start
        #
        #
        # print " -- GOAL: %s"%(str(traj.points[-1].positions))
        # self.pt_publisher.publish(traj.points[-1])
        # self.set_goal(traj.points[-1].positions)
        # start_t = rospy.Time.now()
        #
        # # wait until robot is at goal
        # #while self.moving:
        # while not self.at_goal:
        #     if (rospy.Time.now() - start_t).to_sec() > 3:
        #         return 'FAILURE - timeout'
        #     rate.sleep()
        #
        # if self.at_goal:
        #     return 'SUCCESS - moved to pose'
        # else:
        #     return 'FAILURE - did not reach destination'

    # def handle_tick(self):
    #   rospy.logwarn('Function "handle_tick" not yet implemented for PSM!')
    #   # TODO handle control mode
    #   if self.driver_status == 'TEACH':
    #     pass
    #   else:
    #     pass

