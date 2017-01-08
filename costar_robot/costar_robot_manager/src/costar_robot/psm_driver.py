
import tf
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
import tf_conversions.posemath as pm
import numpy as np

# from dvrk import psm
import dvrk
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

        self.dvrk_arm = dvrk.psm('PSM1')
        # dvrk_arm.home()
	# dvrk_arm.insert_tool()

        super(CostarPSMDriver, self).__init__(base_link,end_link,planning_group, dof=6)

    def home(self):
        print "home function"
        self.dvrk_arm.home()
        if self.dvrk_arm.home():
            return 'SUCCESS'
        else:
            return 'FAILURE'

    def insert_tool(self):
        print "insert tool function"
        self.dvrk_arm.insert_tool(0.1)
        if self.dvrk_arm.insert_tool(0.1):
            return 'SUCCESS'
        else:
            return 'FAILURE'

    # def handle_tick(self):
    #   raise NotImplementedError('your code here')
    #   if self.driver_status == 'SHUTDOWN':
    #     if not self.simulation: # this may not be correct
    #       # close connection to robot
    #       pass
    # pass
    #   elif self.driver_status == 'SERVO':
    # pass
    #   elif self.driver_status == 'IDLE':
    # pass
    #   elif self.driver_status == 'TEACH':
    # pass
    #     # use interactive marker to publish robot /endpoint
    #     # then we can use instructor to save these positions for the robot
    #     print "HANDLING TEACH MODE"

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5,cartesian=False, linear=False):
	pass
    #    traj_way_point = PyKDL.Vector(traj.points[0].positions[0],traj.points[0].positions[1],traj.points[0].positions[2])
    #
    #    print "waypoint [0]: " + str(traj_way_point)
    #    self.dvrk_arm.move(traj_way_point)
