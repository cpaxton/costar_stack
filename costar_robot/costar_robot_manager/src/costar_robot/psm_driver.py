
import tf
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
from visualization_msgs.msg import *
import tf_conversions.posemath as pm
import numpy as np

# for interactively show instructor marker when switching to TEACH
import dynamic_reconfigure.client
from subprocess import call

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
        self.psm_initialized = False
        # dvrk_arm.home()
	    # dvrk_arm.insert_tool()

        rospy.Subscriber("/instructor_marker/feedback", InteractiveMarkerFeedback, self.marker_callback)
        self.last_marker_frame = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0),PyKDL.Vector(0,0,0))
        self.last_marker_trans = (0,0,0)
        self.last_marker_rot = (0,0,0,1)

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

    def marker_callback(self,data):
        (self.last_marker_trans,self.last_marker_rot) = pm.toTf(pm.fromMsg(data.pose))
        # self.last_marker_trans = data.pose.position
        # self.last_marker_rot = data.pose.orientation

    def handle_tick(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/base_link",
                         self.base_link)
        if self.driver_status == 'SHUTDOWN':
            pass
        elif self.driver_status == 'SERVO':
            print "HANDLING SERVO MODE"
            # call(["rosrun", "costar_robot_manager", "instructor_marker.py"])
        elif self.driver_status == 'IDLE':
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),
                             "/endpoint",self.end_link)
            print "DRIVER IN IDLE"
        elif self.driver_status == 'TEACH':
            print "HANDLING TEACH MODE"
            # br.sendTransform((self.last_marker_trans.x, self.last_marker_trans.y, self.last_marker_trans.z),
            #                  (self.last_marker_rot.x, self.last_marker_rot.y, self.last_marker_rot.z, self.last_marker_rot.w),
            #                  rospy.Time.now(),"/endpoint",self.end_link)
            br.sendTransform(self.last_marker_trans, self.last_marker_rot, rospy.Time.now(),"/endpoint",self.end_link)

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5,cartesian=False, linear=False):
        # pass
        if self.psm_initialized != True:
            self.dvrk_arm.home()
            self.dvrk_arm.insert_tool(0.1)
            self.psm_initialized = True
            print "PSM Initialized"
            return
        print "Msg from send_trajectory"


    #    traj_way_point = PyKDL.Vector(traj.points[0].positions[0],traj.points[0].positions[1],traj.points[0].positions[2])
    #
    #    print "waypoint [0]: " + str(traj_way_point)
    #    self.dvrk_arm.move(traj_way_point)
