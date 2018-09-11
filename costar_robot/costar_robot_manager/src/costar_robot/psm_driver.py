
import tf
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import *
import tf_conversions.posemath as pm
import numpy as np
from datetime import datetime 

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

from instructor_core.srv import AddWaypoint

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
        end_link = 'PSM1_tool_wrist_sca_ee_link_0'
        planning_group = 'manipulator'

        self.dvrk_arm = dvrk.psm('PSM1')
        self.psm_initialized = False
        self.record_waypoint = 1

        rospy.Subscriber("/instructor_marker/feedback", InteractiveMarkerFeedback, self.marker_cbback)
        self.last_marker_frame = PyKDL.Frame(PyKDL.Rotation.RPY(0,0,0),PyKDL.Vector(0,0,0))
        self.last_marker_trans = (0,0,0)
        self.last_marker_rot = (0,0,0,1)

        rospy.Subscriber("/dvrk/footpedals/clutch",Joy,self.clutch_cb)
        rospy.wait_for_service("/instructor_core/AddWaypoint",5)
        self.add_waypoint_service = rospy.ServiceProxy('/instructor_core/AddWaypoint', AddWaypoint)

        self.tf_listener = tf.TransformListener()
        self.cur_pose = Pose()

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

    def marker_cbback(self,data):
        (self.last_marker_trans,self.last_marker_rot) = pm.toTf(pm.fromMsg(data.pose))

    def clutch_cb(self,data):
        if self.record_waypoint == data.buttons[0] and self.driver_status == 'TEACH':
            # ((self.cur_pose.position.x,self.cur_pose.position.y,self.cur_pose.position.z), 
            #     (self.cur_pose.orientation.x,self.cur_pose.orientation.y,self.cur_pose.orientation.z,self.cur_pose.orientation.w)) \
            # = self.tf_listener.lookupTransform('/PSM1_tool_wrist_sca_ee_link_0','/world',rospy.Time(0))
            (pos,rot) = self.tf_listener.lookupTransform('/world','/PSM1_tool_wrist_sca_ee_link_0',rospy.Time(0))
            self.cur_pose = pm.toMsg(pm.fromTf((pos,rot)))
            self.add_waypoint_service(name = str(datetime.now()), world_pose = self.cur_pose, relative_pose = self.cur_pose, relative_frame_name = '')
            print "Clutch pressed, add 1 waypoint"

    def handle_tick(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "/base_link",
                         self.base_link)
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-1.5708, 1.5708, 0), rospy.Time.now(),
                         "/PSM1_tool_tip_link_virtual", 'PSM1_tool_wrist_sca_ee_link_0')
        # The above: add tip link with the orientation offset to represent the frame of our concern
        if self.driver_status == 'SHUTDOWN':
            pass
        elif self.driver_status == 'SERVO':
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),
                             "/endpoint", self.end_link)
            # print "HANDLING SERVO MODE"
        elif self.driver_status == 'IDLE':
            br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),
                             "/endpoint",self.end_link)
            # print "DRIVER IN IDLE"
        elif self.driver_status == 'TEACH':
            # print "HANDLING TEACH MODE"
            br.sendTransform(self.last_marker_trans, self.last_marker_rot, rospy.Time.now(), "/endpoint", "/world")
            # print "<<<<<", self.last_marker_trans, self.last_marker_rot, ">>>>>"


    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5,cartesian=False, linear=False):
        if self.psm_initialized != True:
            self.dvrk_arm.home()
            self.dvrk_arm.insert_tool(0.1)
            self.psm_initialized = True
            print "PSM Initialized"
            return

    def js_cb(self,msg):
        pass
        # # if len(msg.position) is self.dof:
        #     self.old_q0 = self.q0
        #     self.q0 = np.array(msg.position[0:6])
        # # else:
        #     # rospy.logwarn('Incorrect joint dimensionality')
        #     print(msg.position[0:6])

    def set_servo_mode_cb(self,req):
        if req.mode == 'SERVO':
            if self.q0 is not None:
                self.send_q(self.q0,0.1,0.1)

            self.driver_status = 'SERVO'
            self.cur_stamp = self.release()
            return 'SUCCESS - servo mode enabled'
        elif req.mode == 'DISABLE':
            # self.detach(actuate = False, add_back_to_planning_scene=False)
            self.cur_stamp = self.acquire()
            self.driver_status = 'IDLE'
            return 'SUCCESS - servo mode disabled'
        else:
            return 'FAILURE'

    def save_frame_cb(self,req):
      rospy.logwarn('Save frame does not check to see if your frame already exists!')
      print "save_frame_cb is called, and ee_pose is:", self.ee_pose
      self.waypoint_manager.save_frame(self.ee_pose, self.world)

      return 'SUCCESS - '

    def servo_to_pose_cb(self,req):
        if self.driver_status == 'SERVO':
            print(req.target)
            T_temp = pm.fromMsg(req.target)
            #frame_offset = PyKDL.Frame(PyKDL.Rotation.RPY(-1.569,1.571,-1.395), PyKDL.Vector(0,0,0))
            # frame_offset = PyKDL.Frame(PyKDL.Rotation.RPY(-1.569,1.571,-1.219), PyKDL.Vector(0,0,0))
            frame_offset = PyKDL.Frame(PyKDL.Rotation.RPY(-1.569,1.571,-1.435), PyKDL.Vector(0,0,0))
            T = T_temp * frame_offset
            self.dvrk_arm.move(T)
            return 'SUCCESS - moved to pose'

        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE - not in servo mode'

