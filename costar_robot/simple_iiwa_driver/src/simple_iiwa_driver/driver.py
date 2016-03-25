
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
import tf_conversions as tf_c
import numpy as np

import PyKDL
import urdf_parser_py
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from simple_planning import SimplePlanning

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm'}

class SimpleIIWADriver:

    def __init__(self):
        base_link = 'iiwa_link_0'
        end_link = 'iiwa_link_ee'
        self.q0 = [0,0,0,0,0,0,0]
        self.teach_mode = rospy.Service('costar/SetTeachMode',SetTeachMode,self.set_teach_mode_call)
        self.servo_mode = rospy.Service('costar/SetServoMode',SetServoMode,self.set_servo_mode_call)
        self.shutdown = rospy.Service('costar/ShutdownArm',EmptyService,self.shutdown_arm_call)
        self.servo = rospy.Service('costar/ServoToPose',ServoToPose,self.servo_to_pose_call)
        self.servo = rospy.Service('costar/PlanToPose',ServoToPose,self.plan_to_pose_call)
        self.driver_status = 'IDLE'
        self.status_publisher = rospy.Publisher('costar/DriverStatus',String,queue_size=1000)
        self.iiwa_mode_publisher = rospy.Publisher('/interaction_mode',String,queue_size=1000)
        self.pt_publisher = rospy.Publisher('/joint_traj_pt_cmd',JointTrajectoryPoint,queue_size=1000)
        self.robot = URDF.from_parameter_server()
        self.js_subscriber = rospy.Subscriber('/joint_states',JointState,self.js_cb)
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain(base_link, end_link)
        #print self.tree.getNrOfSegments()
        #print self.chain.getNrOfJoints()
        self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)

        self.planner = SimplePlanning(base_link,end_link,"manipulator")

        self.MAX_ACC = 1;
        self.MAX_VEL = 1;

        self.at_goal = True
        self.goal = np.array([0,0,0,0,0,0,0])

    def js_cb(self,msg):
        self.q0 = np.array(msg.position)
        goal_diff = np.abs(self.goal - self.q0).sum() / self.q0.shape[0]
        #print goal_diff
        #if self.at_goal:
        #    self.goal = self.q0
        if goal_diff < 0.001:
                self.at_goal = True
        #print self.q0

    def plan_to_pose_call(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':
            T = tf_c.toMatrix(tf_c.fromMsg(req.target))
            #a,axis = T.M.GetRotAngle()
            #pose = list(T.p) + [a*axis[0],a*axis[1],a*axis[2]]

            #print T

            # Check acceleration and velocity limits
            if req.accel > self.MAX_ACC:
                acceleration = self.MAX_ACC
            else:
                acceleration = req.accel
            if req.vel > self.MAX_VEL:
                velocity = self.MAX_VEL
            else:
                velocity = req.vel
            # Send command

            pt = JointTrajectoryPoint()
            q = self.kdl_kin.inverse(T,self.q0)

            if q is None:
                q = self.kdl_kin.inverse(T,None)
            
            print self.q0
            print q

            self.planner.getPlan(req.target,self.q0)

            pt.positions = q

            if not q is None:
                self.pt_publisher.publish(pt)
                self.at_goal = False
                self.goal = q
                rate = rospy.Rate(10)
                start_t = rospy.Time.now()

                # wait until robot is at goal
                while not self.at_goal:
                    if (start_t - rospy.Time.now()).to_sec() > 30:
                        return 'FAILED - timeout'
                    rate.sleep()

                return 'SUCCESS - moved to pose'
            else:
                rospy.logerr('SIMPLE UR -- IK failed')
                return 'FAILED - not in servo mode'
        else:
            rospy.logerr('SIMPLE UR -- Not in servo mode')
            return 'FAILED - not in servo mode'

    def servo_to_pose_call(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':
            T = tf_c.toMatrix(tf_c.fromMsg(req.target))
            #a,axis = T.M.GetRotAngle()
            #pose = list(T.p) + [a*axis[0],a*axis[1],a*axis[2]]

            #print T

            # Check acceleration and velocity limits
            if req.accel > self.MAX_ACC:
                acceleration = self.MAX_ACC
            else:
                acceleration = req.accel
            if req.vel > self.MAX_VEL:
                velocity = self.MAX_VEL
            else:
                velocity = req.vel
            # Send command

            pt = JointTrajectoryPoint()
            q = self.kdl_kin.inverse(T,self.q0)

            if q is None:
                q = self.kdl_kin.inverse(T,None)
            
            print self.q0
            print q

            pt.positions = q

            if not q is None:
                self.pt_publisher.publish(pt)
                self.at_goal = False
                self.goal = q
                rate = rospy.Rate(10)
                start_t = rospy.Time.now()

                # wait until robot is at goal
                while not self.at_goal:
                    if (start_t - rospy.Time.now()).to_sec() > 30:
                        return 'FAILED - timeout'
                    rate.sleep()

                return 'SUCCESS - moved to pose'
            else:
                rospy.logerr('SIMPLE UR -- IK failed')
                return 'FAILED - not in servo mode'
        else:
            rospy.logerr('SIMPLE UR -- Not in servo mode')
            return 'FAILED - not in servo mode'

    def set_teach_mode_call(self,req):
        if req.enable == True:
            # self.rob.set_freedrive(True)
            self.driver_status = 'TEACH'
            return 'SUCCESS - teach mode enabled'
        else:
            # self.rob.set_freedrive(False)
            self.driver_status = 'IDLE'
            return 'SUCCESS - teach mode disabled'

    def set_servo_mode_call(self,req):
        if req.mode == 'SERVO':
            self.driver_status = 'SERVO'
            return 'SUCCESS - servo mode enabled'
        elif req.mode == 'DISABLE':
            self.driver_status = 'IDLE'
            return 'SUCCESS - servo mode disabled'

    def shutdown_arm_call(self,req):
        self.driver_status = 'SHUTDOWN'
        pass

    def tick(self):
        self.status_publisher.publish(self.driver_status)
        if self.driver_status in mode.keys():
            self.iiwa_mode_publisher.publish(mode[self.driver_status])
        else:
            rospy.logwarn('IIWA mode for %s not specified!'%self.driver_status)
