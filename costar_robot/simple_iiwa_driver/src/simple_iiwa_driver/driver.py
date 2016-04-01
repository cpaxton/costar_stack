
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

from moveit_msgs.msg import *
from moveit_msgs.srv import *

from predicator_landmark import GetWaypointsService

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm'}

class SimpleIIWADriver:

    def __init__(self,world="/world"):
        base_link = 'iiwa_link_0'
        end_link = 'iiwa_link_ee'

        self.world = world
        self.base_link = base_link
        self.end_link = end_link

        self.q0 = [0,0,0,0,0,0,0]

        self.teach_mode = rospy.Service('costar/SetTeachMode',SetTeachMode,self.set_teach_mode_call)
        self.servo_mode = rospy.Service('costar/SetServoMode',SetServoMode,self.set_servo_mode_call)
        self.shutdown = rospy.Service('costar/ShutdownArm',EmptyService,self.shutdown_arm_call)
        self.servo = rospy.Service('costar/ServoToPose',ServoToPose,self.servo_to_pose_call)
        self.plan = rospy.Service('costar/PlanToPose',ServoToPose,self.plan_to_pose_call)
        self.smartmove = rospy.Service('costar/SmartMove',SmartMove,self.smart_move_call)
        self.get_waypoints_srv = GetWaypointsService(world=world,service=False)
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
        self.display_pub = rospy.Publisher('costar/display_trajectory',DisplayTrajectory,queue_size=1000)

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


    def check_req_speed_params(self,req):
        if req.accel > self.MAX_ACC:
            acceleration = self.MAX_ACC
        else:
            acceleration = req.accel
        if req.vel > self.MAX_VEL:
            velocity = self.MAX_VEL
        else:
            velocity = req.vel
        return (acceleration, velocity)

    def smart_move_call(self,req):


        if not self.driver_status == 'SERVO':
            rospy.logerr('DRIVER -- IK fail/not in servo mode')
            return 'FAILED - not in servo mode'

        (acceleration, velocity) = self.check_req_speed_params(req) 
        (poses,names) = self.get_waypoints_srv.get_waypoints(
                req.obj_class, # object class to move to
                req.predicates, # predicates to match
                [req.pose], # offset/transform from each member of the class
                ["tmp"] # placeholder name
                )
        if not poses is None:
            for (pose,name) in zip(poses,names):
                # try to move to the pose until one succeeds
                T_base_world = tf_c.fromTf(self.listener_.lookupTransform(self.world,self.base_link,rospy.Time(0)))
                T = tf_c.toMatrix(T_base_world.Inverse()*tf_c.fromMsg(req.target))

                # Check acceleration and velocity limits
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
                            msg = "FAILED - timeout"
                        rate.sleep()

                    msg = 'SUCCESS - moved to pose'
                    break
                else:
                    rospy.logwarn('SIMPLE DRIVER -- IK failed for %s'%name)
                    msg = 'FAILED - inverse kinematics failed for %s'%name
            return msg


    def plan_to_pose_call(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':
            T = tf_c.toMatrix(tf_c.fromMsg(req.target))
            #a,axis = T.M.GetRotAngle()
            #pose = list(T.p) + [a*axis[0],a*axis[1],a*axis[2]]

            #print T

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 

            # Send command
            pt = JointTrajectoryPoint()
            q = self.kdl_kin.inverse(T,self.q0)

            if q is None:
                q = self.kdl_kin.inverse(T,None)
            
            (code,res) = self.planner.getPlan(req.target,self.q0)

            #pt.positions = q
            if code > 0:
              self.at_goal = True
              return 'SUCCESS - at goal'
            if not res is None and len(res.planned_trajectory.joint_trajectory.points) > 0:


                disp = DisplayTrajectory()
                disp.trajectory.append(res.planned_trajectory)
                disp.trajectory_start = res.trajectory_start
                self.display_pub.publish(disp)

                traj = res.planned_trajectory.joint_trajectory

                t = rospy.Time(0)

                for pt in traj.points:
                  self.pt_publisher.publish(pt)

                  rospy.sleep(rospy.Duration(pt.time_from_start.to_sec() - t.to_sec()))
                  t = pt.time_from_start

                pt = traj.points[-1]
                self.at_goal = False
                self.goal = pt.positions
                rate = rospy.Rate(10)
                start_t = rospy.Time.now()

                # wait until robot is at goal
                while not self.at_goal:
                    if (rospy.Time.now() - start_t).to_sec() > 30:
                        return 'FAILED - timeout'
                    rate.sleep()

                return 'SUCCESS - moved to pose'
            else:
                rospy.logerr('DRIVER -- PLANNING failed')
                return 'FAILED - not in servo mode'
        else:
            rospy.logerr('DRIVER -- IK fail/not in servo mode')
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
            (acceleration, velocity) = self.check_req_speed_params(req) 

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
                rospy.logerr('SIMPLE DRIVER -- IK failed')
                return 'FAILED - not in servo mode'
        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
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
