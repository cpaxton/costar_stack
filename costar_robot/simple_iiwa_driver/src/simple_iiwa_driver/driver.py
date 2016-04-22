
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

from simple_planning import SimplePlanning

from moveit_msgs.msg import *
from moveit_msgs.srv import *

from predicator_landmark import GetWaypointsService

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm'}

class SimpleIIWADriver:

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
        self.planning_group = 'manipulator'

        self.world = world
        self.base_link = base_link
        self.end_link = end_link

        self.MAX_ACC = max_acc
        self.MAX_VEL = max_vel

        self.traj_step_t = traj_step_t

        self.max_goal_diff = max_goal_diff
        self.max_q_diff = max_q_diff
        self.goal_rotation_weight = goal_rotation_weight

        self.at_goal = True
        self.near_goal = True
        self.moving = False
        self.q0 = [0,0,0,0,0,0,0]
        self.old_q0 = [0,0,0,0,0,0,0]

        self.teach_mode = rospy.Service('/costar/SetTeachMode',SetTeachMode,self.set_teach_mode_call)
        self.servo_mode = rospy.Service('/costar/SetServoMode',SetServoMode,self.set_servo_mode_call)
        self.shutdown = rospy.Service('/costar/ShutdownArm',EmptyService,self.shutdown_arm_call)
        self.servo = rospy.Service('/costar/ServoToPose',ServoToPose,self.servo_to_pose_call)
        self.plan = rospy.Service('/costar/PlanToPose',ServoToPose,self.plan_to_pose_call)
        self.smartmove = rospy.Service('/costar/SmartMove',SmartMove,self.smart_move_call)
        self.get_waypoints_srv = GetWaypointsService(world=world,service=False)
        self.driver_status = 'IDLE'
        self.status_publisher = rospy.Publisher('/costar/DriverStatus',String,queue_size=1000)
        self.iiwa_mode_publisher = rospy.Publisher('/interaction_mode',String,queue_size=1000)
        self.pt_publisher = rospy.Publisher('/joint_traj_pt_cmd',JointTrajectoryPoint,queue_size=1000)
        self.robot = URDF.from_parameter_server()
        self.js_subscriber = rospy.Subscriber('joint_states',JointState,self.js_cb)
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain(base_link, end_link)

        if listener is None:
            self.listener = tf.TransformListener()
        else:
            self.listener = listener

        #print self.tree.getNrOfSegments()
        #print self.chain.getNrOfJoints()
        self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)
        self.display_pub = rospy.Publisher('costar/display_trajectory',DisplayTrajectory,queue_size=1000)

        self.set_goal(self.q0)
        self.ee_pose = None

        self.planner = SimplePlanning(self.robot,base_link,end_link,self.planning_group)

    def js_cb(self,msg):
        self.old_q0 = self.q0
        self.q0 = np.array(msg.position)
        self.ee_pose = pm.fromMatrix(self.kdl_kin.forward(self.q0))

        #goal_diff = np.abs(self.goal - self.q0).sum() / self.q0.shape[0]
        cart_diff = (self.ee_pose.p - self.goal.p).Norm()
        rot_diff = self.goal_rotation_weight * (pm.Vector(*self.ee_pose.M.GetRPY()) - pm.Vector(*self.goal.M.GetRPY())).Norm()
        goal_diff = cart_diff + rot_diff

        if goal_diff < self.max_goal_diff:
            self.at_goal = True

        if goal_diff < 10*self.max_goal_diff:
            self.near_goal = True

        q_diff = np.abs(self.old_q0 - self.q0).sum()

        if q_diff < self.max_q_diff:
            self.moving = False
        else:
            self.moving = True

        #print "moving=%s, at goal=%s, diff=%s"%(str(self.moving),str(self.at_goal),str(q_diff))


        #if not self.at_goal:
        #    print "%f + %f = %f"%(cart_diff,rot_diff,goal_diff)


        #print self.at_goal
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

    '''
    Find any valid object that meets the requirements.
    Find a cartesian path or possibly longer path through joint space.
    '''
    def smart_move_call(self,req):

        if False and not self.driver_status == 'SERVO':
            rospy.logerr('DRIVER -- Not in servo mode!')
            return 'FAILURE - not in servo mode'

        (acceleration, velocity) = self.check_req_speed_params(req) 
        (poses,names) = self.get_waypoints_srv.get_waypoints(
                req.obj_class, # object class to move to
                req.predicates, # predicates to match
                [req.pose], # offset/transform from each member of the class
                ["tmp"] # placeholder name
                )

        print req.obj_class
        print req.predicates
        print names
        print poses

        if not poses is None:
            msg = 'FAILURE - no valid objects found!'
            qs = []
            dists = []
            for (pose,name) in zip(poses,names):

                # figure out which tf frame we care about
                tf_path = name.split('/')
                tf_frame = ''
                for part in tf_path:
                    if len(part) > 0:
                        tf_frame = part
                        break

                if len(tf_frame) == 0:
                    continue

                # try to move to the pose until one succeeds
                T_base_world = pm.fromTf(self.listener.lookupTransform(self.world,self.base_link,rospy.Time(0)))
                T = T_base_world.Inverse()*pm.fromMsg(pose)

                # Check acceleration and velocity limits
                # Send command

                pt = JointTrajectoryPoint()

                #q = self.planner.ik(T, self.q0)
                traj = self.planner.getCartesianMove(T,self.q0)
                #if len(traj.points) == 0:
                #    print T
                #    (code,res) = self.planner.getPlan(pm.toMsg(T),self.q0) # find a non-local movement
                #    traj = res.planned_trajectory.joint_trajectory

                print "Considering object with name = %s"%tf_frame

                if len(traj.points) > 0:
                    qs.append(traj)
                    dists.append((traj.points[-1].positions - self.q0).sum())
                else:
                    rospy.logwarn('SIMPLE DRIVER -- IK failed for %s'%name)

            if len(qs) == 0:
                msg = 'FAILURE - no joint configurations found!'

            possible_goals = zip(dists,qs)
            possible_goals.sort()

            print "POSSIBLE GOALS"
            print possible_goals
            
            (dist,traj) = possible_goals[0]
            rospy.logwarn("Trying to move to frame at distance %f"%(dist))

            msg = self.send_trajectory(traj)

            return msg

        else:
            msg = 'FAILURE - no match to predicate moves'
            return msg

    def set_goal(self,q):
        self.at_goal = False
        self.near_goal = False
        self.goal = pm.fromMatrix(self.kdl_kin.forward(q))

    '''
    Definitely do a planned motion.
    '''
    def plan_to_pose_call(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':
            T = pm.fromMsg(req.target)

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 

            # Send command
            pt = JointTrajectoryPoint()
            traj = self.planner.getCartesianMove(T,self.q0)
            if len(traj.points) > 0:
                (code,res) = self.planner.getPlan(req.target,traj.points[-1].positions)
            else:
                (code,res) = self.planner.getPlan(req.target,self.q0)

            #pt.positions = q
            if code > 0:
              self.at_goal = True
              return 'SUCCESS - at goal'
            if (not res is None) and len(res.planned_trajectory.joint_trajectory.points) > 0:

                disp = DisplayTrajectory()
                disp.trajectory.append(res.planned_trajectory)
                disp.trajectory_start = res.trajectory_start
                self.display_pub.publish(disp)

                traj = res.planned_trajectory.joint_trajectory
                
                return self.send_trajectory(traj)

            else:
                rospy.logerr(res)
                rospy.logerr('DRIVER -- PLANNING failed')
                return 'FAILURE - not in servo mode'
        else:
            rospy.logerr('DRIVER -- not in servo mode!')
            return 'FAILURE - not in servo mode'


    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj):

        rate = rospy.Rate(30)
        t = rospy.Time(0)

        for pt in traj.points[:-1]:
          self.pt_publisher.publish(pt)
          self.set_goal(pt.positions)

          print " -- %s"%(str(pt.positions))
          start_t = rospy.Time.now()

          rospy.sleep(rospy.Duration(pt.time_from_start.to_sec() - t.to_sec()))
          t = pt.time_from_start

          while not self.near_goal:
            if (rospy.Time.now() - start_t).to_sec() > 10*t.to_sec():
                break
            rate.sleep()

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

    '''
    Send a whole sequence of points to a robot...
    that is listening to individual joint states.
    '''
    def send_sequence(self,traj):
        q0 = self.q0
        for q in traj:
            pt = JointTrajectoryPoint(positions=q)
            self.pt_publisher.publish(pt)
            self.set_goal(q)

            #rospy.sleep(0.9*np.sqrt(np.sum((q-q0)**2)))

        if len(traj) > 0:
            self.pt_publisher.publish(pt)
            self.set_goal(traj[-1])
            rate = rospy.Rate(10)
            start_t = rospy.Time.now()

            # wait until robot is at goal
            while not self.at_goal:
                if (rospy.Time.now() - start_t).to_sec() > 10:
                    return 'FAILURE - timeout'
                rate.sleep()

            return 'SUCCESS - moved to pose'

    '''
    Standard movement call.
    Tries a cartesian move, then if that fails goes into a joint-space move.
    '''
    def servo_to_pose_call(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':
            T = pm.fromMsg(req.target)

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 

            # inverse kinematics
            traj = self.planner.getCartesianMove(T,self.q0)
            if len(traj.points) == 0:
                (code,res) = self.planner.getPlan(req.target,self.q0) # find a non-local movement
                if not res is None:
                    traj = res.planned_trajectory.joint_trajectory

            #if len(traj) == 0:
            #    traj.append(self.planner.ik(T,self.q0))

            # Send command
            if len(traj.points) > 0:
                rospy.logwarn("Robot moving to " + str(traj.points[-1].positions))
                return self.send_trajectory(traj)
            else:
                rospy.logerr('SIMPLE DRIVER -- IK failed')
                return 'FAILURE - not in servo mode'
        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE - not in servo mode'

    '''
    set teach mode
    '''
    def set_teach_mode_call(self,req):
        if req.enable == True:

            # self.rob.set_freedrive(True)
            self.driver_status = 'TEACH'
            return 'SUCCESS - teach mode enabled'
        else:
            # self.rob.set_freedrive(False)
            self.driver_status = 'IDLE'
            return 'SUCCESS - teach mode disabled'

    '''
    activate servo mode
    '''
    def set_servo_mode_call(self,req):
        if req.mode == 'SERVO':

            pt = JointTrajectoryPoint()
            pt.positions = self.q0

            self.pt_publisher.publish(pt)

            #rospy.sleep(0.5)

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
            #rospy.logwarn('IIWA mode for %s not specified!'%self.driver_status)
            pass
