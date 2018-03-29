
import os
import tf
import rospy
from costar_component import CostarComponent
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService
from sensor_msgs.msg import JointState
import tf_conversions.posemath as pm
import numpy as np

import PyKDL as kdl
import urdf_parser_py
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from costar_robot import SimplePlanning

from moveit_msgs.msg import *
from moveit_msgs.srv import *

from predicator_landmark import GetWaypointsService

import copy
from threading import Lock

class CostarArm(CostarComponent):

    def __init__(self,
            base_link=None, end_link=None, planning_group=None,
            world="/world",
            namespace="/costar",
            listener=None,
            broadcaster=None,
            traj_step_t=0.1,
            max_acc=1,
            max_vel=1,
            max_goal_diff = 0.02,
            goal_rotation_weight = 0.01,
            max_q_diff = 1e-6,
            base_steps=5,
            steps_per_meter=10,
            steps_per_radians=2,
            closed_form_IK_solver = None,
            default_distance = 0.05,
            state_validity_penalty = 1e5,
            max_dist_from_table = 0.75,
            dof=7,
            debug=False,
            perception_ns="/costar",):

        super(CostarArm, self).__init__(name="Arm", namespace=namespace)
        
        self.debug = debug
        self.namespace = namespace
        self.table_frame = rospy.get_param(os.path.join(self.namespace, "robot", "table_frame"))
        self.table_pose = None
        self.max_dist_from_table = max_dist_from_table

        try:
            self.home_q = rospy.get_param(os.path.join(self.namespace, "robot", "home"))
            self.home_q = [float(q) for q in self.home_q]
        except KeyError, e:
            self.home_q = [0.] * dof

        if base_link is None:
            base_link = rospy.get_param(os.path.join(self.namespace, "robot", "base_link"))
        if end_link is None:
            end_link = rospy.get_param(os.path.join(self.namespace, "robot", "end_link"))
        if planning_group is None:
            planning_group = rospy.get_param(os.path.join(self.namespace, "robot", "planning_group"))

        self.world = world
        self.base_link = base_link
        self.end_link = end_link
        self.planning_group = planning_group
        self.dof = dof
        self.default_distance = default_distance

        self.base_steps = base_steps
        self.steps_per_meter = steps_per_meter
        self.steps_per_radians = steps_per_radians

        self.MAX_ACC = max_acc
        self.MAX_VEL = max_vel

        self.query_frame_name = os.path.join(namespace, "query")
        self.query_frame = None
        self.last_query = None
        self.last_query_idx = None

        self.backoff_waypoints = list()

        self.traj_step_t = traj_step_t

        self.max_goal_diff = max_goal_diff
        self.max_q_diff = max_q_diff
        self.goal_rotation_weight = goal_rotation_weight

        self.at_goal = True
        self.near_goal = True
        self.moving = False
        self.q0 = None
        #[0] * self.dof
        self.old_q0 = [0] * self.dof

        self.cur_stamp_mtx = Lock()

        self.cur_stamp = 0
        self.has_trajectory = 0

        # Set up TF broadcaster
        if not broadcaster is None:
            self.broadcaster = broadcaster
        else:
            self.broadcaster = tf.TransformBroadcaster()

        # Set up TF listener and smartmove manager
        if listener is None:
            self.listener = tf.TransformListener()
        else:
            self.listener = listener

        # Set up services
        # The CostarArm services let the UI put it into teach mode or anything else
        self.teach_mode = self.make_service('SetTeachMode',SetTeachMode,self.set_teach_mode_cb)
        self.servo_mode = self.make_service('SetServoMode',SetServoMode,self.set_servo_mode_cb)
        self.shutdown = self.make_service('ShutdownArm',EmptyService,self.shutdown_arm_cb)
        self.servo = self.make_service('ServoToPose',ServoToPose,self.servo_to_pose_cb)
        self.plan = self.make_service('PlanToPose',ServoToPose,self.plan_to_pose_cb)
        self.cancel_trajectory = self.make_service('StopTrajectory',EmptyService,self.stop_robot_trajectory_cb)

        self.servo_home_srv = self.make_service('ServoToHome',ServoToPose,self.servo_to_home_cb)
        self.plan_home_srv = self.make_service('PlanToHome',ServoToPose,self.plan_to_home_cb)
        self.smartmove = self.make_service('SmartMove',SmartMove,self.smart_move_cb)
        self.js_servo = self.make_service('ServoToJointState',ServoToJointState,self.servo_to_joints_cb)
        self.js_plan = self.make_service('PlanToJointState',ServoToJointState,self.plan_to_joints_cb)
        self.smartmove_release_srv = self.make_service('SmartRelease',SmartMove,self.smartmove_release_cb)
        self.smartmove_grasp_srv = self.make_service('SmartGrasp',SmartMove,self.smartmove_grasp_cb)
        self.smartmove_grasp_srv = self.make_service('SmartPlace',SmartMove,self.smartmove_place_cb)
        self.smartmove_query_srv = self.make_service('Query',SmartMove,self.query_cb)
        self.enable_collisions_srv = self.make_service('EnableCollision',Object,self.enable_collision_cb)
        self.disable_collisions_srv = self.make_service('DisableCollision',Object,self.disable_collision_cb)

        self.get_waypoints_srv = GetWaypointsService(world=world,
                                                     service=False,
                                                     ns=perception_ns)
        self.driver_status = 'IDLE'
      
        # Create publishers. These will send necessary information out about the state of the robot.
        # TODO(ahundt): this is for the KUKA robot. Make sure it still works.
        self.pt_publisher = rospy.Publisher('/joint_traj_pt_cmd',JointTrajectoryPoint,queue_size=1000)

        self.status_pub = self.make_pub('DriverStatus',String,queue_size=1000)
        self.info_pub = self.make_pub('info',String,queue_size=1000)
        self.object_pub = self.make_pub('SmartMove/object',String,queue_size=1000)
        self.display_pub = self.make_pub('display_trajectory',DisplayTrajectory,queue_size=1000)

        self.robot = URDF.from_parameter_server()
        self.js_subscriber = rospy.Subscriber('joint_states',JointState,self.js_cb)
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain(base_link, end_link)

        # Create reference to pyKDL kinematics
        self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)

        #self.set_goal(self.q0)
        self.goal = None
        self.ee_pose = None

        self.joint_names = [joint.name for joint in self.robot.joints[:self.dof]]
        rospy.loginfo('Setting joint names to: %s'%(str(self.joint_names)))

        self.joint_weights = rospy.get_param(os.path.join(self.namespace + '/robot/', "joint_weights"))
        if not isinstance(self.joint_weights, list) and not len(self.joint_weights) == self.dof:
            raise RuntimeError('loaded bad weights: %s'%(str(self.joint_weights)))
        
        self.closed_form_IK_solver = closed_form_IK_solver
        if self.closed_form_IK_solver is not None:
            self.closed_form_IK_solver.setJointWeights(self.joint_weights)

        self.state_validity_penalty = state_validity_penalty

        # how important is it to choose small rotations in goal poses
        self.rotation_weight = rospy.get_param(os.path.join(self.namespace + '/robot/', "rotation_weight"))
        self.translation_weight = rospy.get_param(os.path.join(self.namespace + '/robot/', "translation_weight"))
        self.joint_space_weight = 1.0

        # for checking robot configuration validity
        self.state_validity_service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        # self.robot_state = RobotState()
        # self.robot_state.joint_state.name = self.joint_names

        has_gripper = rospy.get_param(os.path.join(self.namespace, "robot", "has_gripper"))
        has_planning_scene = rospy.get_param(os.path.join(self.namespace, "robot", "has_planning_scene")) 

        if has_gripper:
          self.gripper_close = self.make_service_proxy('gripper/close',EmptyService)
          self.gripper_open = self.make_service_proxy('gripper/open',EmptyService)

        if has_planning_scene:
            self.get_planning_scene = self.make_service_proxy('get_planning_scene',
                GetPlanningScene,
                use_namespace=False)
            self.planner = SimplePlanning(self.robot,base_link,end_link,
                self.planning_group,
                kdl_kin=self.kdl_kin,
                joint_names=self.joint_names,
                closed_form_IK_solver=closed_form_IK_solver)

        rospy.loginfo("Simple planning interface created successfully.")

    '''
    Preemption logic -- acquire at the beginning of a trajectory.
    This returns the next stamp, and updates the current master stamp.

    Note that this is not a mutex in the ordinary sense: it doesn't wait. In fact,
    '''
    def acquire(self):
        stamp = rospy.Time.now().to_sec()
        if stamp > self.cur_stamp:
            self.cur_stamp_mtx.acquire()
            self.cur_stamp = stamp
            self.cur_stamp_mtx.release()
            return stamp
        else:
            return None

    '''
    Make sure we still own the mutex.
    '''
    def valid_verify(self, stamp):
        self.cur_stamp_mtx.acquire()
        if self.cur_stamp is not None:
            valid = abs(self.cur_stamp - stamp) < 0.001
            # if not valid:
            #     rospy.logwarn('cur_stamp %.3f != %.3f'%(self.cur_stamp,stamp))
        else:
            valid = False
            # rospy.logwarn('cur_stamp is None')
        self.cur_stamp_mtx.release()
        return valid

    '''
    Preemption logic -- release at the end of a trajectory.
    By default, this does nothing. If in your specific implementation it should do something,
    by all means implement that.
    '''
    def release(self):
        self.cur_stamp_mtx.acquire()
        # custom logic here; just maintaining this as an example.
        self.cur_stamp_mtx.release()

    '''
    js_cb
    listen to robot joint state information
    '''
    def js_cb(self,msg):
        if len(msg.position) is self.dof:
            self.old_q0 = self.q0
            self.q0 = np.array(msg.position)
        else:
            rospy.logwarn('Incorrect joint dimensionality')

    '''
    update current position information
    '''
    def update_position(self):

        if self.q0 is None or self.old_q0 is None:
            return

        self.ee_pose = pm.fromMatrix(self.kdl_kin.forward(self.q0))

        if self.goal is not None:

            cart_diff = (self.ee_pose.p - self.goal.p).Norm()
            rot_diff = self.goal_rotation_weight * \
              (pm.Vector(*self.ee_pose.M.GetRPY()) - pm.Vector(*self.goal.M.GetRPY())).Norm()
            goal_diff = cart_diff + rot_diff

            if goal_diff < self.max_goal_diff:
                self.at_goal = True
            else:
                self.at_goal = False

            if goal_diff < 10*self.max_goal_diff:
                self.near_goal = True
        
        q_diff = np.abs(self.old_q0 - self.q0).sum()

        if q_diff < self.max_q_diff:
            self.moving = False
        else:
            self.moving = True

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
    ik: handles calls to KDL inverse kinematics
    '''
    def ik(self, T, q0, dist=0.5):
      q = None
      if self.closed_form_IK_solver is not None:
      #T = pm.toMatrix(F)
        q = self.closed_form_IK_solver.findClosestIK(T,q0)
      else:
        q = self.kdl_kin.inverse(T,q0)

      # NOTE: this results in unsafe behavior; do not use without checks
      #if q is None:
      #    q = self.kdl_kin.inverse(T)
      return q

    '''
    Find any valid object that meets the requirements.
    Find a cartesian path or possibly longer path through joint space.
    '''
    def smart_move_cb(self,req):

        if not self.driver_status == 'SERVO':
            rospy.logerr('DRIVER -- Not in servo mode!')
            return 'FAILURE -- not in servo mode'

        # Check acceleration and velocity limits
        (acceleration, velocity) = self.check_req_speed_params(req) 
        possible_goals = self.query(req)
        stamp = self.acquire()

        if len(possible_goals) == 0:
            return 'FAILURE -- no valid poses found'

        for (dist,T,obj,name) in possible_goals:
            rospy.logwarn("Trying to move to frame at distance %f"%(dist))

            # plan to T
            if not self.valid_verify(stamp):
                rospy.logwarn('Stopping action because robot has been preempted by another process,')
                return msg
            (code,res) = self.planner.getPlan(T,self.q0,obj=obj)
            msg = self.send_and_publish_planning_result(res,stamp,acceleration,velocity)

            if msg[0:7] == 'SUCCESS':
                break

        return msg

    def stop_robot_trajectory_cb(self,req):
        self.detach(actuate = False, add_back_to_planning_scene=False)
        self.acquire()
        return []

    def set_goal(self,q):
        self.at_goal = False
        self.near_goal = False
        self.goal = pm.fromMatrix(self.kdl_kin.forward(q))
        # rospy.logwarn("set goal to " + str(self.goal))

    def send_and_publish_planning_result(self,res,stamp,acceleration,velocity):
        if (not res is None) and len(res.planned_trajectory.joint_trajectory.points) > 0:
            disp = DisplayTrajectory()
            disp.trajectory.append(res.planned_trajectory)
            disp.trajectory_start = res.trajectory_start
            self.display_pub.publish(disp)

            traj = res.planned_trajectory.joint_trajectory
            
            if stamp is not None:
                rospy.loginfo("Sending trajectory of length " + str(len(traj.points)))
                res = self.send_trajectory(traj,stamp,acceleration,velocity,cartesian=False)
            else:
                res = 'FAILURE -- could not preempt current arm control.'

            return res

        else:
            rospy.logerr('DRIVER -- PLANNING failed')
            return 'FAILURE -- planning failed'


    '''
    Definitely do a planned motion.
    '''
    def plan_to_pose_cb(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':
            T = pm.fromMsg(req.target)

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 
            stamp = self.acquire()

            # Send command
            pt = JointTrajectoryPoint()
            pose = pm.fromMsg(req.target)
            (code,res) = self.planner.getPlan(frame=pose,q=self.q0)
            return self.send_and_publish_planning_result(res,stamp,acceleration,velocity)
        else:
            rospy.logerr('DRIVER -- not in servo mode!')
            return 'FAILURE -- not in servo mode'

    '''
    Definitely do a planned motion to the home joint state.
    '''
    def plan_to_home_cb(self,req): 
        #rospy.loginfo('Recieved servo to pose request')
        #print req
        if self.driver_status == 'SERVO':

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 
            stamp = self.acquire()

            # Send command
            pt = JointTrajectoryPoint()
            (code,res) = self.planner.getPlan(q_goal=self.home_q,q=self.q0)
            rospy.loginfo("Planning returned code:" + str(code))
            return self.send_and_publish_planning_result(res,stamp,acceleration,velocity)
        else:
            rospy.logerr('DRIVER -- not in servo mode!')
            return 'FAILURE -- not in servo mode'

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,stamp,acceleration=0.5,velocity=0.5,cartesian=False):
        rospy.logerr("Function 'send_trajectory' not implemented for base class!")
        return "FAILURE -- running base class!"

    def info(self, msg, object_name):
        self.info_pub.publish(data=msg)
        self.object_pub.publish(data=object_name)

    '''
    Standard movement call.
    Tries a cartesian move, then if that fails goes into a joint-space move.
    '''
    def servo_to_pose_cb(self,req): 
        if self.driver_status == 'SERVO':
            T = pm.fromMsg(req.target)

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req)

            stamp = self.acquire()
            # inverse kinematics
            traj = self.planner.getCartesianMove(T,
                self.q0,
                self.base_steps,
                self.steps_per_meter,
                self.steps_per_radians,
                time_multiplier = (1./velocity),
                percent_acc = acceleration,
                use_joint_move = True,
                table_frame = self.table_pose)

            # Send command
            if len(traj.points) > 0:
                if stamp is not None:
                    rospy.logwarn("Robot moving to " + str(traj.points[-1].positions))
                    res = self.send_trajectory(traj,stamp,acceleration,velocity,cartesian=False)
                    self.release()
                else:
                    res = 'FAILURE -- could not preempt current arm control.'
                return res
            else:
                rospy.logerr('SIMPLE DRIVER -- no trajectory points')
                return 'FAILURE -- no trajectory points'
        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE -- not in servo mode'

    def servo_to_home_cb(self,req): 
        if self.driver_status == 'SERVO':

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req)

            stamp = self.acquire()

            # inverse kinematics
            traj = self.planner.getJointMove(self.home_q,
                self.q0,
                self.base_steps,
                self.steps_per_meter,
                self.steps_per_radians,
                time_multiplier = (1./velocity),
                percent_acc = acceleration,
                use_joint_move = True,
                table_frame = self.table_pose)

            # Send command
            if len(traj.points) > 0:
                if stamp is not None:
                    rospy.logwarn("Robot moving to " + str(traj.points[-1].positions))
                    res = self.send_trajectory(traj,stamp,acceleration,velocity,cartesian=False)
                    self.release()
                else:
                    res = 'FAILURE -- could not preempt current arm control.'
                return res
            else:
                rospy.logerr('SIMPLE DRIVER -- IK failed')
                return 'FAILURE -- no trajectory points'
        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE -- not in servo mode'

    def servo_to_joints_cb(self,req):
        if self.driver_status == 'SERVO':

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req)

            stamp = self.acquire()

            # inverse kinematics
            traj = self.planner.getJointMove(req.target.position,
                self.q0,
                self.base_steps,
                self.steps_per_meter,
                self.steps_per_radians,
                time_multiplier = (1./velocity),
                percent_acc = acceleration,
                use_joint_move = True,
                table_frame = self.table_pose)

            # Send command
            if len(traj.points) > 0:
                if stamp is not None:
                    rospy.logwarn("Robot moving to " + str(traj.points[-1].positions))
                    res = self.send_trajectory(traj,stamp,acceleration,velocity,cartesian=False)
                    self.release()
                else:
                    res = 'FAILURE -- could not preempt current arm control.'
                return res
            else:
                rospy.logerr('SIMPLE DRIVER -- IK failed')
                return 'FAILURE -- no trajectory points'
        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE -- not in servo mode'

    def plan_to_joints_cb(self,req):

        if self.driver_status == 'SERVO':
            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 
            stamp = self.acquire()

            # Send command
            pt = JointTrajectoryPoint()
            (code,res) = self.planner.getPlan(q_goal=req.target.position,q=self.q0)
            rospy.loginfo("Planning returned code:" + str(code))
            return self.send_and_publish_planning_result(res,stamp,acceleration,velocity)

        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE -- not in servo mode'
        

    '''
    set teach mode
    '''
    def set_teach_mode_cb(self,req):
        if req.enable == True:
            self.driver_status = 'TEACH'
            return 'SUCCESS -- teach mode enabled'
        else:
            self.driver_status = 'IDLE'
            return 'SUCCESS -- teach mode disabled'

    '''
    send a single point
    '''
    def send_q(self,pt,acceleration,velocity):
        pt = JointTrajectoryPoint()
        pt.positions = self.q0

        self.pt_publisher.publish(pt)

    def enable_collision_cb(self, msg):
        self.planner.updateAllowedCollisions(msg.object,False)
        return "SUCCESS"

    def disable_collision_cb(self, msg):
        rospy.logerr("DISABLING COLLISIONS WITH" + str(msg))
        self.planner.updateAllowedCollisions(msg.object,True)
        remove_object = None

        res = self.get_planning_scene(components=
            PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY))
        all_objects = res.scene.world.collision_objects
        for col_obj in all_objects:
            if col_obj.id == msg.object:
                remove_object = col_obj
                break

        # self.planning_group.attachObject(object_name, self.end_link)
        if remove_object is not None:
            self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size = 10)
            planning_scene_diff = PlanningScene(is_diff=True)
            remove_object.operation = CollisionObject.REMOVE
            del planning_scene_diff.world.collision_objects[:];
            planning_scene_diff.world.collision_objects.append(copy.deepcopy(remove_object));

            self.planning_scene_publisher.publish(planning_scene_diff)
        return "SUCCESS"

    '''
    activate servo mode
    '''
    def set_servo_mode_cb(self,req):
        if req.mode.upper() == 'SERVO':
            if self.q0 is not None:
                self.send_q(self.q0,0.1,0.1)

            self.driver_status = 'SERVO'
            self.release()
            return 'SUCCESS -- servo mode enabled'
        elif req.mode.upper() == 'DISABLE':
            self.detach(actuate = False, add_back_to_planning_scene=False)
            self.acquire()
            self.driver_status = 'IDLE'
            return 'SUCCESS -- servo mode disabled'
        else:
            return 'FAILURE -- invalid servo mode'

    def shutdown_arm_cb(self,req):
        self.driver_status = 'SHUTDOWN'
        pass

    '''
    robot-specific logic to update state every "tick"
    '''
    # TODO: Modify this part
    def handle_tick(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"/endpoint",self.end_link)
        if not self.base_link == "base_link":
            br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"/base_link",self.base_link)

        if self.query_frame is not None:
            trans, rot = self.query_frame
            br.sendTransform(trans, rot, rospy.Time.now(),self.query_frame_name,self.world)

        if self.debug:
            if self.backoff_waypoints is not None:
                for tf_name, transform in self.backoff_waypoints:
                    trans, rot = pm.toTf(transform)
                    br.sendTransform(trans, rot, rospy.Time.now(),tf_name,self.world)

        if self.table_frame is not None:
            self.listener.waitForTransform(self.world, self.table_frame, rospy.Time(), rospy.Duration(1.0))
            try:
                self.table_pose = self.listener.lookupTransform(self.world, self.table_frame, rospy.Time(0))
            except tf.ExtrapolationException, e:
                rospy.logwarn(str(e))

    '''
    call this when "spinning" to keep updating things
    '''
    def tick(self):
        self.status_pub.publish(self.driver_status)
        self.update_position()
        self.handle_tick()
    
    '''
    call this to get rough estimate whether the input robot configuration is in collision or not
    '''
    def check_robot_position_validity(self, robot_joint_position):
        if len(robot_joint_position) != self.dof:
            # Incorrect input joint length
            rospy.logwarn('Incorrect joint length')
            return GetStateValidityResponse(valid = False)
        else:
            get_state_validity_req = GetStateValidityRequest()
            get_state_validity_req.group_name = self.planning_group
            # name = self.joint_names
            # rospy.logwarn(str(self.q0))
            # rospy.logwarn(str(robot_joint_position))
            current_robot_state = RobotState(joint_state=JointState(position=robot_joint_position, name=self.joint_names), is_diff=True) #self.robot_state
            get_state_validity_req.robot_state = current_robot_state
            
            return self.state_validity_service.call(get_state_validity_req)

    '''
    Attach an object to the planning scene.

    TODO(fjonath): currently attached object is missing its geometry.
    '''
    def attach(self, object_name, actuate = True):
        if actuate:
            # attach the collision object to the gripper
            self.gripper_close.call()

        # get the actual collision obj from planning scene
        remove_object = None

        rospy.logwarn('Attaching object: %s'% object_name)
        res = self.get_planning_scene(components=
            PlanningSceneComponents(components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY))
        all_objects = res.scene.world.collision_objects
        for col_obj in all_objects:
            if col_obj.id == object_name:
                remove_object = col_obj
                break

        # self.planning_group.attachObject(object_name, self.end_link)
        if remove_object is not None:
            self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size = 10)
            planning_scene_diff = PlanningScene(is_diff=True)
            remove_object.operation = CollisionObject.REMOVE
            del planning_scene_diff.world.collision_objects[:];
            planning_scene_diff.world.collision_objects.append(copy.deepcopy(remove_object));

            attached_object = AttachedCollisionObject()
            attached_object.object = remove_object
            attached_object.link_name = self.end_link
            attached_object.object.header.frame_id = self.joint_names[-1]
            attached_object.object.operation = CollisionObject.ADD
            del planning_scene_diff.robot_state.attached_collision_objects[:];
            planning_scene_diff.robot_state.attached_collision_objects.append(attached_object);

            self.planning_scene_publisher.publish(planning_scene_diff)

    '''
    Detach an object from the planning scene.
    '''
    def detach(self, object_name="", actuate = True, add_back_to_planning_scene = True):
        if actuate:
            # detach the collision object to the gripper
            self.gripper_open.call()
        # self.planning_group.detachObject(object_name, self.end_link)
        self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size=100)
        planning_scene_diff = PlanningScene(is_diff=True)
        T_fwd = pm.fromMatrix(self.kdl_kin.forward(self.q0))

        # get the actual collision obj from planning scene
        res = self.get_planning_scene(components=
            PlanningSceneComponents(components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS))
        all_objects = res.scene.robot_state.attached_collision_objects
        for col_obj in all_objects:
            rospy.logwarn(str(object_name)+", "+str(col_obj.link_name)+", "+str(col_obj.object.id))

            # remove from attached objects
            diff_obj = AttachedCollisionObject(
                link_name=self.end_link,
                object=CollisionObject(id=col_obj.object.id))
            diff_obj.object.operation = CollisionObject.REMOVE
            planning_scene_diff.robot_state.attached_collision_objects.append(diff_obj)
            
            if add_back_to_planning_scene:
                # add into planning scene
                add_object = col_obj.object
                add_object.header.frame_id = self.base_link
                add_object.operation = CollisionObject.ADD
                for index, mesh_pose in enumerate(col_obj.object.mesh_poses):
                    add_object.mesh_poses[index] = pm.toMsg(T_fwd * pm.fromMsg(mesh_pose))
                planning_scene_diff.world.collision_objects.append(add_object)

            # if col_obj.id == object_name:
            #     detach_object = col_obj
            #     break

        # remove object from the gripper, then add the original collision object
        # detach_object = AttachedCollisionObject()
        # detach_object.object.id = object_name
        # detach_object.link_name = self.end_link
        # detach_object.object.operation = detach_object.object.REMOVE
        # planning_scene_diff.robot_state.attached_collision_objects.append(copy.deepcopy(detach_object));

        # add_object = CollisionObject()

        self.planning_scene_publisher.publish(planning_scene_diff)
    '''
    Calculate the best distance between current joint position to the target pose given a IK solution or list of IK solutions 
    '''
    def get_best_distance(self, T, T_fwd, q0, check_closest_only = False, obj_name = None):
        quaternion_dot = np.dot(np.array(T_fwd.M.GetQuaternion()),np.array(T.M.GetQuaternion()))
        delta_rotation = np.arccos(2 * quaternion_dot ** 2 - 1) 

        q_new = list()

        if self.closed_form_IK_solver == None or check_closest_only:
            q_new.append(self.ik(pm.toMatrix(T),self.q0))
        else:
            q_new = self.closed_form_IK_solver.solveIK(pm.toMatrix(T))

        if q_new is not None and len(q_new) > 0:
            message_print = ''
            message_print_invalid = ''
            ik_joint_solution_validity = ''
            best_dist = float('inf')
            best_invalid = float('inf')
            best_q = None
            best_q_invalid = None

            for q_i in q_new:
                if q_i is None:
                    continue
                dq = np.absolute(q_i - self.q0) * self.joint_weights
                combined_distance = (T.p - T_fwd.p).Norm() * self.translation_weight + \
                      self.rotation_weight * delta_rotation + \
                      self.joint_space_weight * np.sum(dq)

                # rospy.loginfo(str(q_i))
                result = self.check_robot_position_validity(q_i.tolist())

                other_obj_collisions = ''
                if obj_name is not None and not result.valid:
                    other_obj_collision = False
                    contacts = result.contacts
                    for collision in contacts:
                        if collision.contact_body_1 != obj_name and collision.contact_body_2 != obj_name:
                            other_obj_collision = True
                            break
                    # if not other_obj_collision:
                    #     rospy.logwarn('[Query] State Pose is actually valid')
                    #     rospy.logwarn(str(contacts))
                    result.valid = not other_obj_collision
                # ik_joint_solution_validity += '%s ' % result.valid

                if result.valid and combined_distance < best_dist:
                    best_q = q_i
                    best_dist = combined_distance
                    message_print = 'valid %s pose, translation: %f, rotation: %f, dq6: %f, dist: %f'  % \
                     (obj_name ,(T.p - T_fwd.p).Norm(),self.rotation_weight * delta_rotation, dq[-1], combined_distance )

                elif not result.valid and combined_distance < best_invalid:
                    best_q_invalid = q_i
                    best_invalid = combined_distance 
                    message_print_invalid = 'invalid %s pose, translation: %f, rotation: %f, dq6: %f, dist: %f'  % \
                        (obj_name, (T.p - T_fwd.p).Norm(),self.rotation_weight * delta_rotation, dq[-1], combined_distance )
                    message_print_invalid += '\nCollisions found between: '
                    for contact_info in result.contacts:
                        message_print_invalid += '(%s and %s) '%(contact_info.contact_body_1,contact_info.contact_body_2)

            if (best_q is not None) or (best_q_invalid is not None):
                valid_pose = (best_q is not None)
                if not valid_pose:
                    best_q = best_q_invalid
                return valid_pose, best_dist, best_invalid, message_print, message_print_invalid, best_q

        return False, float('inf'), float('inf'), '%s: No valid IK solution'%obj_name,'%s: No valid IK solution'%obj_name, list()

    def query(self, req, disable_target_object_collision = False):
        # Get the best object to manipulate, just like smart move, but without the actual movement
        # This will check robot collision and reachability on all possible object grasp position based on its symmetry.
        # Then, it will returns one of the best symmetry to work with for grasp and release.
        # it will be put on parameter server

        # Find possible poses
        res = self.get_waypoints_srv.get_waypoints(
                req.obj_class, # object class to move to
                req.predicates, # predicates to match
                [req.pose], # offset/transform from each member of the class
                [req.name], # placeholder name
                req.constraints,
                )

        if res is None:
            # no poses found that meet the query!
            rospy.logerr("No waypoints found for query")
            return []

        (poses,names,objects) = res
        if poses is None:
            rospy.logerr("No poses found for query")
            return []

        selected_objs, selected_names = [], []
        dists = []
        Ts = []
        if self.q0 is None:
            rospy.logerr("Robot state has not yet been received!")
            return "FAILURE -- robot state not yet received!"
        T_fwd = pm.fromMatrix(self.kdl_kin.forward(self.q0))

        number_of_valid_query_poses, number_of_invalid_query_poses = 0, 0
        for (pose,name,obj) in zip(poses,names,objects):

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

            # Ignore anything below the table: we don't need to even bother
            # checking that position.
            if self.table_pose is not None:
                if T.p[2] < self.table_pose[0][2]:
                    rospy.logwarn("[QUERY] Ignoring due to relative z: %f < %f x=%f y=%f %s"%(T.p[2],self.table_pose[0][2],T.p[0],T.p[1], obj))
                    continue
                dist_from_table = (T.p - pm.Vector(*self.table_pose[0])).Norm()
            if dist_from_table > self.max_dist_from_table:
                rospy.logwarn("[QUERY] Ignoring due to table distance: %f > %f"%(
                    dist_from_table,
                    self.max_dist_from_table))
                continue
    
            # Get metrics for the best distance to a matching objet
            valid_pose, best_dist, best_invalid, message_print, message_print_invalid, best_q = self.get_best_distance(T,T_fwd,self.q0, check_closest_only = False, obj_name = obj)

            if best_q is None or len(best_q) == 0:
                rospy.logwarn("[QUERY] DID NOT ADD:"+message_print)
                continue
            elif valid_pose:
                rospy.loginfo('[QUERY] %s'%message_print)
                Ts.append(T)
                selected_objs.append(obj)
                selected_names.append(name)
                dists.append(best_dist)
                number_of_valid_query_poses += 1
            else:
                rospy.loginfo('[QUERY] %s'%message_print_invalid)
                Ts.append(T)
                selected_objs.append(obj)
                selected_names.append(name)
                dists.append(best_invalid + self.state_validity_penalty)
                number_of_invalid_query_poses += 1


        if len(Ts) is not len(dists):
            raise RuntimeError('You miscounted the number of transforms somehow.')
        if len(Ts) is not len(selected_objs):
            raise RuntimeError('You miscounted the number of transforms vs. objects somehow.')
        if len(Ts) is not len(selected_names):
            raise RuntimeError('You miscounted the number of transforms vs. names somehow.')

        if len(Ts) == 0:
            possible_goals = []
        else:
            possible_goals = [(d,T,o,n) for (d,T,o,n) in zip(dists,Ts,selected_objs,selected_names) if T is not None]
            possible_goals.sort()


        joint = JointState()
        # joint.position = self.ik(T,self.q0)
        rospy.loginfo("[QUERY] There are %i valid poses and %i invalid poses" % 
                (number_of_valid_query_poses, number_of_invalid_query_poses))

        return possible_goals

    def smartmove_multipurpose_gripper(self,
            stamp,
            possible_goals,
            distance,
            gripper_function,
            velocity,
            acceleration,
            backup_in_gripper_frame):
        '''
        Basic function that handles collecting and aggregating smartmoves
        '''

        list_of_valid_sequence = list()
        list_of_invalid_sequence = list()
        self.backoff_waypoints = []

        if self.q0 is None:
            return "FAILURE -- Initial joint position is None"

        T_fwd = pm.fromMatrix(self.kdl_kin.forward(self.q0))
        for (dist,T,obj,name) in possible_goals:
            if not self.valid_verify(stamp):
                rospy.logwarn('Stopping action because robot has been preempted by another process,')
                return "FAILURE -- Robot has been preempted by another process"

            rospy.loginfo("check: " + str(dist) + " " + str(name))

            if backup_in_gripper_frame:
                backup_waypoint = kdl.Frame(kdl.Vector(-distance,0.,0.))
                backup_waypoint = T * backup_waypoint
            else:
                backup_waypoint = kdl.Frame(kdl.Vector(0.,0.,distance))
                backup_waypoint = backup_waypoint * T

            self.backoff_waypoints.append(("%s/%s_backoff/%f"%(obj,name,dist),backup_waypoint))
            self.backoff_waypoints.append(("%s/%s_grasp/%f"%(obj,name,dist),T))

            query_backup_message = ''
            if dist < self.state_validity_penalty:
                query_backup_message = "valid query's(dist = %.3f) backup msg"%dist
            else:
                query_backup_message = "invalid query's(dist = %.3f) backup msg"%(dist - self.state_validity_penalty)

            valid_pose, best_backup_dist, best_invalid, message_print, message_print_invalid,best_q = self.get_best_distance(backup_waypoint,T_fwd,self.q0, check_closest_only = False,  obj_name = obj)
            if best_q is None or len(best_q) == 0:
                rospy.loginfo('Skipping %s: %s'%(query_backup_message,message_print_invalid))
                continue

            if valid_pose and dist < self.state_validity_penalty:
                list_of_valid_sequence.append((backup_waypoint,T,obj,best_backup_dist,dist,name))
                rospy.loginfo('Valid sequence: %s: %s'%(query_backup_message, message_print))
            else:
                if valid_pose:
                    rospy.loginfo('Invalid sequence: %s: %s'%(query_backup_message, message_print))
                    list_of_invalid_sequence.append((backup_waypoint,T,obj,best_backup_dist,dist,name))
                else:
                    rospy.loginfo('Invalid sequence: %s: %s'%(query_backup_message, message_print_invalid))
                    list_of_invalid_sequence.append((backup_waypoint,T,obj,best_invalid,dist,name))

        sequence_to_execute = list()
        if len(list_of_valid_sequence) > 0:
            sequence_to_execute = list_of_valid_sequence + list_of_invalid_sequence[:5]
        else:
            rospy.logwarn("WARNING -- no sequential valid grasp action found")
            sequence_to_execute = list_of_invalid_sequence[:5]

        rospy.loginfo('There is %i valid sequence and %i invalid sequence to try'%(len(list_of_valid_sequence),len(list_of_invalid_sequence)))
        # print 'Number of sequence to execute:', len(sequence_to_execute)
        msg = None
        for sequence_number, (backup_waypoint,T,obj,backup_dist,query_dist,name) in enumerate(sequence_to_execute,1):
            rospy.loginfo(str(sequence_number) + " moving to " + str(name))
            if not self.valid_verify(stamp):
                rospy.logwarn('Stopping action because robot has been preempted by another process,')
                return "FAILURE -- Robot has been preempted by another process"

            rospy.loginfo("Trying sequence number %i: backup_dist: %.3f query_dist: %.3f"%(sequence_number,backup_dist,query_dist))
            # plan to T
            rospy.sleep(0.01)
            (code,res) = self.planner.getPlan(backup_waypoint,self.q0,obj=None)
            if (not res is None) and len(res.planned_trajectory.joint_trajectory.points) > 0:
                q_2 = res.planned_trajectory.joint_trajectory.points[-1].positions
                (code2,res2) = self.planner.getPlan(T,q_2,obj=obj)

                if ((not res2 is None) and len(res2.planned_trajectory.joint_trajectory.points) > 0):
                    q_3 = res.planned_trajectory.joint_trajectory.points[-1].positions
                    print(q_2,q_3)
                    dist = np.linalg.norm(np.array(q_2)-np.array(q_3))
                    if dist > 2 * backup_dist:
                        rospy.logwarn("Backoff failed for pose %i: distance was %f vs %f"%(sequence_number,dist,2*backup_dist))
                        continue
                    self.info("appoach_%s"%obj, obj)
                    msg = self.send_and_publish_planning_result(res,stamp,acceleration,velocity)
                    rospy.sleep(0.1)

                    if msg[0:7] == 'SUCCESS':
                        self.info("move_to_grasp_%s"%obj, obj)
                        msg = self.send_and_publish_planning_result(res2,stamp,acceleration,velocity)
                        rospy.sleep(0.1)
                        
                        if msg[0:7] == 'SUCCESS':
                            self.info("take_%s"%obj, obj)
                            gripper_function(obj)

                            traj = res2.planned_trajectory.joint_trajectory
                            traj.points.reverse()
                            end_t = traj.points[0].time_from_start
                            for i, pt in enumerate(traj.points):
                                pt.velocities = [-v for v in pt.velocities]
                                pt.accelerations = []
                                pt.effort = []
                                pt.time_from_start = end_t - pt.time_from_start
                            traj.points[0].positions = self.q0
                            traj.points[-1].velocities = [0.]*len(self.q0)
                            
                            self.info("backoff_from_%s"%obj, obj)
                            msg = self.send_and_publish_planning_result(res2,stamp,acceleration,velocity)
                            return msg
                        else:
                            rospy.logwarn("Fail to move to grasp pose in sequence %i" % sequence_number)
                    else:
                        rospy.logwarn("Fail to move to backup pose in sequence %i" % sequence_number)
                else:
                    rospy.logwarn("Plan to pose in sequence %i does not work" % sequence_number)
            else:
                rospy.logwarn("Plan Backoff pose in sequence %i does not work" % sequence_number)
        return "FAILURE -- No sequential motions work."

    def execute_planning_sequence(self, list_of_sequence, obj):
        pass

    def smartmove_grasp(self, stamp, possible_goals, distance, velocity, acceleration):
        # Execute the list of waypoints to the selected object
        # It receive one object frame from select, and do motion planning for that
        # close gripper

        # compute backup from grasp pose in each [list of waypoints]
        # backup transform is [0, 0, -distance]
        # call /costar/gripper/close
        # move back to original tform
        return self.smartmove_multipurpose_gripper(stamp,
                possible_goals,
                distance, self.attach, velocity, acceleration,
                True # is backup in the grasp frame?
                )

    def smartmove_release(self, stamp, possible_goals, distance, velocity, acceleration):
        '''
        SmartMove Release:
        - takes list of waypoints and loops over them
        - moves in some distance (hard coded initially?)
        - open gripper
        - move back
        '''

        # Execute the list of waypoints to the selected object
        # open gripper

        # compute backup from release pose in each [list of waypoints]
        # backup transform is [0, 0, -distance]
        # call /costar/gripper/open
        # move back to original tform

        return self.smartmove_multipurpose_gripper(stamp, possible_goals, distance, self.detach, velocity, acceleration, False)

    def smartmove_place_cb(self, req):
        stamp = self.acquire()
        distance = req.backoff
        T_base_world = pm.fromTf(self.listener.lookupTransform(self.world,self.base_link,rospy.Time(0)))
        pose = req.pose
        T = T_base_world.Inverse()*pm.fromMsg(pose)
        # Create list of waypoints for a place action
        list_of_waypoints = [(0.,T,req.name,str(req.name)+"_goal")]
        return self.smartmove_multipurpose_gripper(stamp,
                list_of_waypoints, # list of waypoints
                distance, # backup distance
                self.detach, # detach object
                req.vel, req.accel,
                False, # is backup in table frame
                )

    def smartmove_release_cb(self, req):
        '''
        Wrapper for the RELEASE service
        '''
        stamp = self.acquire()
        list_of_waypoints = self.query(req, True)
        if len(list_of_waypoints) == 0:
            return "FAILURE -- no suitable waypoints found for release"
        distance = req.backoff
        return self.smartmove_release(stamp, list_of_waypoints, distance, req.vel, req.accel)

    def smartmove_grasp_cb(self, req):
        '''
        Wrapper for the GRASP service
        '''
        stamp = self.acquire()
        list_of_waypoints = self.query(req, True)
        if len(list_of_waypoints) == 0:
            return "FAILURE -- no suitable waypoints found for grasp"
        distance = req.backoff
        rospy.loginfo("making smart grasp request")
        return self.smartmove_grasp(stamp, list_of_waypoints, distance, req.vel, req.accel)

    def query_cb(self,req):
        '''
        Wrapper for the QUERY service. This gets the list of waypoints matching some criteria.
        '''
        list_of_waypoints = self.query(req)
        if len(list_of_waypoints) == 0:
            return "FAILURE"
        else:
            # set param and publish TF frame appropriately under reserved name
            if self.last_query is not None and self.last_query == req:
                # return idx + 1
                self.last_query_idx += 1
                if self.last_query_idx >= len(list_of_waypoints):
                    self.last_query_idx = 0
            else:
                self.last_query = req
                self.last_query_idx = 0
           
            dist,T,object_t,name = list_of_waypoints[self.last_query_idx]          
            self.query_frame = pm.toTf(T)

            return "SUCCESS"
