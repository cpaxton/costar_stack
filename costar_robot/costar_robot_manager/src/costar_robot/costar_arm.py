
import os
import tf
import rospy
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
from smart_waypoint_manager import SmartWaypointManager
from waypoint_manager import WaypointManager

import copy

class CostarArm(object):

    def make_service(self, name, srv_t, callback, *args, **kwargs):
        service_name = os.path.join(self.namespace, name)
        return rospy.Service(service_name, srv_t, callback, *args, **kwargs)

    def make_pub(self, name, msg_t, *args, **kwargs):
       pub_name = os.path.join(self.namespace, name)
       return rospy.Publisher(pub_name, msg_t, *args, **kwargs)

    def make_service_proxy(self, name, srv_t, use_namespace = True):
        if use_namespace:
            service_name = os.path.join(self.namespace, name)
        else:
            service_name = name
        print "service name: %s"%service_name
        rospy.wait_for_service(service_name)
        return rospy.ServiceProxy(service_name,srv_t)

    def __init__(self,
            base_link, end_link, planning_group,
            world="/world",
               namespace="costar",
            listener=None,
            broadcaster=None,
            traj_step_t=0.1,
            max_acc=1,
            max_vel=1,
            max_goal_diff = 0.02,
            goal_rotation_weight = 0.01,
            max_q_diff = 1e-6,
            start_js_cb=True,
            base_steps=2,
            steps_per_meter=300,
            steps_per_radians=4,
            closed_form_IK_solver = None,
            default_distance = 0.05,
            state_validity_penalty = 1e5,
            table_frame = "table_frame",
            max_dist_from_table = 0.5,
            dof=7,
            debug=False,
            perception_ns="/SPServer",):
        
        self.debug = debug
        self.namespace = namespace
        self.table_frame = table_frame
        self.table_pose = None
        self.max_dist_from_table = max_dist_from_table

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

        # Currently this class does not need a smart waypoint manager.
        # That will remain in the CoSTAR BT.
        #self.smartmove_manager = SmartWaypointManager(
        #        listener=self.listener,
        #        broadcaster=self.broadcaster)

        # TODO: ensure the manager is set up properly
        # Note that while the waypoint manager is currently a part of CostarArm
        # If we wanted to set this up for multiple robots it should be treated
        # as an independent component.
        self.waypoint_manager = WaypointManager(service=True,
                broadcaster=self.broadcaster)

        # Set up services
        # The CostarArm services let the UI put it into teach mode or anything else
        self.teach_mode = self.make_service('SetTeachMode',SetTeachMode,self.set_teach_mode_call)
        self.servo_mode = self.make_service('SetServoMode',SetServoMode,self.set_servo_mode_call)
        self.shutdown = self.make_service('ShutdownArm',EmptyService,self.shutdown_arm_call)
        self.servo = self.make_service('ServoToPose',ServoToPose,self.servo_to_pose_call)
        self.plan = self.make_service('PlanToPose',ServoToPose,self.plan_to_pose_call)
        self.smartmove = self.make_service('SmartMove',SmartMove,self.smart_move_call)
        self.js_servo = self.make_service('ServoToJointState',ServoToJointState,self.servo_to_joints_call)
        self.save_frame = self.make_service('SaveFrame',SaveFrame,self.save_frame_call)
        self.save_joints = self.make_service('SaveJointPosition',SaveFrame,self.save_joints_call)
        self.smartmove_release_srv = self.make_service('SmartRelease',SmartMove,self.smartmove_release_cb)
        self.smartmove_grasp_srv = self.make_service('SmartGrasp',SmartMove,self.smartmove_grasp_cb)
        self.smartmove_query_srv = self.make_service('Query',SmartMove,self.query_cb)

        self.get_waypoints_srv = GetWaypointsService(world=world,
                                                     service=False,
                                                     ns=perception_ns)
        self.driver_status = 'IDLE'
      
        # Create publishers. These will send necessary information out about the state of the robot.
        # TODO(ahundt): this is for the KUKA robot. Make sure it still works.
        self.pt_publisher = rospy.Publisher('/joint_traj_pt_cmd',JointTrajectoryPoint,queue_size=1000)

        self.status_publisher = self.make_pub('DriverStatus',String,queue_size=1000)
        self.display_pub = self.make_pub('display_trajectory',DisplayTrajectory,queue_size=1000)

        self.robot = URDF.from_parameter_server()
        if start_js_cb:
            self.js_subscriber = rospy.Subscriber('joint_states',JointState,self.js_cb)
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain(base_link, end_link)

        # Create reference to pyKDL kinematics
        self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)


        #self.set_goal(self.q0)
        self.goal = None
        self.ee_pose = None

        self.joint_names = [joint.name for joint in self.robot.joints[:self.dof]]

        self.closed_form_IK_solver = closed_form_IK_solver
        self.state_validity_penalty = state_validity_penalty
        # how important is it to choose small rotations in goal poses
        self.rotation_weight = 0.5
        self.joint_space_weight = 0.05

        # for checking robot configuration validity
        self.state_validity_service = rospy.ServiceProxy("/check_state_validity", GetStateValidity)
        self.robot_state = RobotState()
        self.robot_state.joint_state.name = self.joint_names

        self.gripper_close = self.make_service_proxy('gripper/close',EmptyService)
        self.gripper_open = self.make_service_proxy('gripper/open',EmptyService)
        self.get_planning_scene = self.make_service_proxy('/get_planning_scene',GetPlanningScene)
        self.planner = SimplePlanning(self.robot,base_link,end_link,
            self.planning_group,
            kdl_kin=self.kdl_kin,
            joint_names=self.joint_names,
            closed_form_IK_solver=closed_form_IK_solver)

    '''
    Preemption logic -- acquire at the beginning of a trajectory.
    This returns the next stamp, and updates the current master stamp.

    Note that this is not a mutex in the ordinary sense: it doesn't wait. In fact,
    '''
    def acquire(self):
        stamp = rospy.Time.now().to_sec()
        if stamp > self.cur_stamp:
            self.cur_stamp = stamp
            return stamp
        else:
            return None

    '''
    Preemption logic -- release at the end of a trajectory.
    By default, this does nothing. If in your specific implementation it should do something,
    by all means implement that.
    '''
    def release(self):
        self.cur_stamp = 0

    '''
    js_cb
    listen to robot joint state information
    '''
    def js_cb(self,msg):
     
        if len(msg.position) is self.dof:
            pass
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

            #goal_diff = np.abs(self.goal - self.q0).sum() / self.q0.shape[0]
            cart_diff = (self.ee_pose.p - self.goal.p).Norm()
            rot_diff = 0 #self.goal_rotation_weight * (pm.Vector(*self.ee_pose.M.GetRPY()) - pm.Vector(*self.goal.M.GetRPY())).Norm()
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
    Save the current end effector pose as a frame that we can return to
    '''
    def save_frame_call(self,req):
      rospy.logwarn('Save frame does not check to see if your frame already exists!')
      print self.ee_pose
      self.waypoint_manager.save_frame(self.ee_pose, self.world)

      return 'SUCCESS - '

    '''
    Save the current joint states as a frame that we can return to
    '''
    def save_joints_call(self,req):
      rospy.logwarn('Save frame does not check to see if your joint position already exists!')
      print self.q0
      self.waypoint_manager.save_frame(self.q0)

      return 'SUCCESS - '
    
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
    def smart_move_call(self,req):

        if not self.driver_status == 'SERVO':
            rospy.logerr('DRIVER -- Not in servo mode!')
            return 'FAILURE - not in servo mode'

        # Check acceleration and velocity limits
        (acceleration, velocity) = self.check_req_speed_params(req) 
        possible_goals = self.query(req)

        if len(possible_goals) == 0:
            return 'FAILURE -- no valid poses found'

        for (dist,T,obj,name) in possible_goals:
            rospy.logwarn("Trying to move to frame at distance %f"%(dist))

            # plan to T
            (code,res) = self.planner.getPlan(T,self.q0,obj=obj)
            msg = self.send_and_publish_planning_result(res,acceleration,velocity)

            if msg[0:7] == 'SUCCESS':
                break

        return msg

    def set_goal(self,q):
        self.at_goal = False
        self.near_goal = False
        self.goal = pm.fromMatrix(self.kdl_kin.forward(q))
        # rospy.logwarn("set goal to " + str(self.goal))

    def send_and_publish_planning_result(self,res,acceleration,velocity):
        if (not res is None) and len(res.planned_trajectory.joint_trajectory.points) > 0:

            disp = DisplayTrajectory()
            disp.trajectory.append(res.planned_trajectory)
            disp.trajectory_start = res.trajectory_start
            self.display_pub.publish(disp)

            traj = res.planned_trajectory.joint_trajectory
            
            stamp = self.acquire()
            if stamp is not None:
                res = self.send_trajectory(traj,stamp,acceleration,velocity,cartesian=False)
                self.release()
            else:
                res = 'FAILURE - could not preempt current arm control.'

            return res

        else:
            rospy.logerr('DRIVER -- PLANNING failed')
            return 'FAILURE - not in servo mode'


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
            pose = pm.fromMsg(req.target)
            (code,res) = self.planner.getPlan(pose,self.q0)
            return self.send_and_publish_planning_result(res,acceleration,velocity)
        else:
            rospy.logerr('DRIVER -- not in servo mode!')
            return 'FAILURE - not in servo mode'


    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5,cartesian=False):
        rospy.logerr("Function 'send_trajectory' not implemented for base class!")
        return "FAILURE - running base class!"

    '''
    Standard movement call.
    Tries a cartesian move, then if that fails goes into a joint-space move.
    '''
    def servo_to_pose_call(self,req): 
        if self.driver_status == 'SERVO':
            T = pm.fromMsg(req.target)

            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req)

            # inverse kinematics
            traj = self.planner.getCartesianMove(T,
                self.q0,
                self.base_steps,
                self.steps_per_meter,
                self.steps_per_radians)

            # Send command
            if len(traj.points) > 0:
                stamp = self.acquire()
                if stamp is not None:
                    rospy.logwarn("Robot moving to " + str(traj.points[-1].positions))
                    res = self.send_trajectory(traj,stamp,acceleration,velocity,cartesian=False)
                    self.release()
                else:
                    res = 'FAILURE - could not preempt current arm control.'
                return res
            else:
                rospy.logerr('SIMPLE DRIVER -- IK failed')
                return 'FAILURE - not in servo mode'
        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE - not in servo mode'

    '''
    Standard move call.
    Make a joint space move to a destination.
    '''
    def servo_to_joints_call(self,req):

        if self.driver_status == 'SERVO':
            # Check acceleration and velocity limits
            (acceleration, velocity) = self.check_req_speed_params(req) 
            return 'FAILURE - not yet implemented!'

        else:
            rospy.logerr('SIMPLE DRIVER -- Not in servo mode')
            return 'FAILURE - not in servo mode'
        

    '''
    set teach mode
    '''
    def set_teach_mode_call(self,req):
        if req.enable == True:
            self.driver_status = 'TEACH'
            return 'SUCCESS - teach mode enabled'
        else:
            self.driver_status = 'IDLE'
            return 'SUCCESS - teach mode disabled'

    '''
    send a single point
    '''
    def send_q(self,pt,acceleration,velocity):
        pt = JointTrajectoryPoint()
        pt.positions = self.q0

        self.pt_publisher.publish(pt)

    '''
    activate servo mode
    '''
    def set_servo_mode_call(self,req):
        if req.mode == 'SERVO':
            if self.q0 is not None:
                self.send_q(self.q0,0.1,0.1)

            self.driver_status = 'SERVO'
            self.cur_stamp = self.release()
            return 'SUCCESS - servo mode enabled'
        elif req.mode == 'DISABLE':
            self.detach()
            self.cur_stamp = self.acquire()
            self.driver_status = 'IDLE'
            return 'SUCCESS - servo mode disabled'

    def shutdown_arm_call(self,req):
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

        try:
            self.table_pose = self.listener.lookupTransform(self.world,self.table_frame,rospy.Time(0))
        except tf.ExtrapolationException, e:
            rospy.logwarn(str(e))

    '''
    call this when "spinning" to keep updating things
    '''
    def tick(self):
        self.status_publisher.publish(self.driver_status)
        self.update_position()
        self.handle_tick()

        # publish TF messages to display frames
        self.waypoint_manager.publish_tf()
    
    '''
    call this to get rough estimate whether the input robot configuration is in collision or not
    '''
    def check_robot_position_validity(self, robot_joint_position):
        get_state_validity_req = GetStateValidityRequest()
        get_state_validity_req.group_name = self.planning_group
        current_robot_state = self.robot_state
        current_robot_state.joint_state.position = robot_joint_position
        get_state_validity_req.robot_state = current_robot_state
        
        return self.state_validity_service.call(get_state_validity_req)

    '''
    Attach an object to the planning scene.

    TODO(fjonath): currently attached object is missing its geometry.
    '''
    def attach(self, object_name):
        # attach the collision object to the gripper
        self.gripper_close.call()

        # get the actual collision obj from planning scene
        remove_object = None

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
    def detach(self, object_name=""):
        # detach the collision object to the gripper
        self.gripper_open.call()
        # self.planning_group.detachObject(object_name, self.end_link)
        self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene)
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
    def get_best_distance(self, T, T_fwd, q0):
        quaternion_dot = np.dot(np.array(T_fwd.M.GetQuaternion()),np.array(T.M.GetQuaternion()))
        delta_rotation = np.arccos(2 * quaternion_dot ** 2 - 1) 

        q_new = list()

        if self.closed_form_IK_solver == None:
            q_new.append(self.ik(pm.toMatrix(T),self.q0))
        else:
            q_new = self.closed_form_IK_solver.solveIK(pm.toMatrix(T))

        if q_new is not None:
            message_print = ''
            message_print_invalid = ''
            ik_joint_solution_validity = ''
            best_dist = float('inf')
            best_invalid = float('inf')
            best_q = None
            best_q_invalid = None

            for q_i in q_new:
                dq = np.absolute(q_i - self.q0) * self.joint_weights
                combined_distance = (T.p - T_fwd.p).Norm() + \
                      self.rotation_weight * delta_rotation + \
                      self.joint_space_weight * np.sum(dq)
                result = self.check_robot_position_validity(q_i.tolist())
                ik_joint_solution_validity += '%s ' % result.valid

                if result.valid and combined_distance < best_dist:
                    best_q = q_i
                    best_dist = combined_distance
                    message_print = 'valid pose, translation: %f, rotation: %f, dq6: %f, dist: %f'  % \
                     ( (T.p - T_fwd.p).Norm(),self.rotation_weight * delta_rotation, dq[-1], combined_distance )

                elif not result.valid and combined_distance < best_invalid:
                    best_q_invalid = q_i
                    best_invalid = combined_distance 
                    message_print_invalid = 'invalid pose, translation: %f, rotation: %f, dq6: %f, dist: %f'  % \
                     ( (T.p - T_fwd.p).Norm(),self.rotation_weight * delta_rotation, dq[-1], combined_distance )
            valid_pose = (best_q is not None)

            if not valid_pose:
                best_q = best_q_invalid

            return valid_pose, best_dist, best_invalid, message_print, message_print_invalid, best_q
        else:
            return False, float('inf'), float('inf'), 'No valid IK solution', 'No valid IK solution', list()

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
                [req.name] # placeholder name
                )

        if res is None:
            # no poses found that meet the query!
            return []

        (poses,names,objects) = res
        if poses is None:
            return []

        dists = []
        Ts = []
        if self.q0 is None:
        	rospy.logerr("Robot state has not yet been received!")
        	return "FAILURE -- robot state not yet received!"
        T_fwd = pm.fromMatrix(self.kdl_kin.forward(self.q0))
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
            if T.p[2] < self.table_pose[0][2]:
                rospy.logwarn("Ignoring due to relative z: %f < %f"%(T.p[2],self.table_pose[0][2]))
                continue
            dist_from_table = (T.p - pm.Vector(*self.table_pose[0])).Norm()
            if dist_from_table > self.max_dist_from_table:
                rospy.logwarn("Ignoring due to table distance: %f > %f"%(
                    dist_from_table,
                    self.max_dist_from_table))
                continue

            Ts.append(T)
            if disable_target_object_collision:
	            self.planner.updateAllowedCollisions(obj,True)
            valid_pose, best_dist, best_invalid, message_print, message_print_invalid, best_q = self.get_best_distance(T,T_fwd,self.q0)
            if disable_target_object_collision:
	            self.planner.updateAllowedCollisions(obj,False)

            if len(best_q) == 0:
                rospy.logwarn("[QUERY] DID NOT ADD:"+message_print)
                continue
            elif valid_pose:
                print message_print
                dists.append(best_dist)
            else:
                print message_print_invalid
                dists.append(best_invalid + self.state_validity_penalty)


        if len(Ts) == 0:
            possible_goals = []
        else:
            possible_goals = zip(dists,Ts,objects,names)
            possible_goals.sort()

        joint = JointState()
        # joint.position = self.ik(T,self.q0)
        return possible_goals

    def smartmove_multipurpose_gripper(self, possible_goals, distance, gripper_function, velocity, acceleration, backup_in_gripper_frame):
        list_of_valid_sequence = list()
        list_of_invalid_sequence = list()
        self.backoff_waypoints = []

        if self.q0 is None:
            return "FAILURE"
        T_fwd = pm.fromMatrix(self.kdl_kin.forward(self.q0))
        for (dist,T,obj,name) in possible_goals:
           if backup_in_gripper_frame:
               backup_waypoint = kdl.Frame(kdl.Vector(-distance,0.,0.))
               backup_waypoint = T * backup_waypoint
           else:
               backup_waypoint = kdl.Frame(kdl.Vector(0.,0.,distance))
               backup_waypoint = backup_waypoint * T
           self.backoff_waypoints.append(("%s/%s_backoff/%f"%(obj,name,dist),backup_waypoint))
           self.backoff_waypoints.append(("%s/%s_grasp/%f"%(obj,name,dist),T))

           valid_pose, best_dist, best_invalid, message_print, message_print_invalid,best_q = self.get_best_distance(backup_waypoint,T_fwd,self.q0)
           if len(best_q) == 0:
              continue

           if valid_pose:
               list_of_valid_sequence.append((backup_waypoint,T,self.q0,best_q,obj))
           else:
		       list_of_invalid_sequence.append((backup_waypoint,T,self.q0,best_q,obj))

        sequence_to_execute = list()
        if len(list_of_valid_sequence) > 0:
            sequence_to_execute = list_of_valid_sequence + list_of_invalid_sequence
            print 'There is %i valid sequence and %i invalid sequence to try'%(len(list_of_valid_sequence),len(list_of_invalid_sequence))
        else:
            rospy.logwarn("WARNING -- no sequential valid grasp action found")
            sequence_to_execute = list_of_invalid_sequence

        print 'Number of sequence to execute:', len(sequence_to_execute)
        msg = None
        for sequence_number, (backup_waypoint,T,self.q0,best_q,obj) in enumerate(sequence_to_execute,1):
            rospy.logwarn("Trying sequence number %i"%(sequence_number))
            # plan to T
            rospy.sleep(0.1)
            (code,res) = self.planner.getPlan(backup_waypoint,self.q0,obj=None)
            if (not res is None) and len(res.planned_trajectory.joint_trajectory.points) > 0:
                q_2 = res.planned_trajectory.joint_trajectory.points[-1].positions
                (code2,res2) = self.planner.getPlan(T,q_2,obj=obj)
                if ((not res2 is None) and len(res2.planned_trajectory.joint_trajectory.points) > 0):
                    msg = self.send_and_publish_planning_result(res,acceleration,velocity)
                    rospy.sleep(0.1)
                    (code2,res2) = self.planner.getPlan(T,self.q0,obj=obj)
                    if msg[0:7] == 'SUCCESS':
                        msg = self.send_and_publish_planning_result(res2,acceleration,velocity)
                        if msg[0:7] == 'SUCCESS':
                            gripper_function(obj)

                            rospy.sleep(0.1)

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
                            
                            msg = self.send_and_publish_planning_result(res2,acceleration,velocity)
                            return msg
                        else:
                        	rospy.logwarn("Fail to move to grasp pose in sequence %i" % sequence_number)
                    else:
                    	rospy.logwarn("Fail to move to backup pose in sequence %i" % sequence_number)
                else:
                    rospy.logwarn("Plan to pose in sequence %i does not work" % sequence_number)
            else:
                rospy.logwarn("Plan Backoff pose in sequence %i does not work" % sequence_number)
        return msg

    def smartmove_grasp(self, possible_goals, distance, velocity, acceleration):
        # Execute the list of waypoints to the selected object
        # It receive one object frame from select, and do motion planning for that
        # close gripper

        # compute backup from grasp pose in each [list of waypoints]
        # backup transform is [0, 0, -distance]
        # call /costar/gripper/close
        # move back to original tform
        return self.smartmove_multipurpose_gripper(possible_goals, distance, self.attach, velocity, acceleration, True)

    '''
    SmartMove Release:
    - takes list of waypoints and loops over them
    - moves in some distance (hard coded initially?)
    - open gripper
    - move back
    '''
    def smartmove_release(self, possible_goals, distance, velocity, acceleration):
        # Execute the list of waypoints to the selected object
        # open gripper

        # compute backup from release pose in each [list of waypoints]
        # backup transform is [0, 0, -distance]
        # call /costar/gripper/open
        # move back to original tform

        return self.smartmove_multipurpose_gripper(possible_goals, distance, self.detach, velocity, acceleration, False)


    '''
    Wrapper for the RELEASE service
    '''
    def smartmove_release_cb(self, req):
        list_of_waypoints = self.query(req, True)
        if len(list_of_waypoints) == 0:
            return "FAILURE -- no suitable points found"
        distance = req.backoff
        return self.smartmove_release(list_of_waypoints, distance, req.vel, req.accel)

    '''
    Wrapper for the GRASP service
    '''
    def smartmove_grasp_cb(self, req):
        list_of_waypoints = self.query(req, True)
        if len(list_of_waypoints) == 0:
            return "FAILURE -- no suitable points found"
        distance = req.backoff
        return self.smartmove_grasp(list_of_waypoints, distance, req.vel, req.accel)

    '''
    Wrapper for the QUERY service
    '''
    def query_cb(self,req):
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
