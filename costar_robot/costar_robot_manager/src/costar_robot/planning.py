'''
(c) 2016 Chris Paxton
'''

import rospy

import PyKDL as kdl
import numpy as np

import tf_conversions.posemath as pm

# import moveit messages
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import actionlib

from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
ModeJoints = 'joints'
ModeCart = 'cartesian'

class SimplePlanning:
    
    def __init__(self,robot,base_link,end_link,group,
            move_group_ns="move_group",
            planning_scene_topic="planning_scene",
            robot_ns="",
            verbose=False,
            kdl_kin=None,
            closed_form_IK_solver = None,
            joint_names=[]):
        self.robot = robot
        self.tree = kdl_tree_from_urdf_model(self.robot)
        self.chain = self.tree.getChain(base_link, end_link)
        if kdl_kin is None:
          self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)
        else:
          self.kdl_kin = kdl_kin
        self.base_link = base_link
        self.joint_names = joint_names
        self.end_link = end_link
        self.group = group
        self.robot_ns = robot_ns
        self.client = actionlib.SimpleActionClient(move_group_ns, MoveGroupAction)

        rospy.wait_for_service('/compute_cartesian_path')
        self.cartesian_path_plan = rospy.ServiceProxy('/compute_cartesian_path',GetCartesianPath)

        self.verbose = verbose
        self.closed_form_IK_solver = closed_form_IK_solver
    
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

    def getCartesianMove(self, frame, q0, base_steps=1000, steps_per_meter=1000, steps_per_radians = 4, time_multiplier=2):

      # interpolate between start and goal
      pose = pm.fromMatrix(self.kdl_kin.forward(q0))

      cur_rpy = np.array(pose.M.GetRPY())
      cur_xyz = np.array(pose.p)
      
      goal_rpy = np.array(frame.M.GetRPY())
      goal_xyz = np.array(frame.p)
      delta_rpy = np.linalg.norm(goal_rpy - cur_rpy)
      delta_translation = (pose.p - frame.p).Norm()

      steps = base_steps + int(delta_translation * steps_per_meter) + int(delta_rpy * steps_per_radians)
      print " -- Computing %f steps"%steps
      min_time = 0.5 #seconds
      ts = ((min_time/time_multiplier + delta_translation + delta_rpy) / steps ) * time_multiplier
      traj = JointTrajectory()
      traj.points.append(JointTrajectoryPoint(positions=q0,
          	velocities=[0]*len(q0),
          	accelerations=[0]*len(q0)))

      # compute IK
      for i in range(1,steps+4):
        xyz = None
        rpy = None
        if i >= steps:
          # slow down at 3 final step: at 50% 25% 10% speed before stopping
          incremental_step = None
          if i == steps:
            incremental_step = 0.5
          elif i == steps + 1:
            incremental_step = 0.75
          elif i == steps + 2:
            incremental_step = 0.9
          else:
            incremental_step = 1.0
          xyz = cur_xyz + ((steps-1)+incremental_step)/steps * (goal_xyz - cur_xyz)
          rpy = cur_rpy + ((steps-1)+incremental_step)/steps * (goal_rpy - cur_rpy)
        else:
          xyz = cur_xyz + ((float(i)/steps) * (goal_xyz - cur_xyz))
          rpy = cur_rpy + ((float(i)/steps) * (goal_rpy - cur_rpy))

        frame = pm.toMatrix(kdl.Frame(kdl.Rotation.RPY(rpy[0],rpy[1],rpy[2]),kdl.Vector(xyz[0],xyz[1],xyz[2])))
        #q = self.kdl_kin.inverse(frame,q0)
        q = self.ik(frame, q0)

        if self.verbose:
          print "%d -- %s %s = %s"%(i,str(xyz),str(rpy),str(q))

        if not q is None:
          pt = JointTrajectoryPoint(positions=q,
          	velocities=[0]*len(q),
          	accelerations=[0]*len(q))
          pt.time_from_start = rospy.Duration(i * ts)
          traj.points.append(pt)
          q0 = q
        elif i == steps:
          return JointTrajectory()

      if len(traj.points) < base_steps:
          print rospy.logerr("Planning failure with " \
                  + str(len(traj.points)) \
                  + " / " + str(base_steps) \
                  + " points.")
          return JointTrajectory()

      traj.joint_names = self.joint_names
      return traj

    def getGoalConstraints(self, frames, q, timeout=2.0, mode = ModeJoints):
        is_list_of_frame = isinstance(frames, list)
        number_of_frames = None

        if len(self.robot_ns) > 0:
            srv = rospy.ServiceProxy(self.robot_ns + "/compute_ik", moveit_msgs.srv.GetPositionIK)
        else:
            srv = rospy.ServiceProxy("compute_ik", moveit_msgs.srv.GetPositionIK)

        goal = Constraints()
        
        if is_list_of_frame:
          number_of_frames = len(frames)
        else:
          number_of_frames = 1

        # print "Start looping thru cartesian points"

        for i in xrange(number_of_frames):
          frame = None
          joints = None
          previous_q = None


          if is_list_of_frame:
            # print "is a list of frames"
            frame = frames[i]
          else:
            # "just one frame"
            frame = frames

          if i == 0:
            previous_q = q

          joints = self.ik(pm.toMatrix(frame),previous_q)

          if joints is not None:
            previous_q = joints
          else:
            return (None, None)
          # print joints is None
          # print joints

          # if joints is None:

          #     p = geometry_msgs.msg.PoseStamped()
          #     p.pose.position.x = frame.position.x
          #     p.pose.position.y = frame.position.y
          #     p.pose.position.z = frame.position.z
          #     p.pose.orientation.x = frame.orientation.x
          #     p.pose.orientation.y = frame.orientation.y
          #     p.pose.orientation.z = frame.orientation.z
          #     p.pose.orientation.w = frame.orientation.w
          #     p.header.frame_id = "/world"

          #     ik_req = moveit_msgs.msg.PositionIKRequest()
          #     ik_req.robot_state.joint_state.name = self.joint_names
          #     ik_req.robot_state.joint_state.position = q
          #     ik_req.avoid_collisions = True
          #     ik_req.timeout = rospy.Duration(timeout)
          #     ik_req.attempts = 5
          #     ik_req.group_name = self.group
          #     ik_req.pose_stamped = p

          #     rospy.logwarn("Getting IK position...")
          #     ik_resp = srv(ik_req)

          #     rospy.logwarn("IK RESULT ERROR CODE = %d"%(ik_resp.error_code.val))

          #     #if ik_resp.error_code.val > 0:
          #     #  return (ik_resp, None)
          #     #print ik_resp.solution

          #     ###############################
          #     # now create the goal based on inverse kinematics

          #     if not ik_resp.error_code.val < 0:
          #         for i in range(0,len(ik_resp.solution.joint_state.name)):
          #             print ik_resp.solution.joint_state.name[i]
          #             print ik_resp.solution.joint_state.position[i]
          #             joint = JointConstraint()
          #             joint.joint_name = ik_resp.solution.joint_state.name[i]
          #             joint.position = ik_resp.solution.joint_state.position[i] 
          #             joint.tolerance_below = 0.005
          #             joint.tolerance_above = 0.005
          #             joint.weight = 1.0
          #             goal.joint_constraints.append(joint)

          #     return (ik_resp, goal)
          if mode == ModeJoints:
            for i in range(0,len(self.joint_names)):
                  joint = JointConstraint()
                  joint.joint_name = self.joint_names[i]
                  joint.position = joints[i] 
                  joint.tolerance_below = 0.01
                  joint.tolerance_above = 0.01
                  joint.weight = 1.0
                  goal.joint_constraints.append(joint)

          else:
            print 'Setting cartesian constraint'
            # TODO: Try to fix this again. Something is wrong
            cartesian_costraint = PositionConstraint()
            cartesian_costraint.header.frame_id = 'base_link'
            cartesian_costraint.link_name = self.joint_names[-1]
            # cartesian_costraint.target_point_offset = frame.p
            bounding_volume = BoundingVolume()
            sphere_bounding = SolidPrimitive()
            sphere_bounding.type = sphere_bounding.SPHERE;
            # constrain position with sphere 1 mm around target
            sphere_bounding.dimensions.append(0.5)

            bounding_volume.primitives.append(sphere_bounding)
            sphere_pose = Pose()
            sphere_pose.position = frame.p
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)

            cartesian_costraint.constraint_region = bounding_volume
            cartesian_costraint.weight = 1.0
            goal.position_constraints.append(cartesian_costraint)

            orientation_costraint = OrientationConstraint()
            orientation_costraint.header.frame_id = 'base_link'
            orientation_costraint.link_name = self.joint_names[-1]
            orientation_costraint.orientation = frame.M.GetQuaternion()
            orientation_costraint.absolute_x_axis_tolerance = 0.1
            orientation_costraint.absolute_y_axis_tolerance = 0.1
            orientation_costraint.absolute_z_axis_tolerance = 0.1
            orientation_costraint.weight = 1.0
            goal.orientation_constraints.append(orientation_costraint)
            print 'Done'
        return(None, goal)


    def updateAllowedCollisions(self,obj,allowed):
        self.planning_scene_publisher = rospy.Publisher('planning_scene', PlanningScene, queue_size = 10)
        rospy.wait_for_service('/get_planning_scene', 10.0)
        get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        request = PlanningSceneComponents(components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX)
        response = get_planning_scene(request)

        acm = response.scene.allowed_collision_matrix
        if not obj in acm.default_entry_names:
          # add button to allowed collision matrix
          acm.default_entry_names += [obj]
          acm.default_entry_values += [allowed]
        else:
          idx = acm.default_entry_names.index(obj)
          acm.default_entry_values[idx] = allowed;

        planning_scene_diff = PlanningScene(
                is_diff=True,
                allowed_collision_matrix=acm)

        self.planning_scene_publisher.publish(planning_scene_diff)

    def getPlan(self,frame,q,obj=None,compute_ik=True):
        planning_options = PlanningOptions()
        planning_options.plan_only = True
        planning_options.replan = False
        planning_options.replan_attempts = 0
        planning_options.replan_delay = 0.1
        planning_options.planning_scene_diff.is_diff = True
        planning_options.planning_scene_diff.robot_state.is_diff = True
        
        if obj is not None:
          self.updateAllowedCollisions(obj,True);

        motion_req = MotionPlanRequest()

        motion_req.start_state.joint_state.position = q
        motion_req.workspace_parameters.header.frame_id = self.base_link
        motion_req.workspace_parameters.max_corner.x = 1.0
        motion_req.workspace_parameters.max_corner.y = 1.0
        motion_req.workspace_parameters.max_corner.z = 1.0
        motion_req.workspace_parameters.min_corner.x = -1.0
        motion_req.workspace_parameters.min_corner.y = -1.0
        motion_req.workspace_parameters.min_corner.z = -1.0

        # create the goal constraints
        # TODO: change this to use cart goal(s)
        # - frame: take a list of frames
        # - returns: goal contraints
        constrain_mode = ModeJoints
        if compute_ik:
          (ik_resp, goal) = self.getGoalConstraints(frame,q,mode=constrain_mode)
        
        #if (ik_resp.error_code.val > 0):
        #  return (1,None)
        # print goal

        motion_req.goal_constraints.append(goal)
        motion_req.group_name = self.group
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 4.0
        motion_req.planner_id = "RRTConnectkConfigDefault"
        
        if goal is None:
          print 'Error: goal is None'
          return (-31, None)
        elif constrain_mode == ModeJoints and motion_req is not None and len(motion_req.goal_constraints[0].joint_constraints) == 0:
          print 'Error: joint constraints length is 0'
          return (-31, None)
        elif ((not ik_resp is None and ik_resp.error_code.val < 0) or (not ik_resp is None and ik_resp.error_code.val < 0)):
          print 'Error: ik resp failure'
          return (-31, None)

        goal = MoveGroupGoal()
        goal.planning_options = planning_options
        goal.request = motion_req

        rospy.logwarn( "Sending request...")

        self.client.send_goal(goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        rospy.logwarn("Done: " + str(res.error_code.val))

        if obj is not None:
          self.updateAllowedCollisions(obj,False);

        return (res.error_code.val, res)

    def getPlanWaypoints(self,waypoints_in_kdl_frame,q,obj=None):
      cartesian_path_req = GetCartesianPathRequest()
      cartesian_path_req.header.frame_id = self.base_link
      cartesian_path_req.start_state = RobotState()
      cartesian_path_req.start_state.joint_state.name = self.joint_names
      if type(q) is list:
        cartesian_path_req.start_state.joint_state.position = q
      else:
        cartesian_path_req.start_state.joint_state.position = q.tolist()
      cartesian_path_req.group_name = self.group
      cartesian_path_req.link_name = self.joint_names[-1]
      cartesian_path_req.avoid_collisions = False
      cartesian_path_req.max_step = 50
      cartesian_path_req.jump_threshold = 0
      # cartesian_path_req.path_constraints = Constraints()

      if obj is not None:
        self.updateAllowedCollisions(obj,True)
      
      cartesian_path_req.waypoints = list()

      for T in waypoints_in_kdl_frame:
        cartesian_path_req.waypoints.append(pm.toMsg(T))

      res = self.cartesian_path_plan.call(cartesian_path_req)

      if obj is not None:
        self.updateAllowedCollisions(obj,False)

      return (res.error_code.val, res)
