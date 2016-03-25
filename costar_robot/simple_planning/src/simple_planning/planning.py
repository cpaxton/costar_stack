'''
(c) 2016 Chris Paxton
'''

import rospy

# import moveit messages
from moveit_msgs.msg import *
from moveit_msgs.srv import *
import actionlib

class SimplePlanning:
    
    def __init__(self,base_link,end_link,group,move_group_ns="/move_group",planning_scene_topic="/planning_scene",robot_ns=""):
        self.base_link = base_link
        self.end_link = end_link
        self.group = group
        self.robot_ns = robot_ns
        self.client = actionlib.SimpleActionClient(move_group_ns, MoveGroupAction)

    def getConstraints(self, frame, q):

        srv = rospy.ServiceProxy(self.robot_ns + "/compute_ik", moveit_msgs.srv.GetPositionIK)

        print frame

        p = geometry_msgs.msg.PoseStamped()
        p.pose.position.x = frame.position.x
        p.pose.position.y = frame.position.y
        p.pose.position.z = frame.position.z
        p.pose.orientation.x = frame.orientation.x
        p.pose.orientation.y = frame.orientation.y
        p.pose.orientation.z = frame.orientation.z
        p.pose.orientation.w = frame.orientation.w
        p.header.frame_id = "/world"

        ik_req = moveit_msgs.msg.PositionIKRequest()
        ik_req.robot_state.joint_state.position = q
        ik_req.avoid_collisions = True
        ik_req.timeout = rospy.Duration(3.0)
        ik_req.attempts = 5
        ik_req.group_name = self.group
        ik_req.pose_stamped = p

        print "Getting IK position..."
        #print p
        #print ik_req
        ik_resp = srv(ik_req)

        print "IK RESULT ERROR CODE = %d"%(ik_resp.error_code.val)

        #print ik_resp.solution

        ###############################
        # now create the goal based on inverse kinematics

        goal = Constraints()

        for i in range(0,len(ik_resp.solution.joint_state.name)):
            print ik_resp.solution.joint_state.name[i]
            print ik_resp.solution.joint_state.position[i]
            joint = JointConstraint()
            joint.joint_name = ik_resp.solution.joint_state.name[i]
            joint.position = ik_resp.solution.joint_state.position[i] 
            joint.tolerance_below = 0.005
            joint.tolerance_above = 0.005
            joint.weight = 1.0
            goal.joint_constraints.append(joint)

        return goal


    def getPlan(self,frame,q):
        planning_options = PlanningOptions()
        planning_options.plan_only = False
        planning_options.replan = False
        planning_options.replan_attempts = 0
        planning_options.replan_delay = 2.0
        planning_options.planning_scene_diff.is_diff = True
        planning_options.planning_scene_diff.robot_state.is_diff = True

        motion_req = MotionPlanRequest()

        motion_req.start_state.joint_state.position = q
        motion_req.workspace_parameters.header.frame_id = self.base_link
        motion_req.workspace_parameters.max_corner.x = 2.0
        motion_req.workspace_parameters.max_corner.y = 2.0
        motion_req.workspace_parameters.max_corner.z = 2.0
        motion_req.workspace_parameters.min_corner.x = -2.0
        motion_req.workspace_parameters.min_corner.y = -2.0
        motion_req.workspace_parameters.min_corner.z = -2.0

        # create the goal constraints
        motion_req.goal_constraints.append(self.getConstraints(frame,q))
        motion_req.group_name = self.group
        motion_req.num_planning_attempts = 10
        motion_req.allowed_planning_time = 5.0
        motion_req.planner_id = "RRTstarkConfigDefault"
        
        if len(motion_req.goal_constraints[0].joint_constraints) == 0:
            return None

        goal = MoveGroupGoal()
        goal.planning_options = planning_options
        goal.request = motion_req

        rospy.logwarn( "Sending request...")

        self.client.send_goal(goal)
        self.client.wait_for_result()
        res = self.client.get_result()

        rospy.logwarn("Done: " + str(res.error_code.val))

        #print res

        return res
