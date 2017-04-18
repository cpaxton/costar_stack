import rospy
import numpy as np
import tf_conversions.posemath as pm
from costar_robot import CostarArm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import String

from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory

# for creating client for ur_modern_driver
import actionlib
from actionlib import GoalStatus

class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2

# for actions
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryGoal

# Use IK solver by Felix Jonathan
from inverseKinematicsUR5 import InverseKinematicsUR5

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm', 'IDLE':'PauseArm'}
urscript_commands = {'TEACH':'set robotmode freedrive','SERVO':'set robotmode run'}

class CostarUR5Driver(CostarArm):

    def __init__(self, simulation=False,
            max_dist_from_table = 0.35,
            *args,
            **kwargs):

        self.simulation = simulation
        self.ur_script_pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=10,*args,**kwargs)

        self.closed_form_IK_solver = InverseKinematicsUR5()
        self.closed_form_IK_solver.setEERotationOffsetROS()
        # self.closed_form_IK_solver.setJointWeights(self.joint_weights)
        self.closed_form_IK_solver.setJointLimits(-np.pi, np.pi)

        super(CostarUR5Driver, self).__init__(steps_per_meter=10,
            base_steps=5,
            dof=6,
            closed_form_IK_solver=self.closed_form_IK_solver)

        self.client = client = actionlib.SimpleActionClient('follow_joint_trajectory',FollowJointTrajectoryAction)

    def acquire(self):
        stamp = rospy.Time.now().to_sec()

        self.cur_stamp_mtx.acquire()
        if self.cur_stamp is not None:
            self.client.stop_tracking_goal()
            # self.client.wait_for_result()
            self.client.cancel_all_goals()
            rospy.sleep(0.1)
            # rospy.logwarn("[UR_DRIVER]: current stamp is not None?")
        self.cur_stamp_mtx.release()

        if stamp > self.cur_stamp:
            self.cur_stamp_mtx.acquire()
            self.cur_stamp = stamp
            self.cur_stamp_mtx.release()
            return stamp
        else:
            return None

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,stamp,acceleration=0.5,velocity=0.5,cartesian=False,linear=False):

        rate = rospy.Rate(30)
        t = rospy.Time(0)
        
        # Make sure that the trajectory 0 is the current joint position
        traj.points[0].positions = self.q0

        if not self.valid_verify(stamp):
            return 'FAILURE - preempted'

        if self.simulation:
            if not linear:
                for pt in traj.points[:-1]:
                    if not cartesian:
                        self.send_q(pt.positions,acceleration,velocity)
                    else:
                        self.send_cart(pt.positions,acceleration,velocity) ##
                    self.set_goal(pt.positions)

                    start_t = rospy.Time.now()

                    rospy.sleep(rospy.Duration(pt.time_from_start.to_sec() - t.to_sec()))
                    t = pt.time_from_start

            if not cartesian:
                self.send_q(traj.points[-1].positions,acceleration,velocity)
            else:
                self.send_cart(traj.points[-1].positions,acceleration,velocity) ##
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
        else:
            self.set_goal(traj.points[-1].positions)

        goal = FollowJointTrajectoryGoal(trajectory=traj)

        if self.valid_verify(stamp):
            self.client.send_goal(goal)
            # max time before returning = 30 s
            done = False
            while self.valid_verify(stamp) and not done:
                done = self.client.wait_for_result(rospy.Duration.from_sec(0.1))

        # Check to make sure we weren't preempted.
        if not self.valid_verify(stamp):
            rospy.logerr('verify failed:'+ str(stamp) + ", " + str(self.cur_stamp))
            res = None
        else:
            rospy.logerr('verify succeeded:'+ str(stamp) + ", " + str(self.cur_stamp))
            res = self.client.get_result()

        if res is not None and res.error_code >= 0:
            return "SUCCESS"
        elif res is None:
            return "FAILURE - UR actionlib call aborted"
        else:
            return "FAILURE - %s"%res.error_code


    '''
    set teach mode
    '''
    def set_teach_mode_cb(self,req,cartesian=False):
        if req.enable == True:
            self.ur_script_pub.publish(urscript_commands['TEACH'])
            self.driver_status = 'TEACH'
            return 'SUCCESS - teach mode enabled'
        else:
            self.ur_script_pub.publish(urscript_commands['SERVO'])
            self.driver_status = 'IDLE'
            return 'SUCCESS - teach mode disabled'

    '''
    send a single joint space position
    '''
    def send_q(self,q,acceleration,velocity):
        pt = JointTrajectoryPoint()
        pt.positions = q

        self.pt_publisher.publish(pt)

    '''
    URX supports Cartesian moves, so we will recover our forward kinematics
    and send a Cartesian pose.
    '''
    def send_cart(self,q,acceleration,velocity):
        if self.simulation:
            pt = JointTrajectoryPoint()
            pt.positions = q

            self.pt_publisher.publish(pt)
        else:
            rospy.logwarn('UR5 cartesian moves are currently unsupported.')
        #else:
        # @TODO(cpaxton) implement UR5 cartesian moves
        #  T = pm.fromMatrix(self.kdl_kin.forward(q))
        #  (angle,axis) = T.M.GetRotAngle()
        #  cmd = list(T.p) + [angle*axis[0],angle*axis[1],angle*axis[2]]
        #  self.ur.movel(cmd,wait=True,acc=acceleration,vel=velocity)


    '''
    The basic handle_tick function just needs to deal with the parents.
    '''
    def handle_tick(self):
        super(CostarUR5Driver, self).handle_tick()

        # TODO(cpaxton) or TODO(fjonath): any necessary handling here?
        if self.driver_status in mode.keys():
            if self.driver_status == 'SHUTDOWN':
                pass
            elif self.driver_status == 'SERVO':
                pass
            elif self.driver_status == 'IDLE':
                pass
            elif self.driver_status == 'TEACH':
                pass
 

