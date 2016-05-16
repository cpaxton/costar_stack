
import rospy
import urx
import numpy as np
from costar_robot import CostarArm
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm', 'IDLE':'PauseArm'}

class CostarUR5Driver(CostarArm):

    def __init__(self,ip_address,simulation=False,
            world="/world",
            listener=None,
            traj_step_t=0.1,
            max_acc=1,
            max_vel=1,
            max_goal_diff = 0.02,
            goal_rotation_weight = 0.01,
            max_q_diff = 1e-6):

        if not simulation:
            self.ur = urx.Robot(ip_address)
        self.simulation = simulation

        base_link = "base_link"
        end_link = "ee_link"
        planning_group = "manipulator"
        super(CostarUR5Driver, self).__init__(base_link,end_link,planning_group)

        self.js_publisher = rospy.Publisher('joint_states',JointState,queue_size=1000)

        self.q0 = np.array(self.ur.getj())
        self.old_q0 = self.q0
        self.set_goal(self.q0)

        self.joint_names = [joint.name for joint in self.robot.joints[:6]]

    '''
    Send a whole joint trajectory message to a robot...
    that is listening to individual joint states.
    '''
    def send_trajectory(self,traj,acceleration=0.5,velocity=0.5):

        rate = rospy.Rate(30)
        t = rospy.Time(0)

        stamp = rospy.Time.now().to_sec()
        self.cur_stamp = stamp

        for pt in traj.points[:-1]:
          self.send_q(pt.positions,acceleration,velocity)
          self.set_goal(pt.positions)

          print " -- %s"%(str(pt.positions))
          start_t = rospy.Time.now()

          if self.cur_stamp > stamp:
            return 'FAILURE - preempted'

          rospy.sleep(rospy.Duration(pt.time_from_start.to_sec() - t.to_sec()))
          t = pt.time_from_start

        print " -- GOAL: %s"%(str(traj.points[-1].positions))
        self.send_q(traj.points[-1].positions,acceleration,velocity)
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
    set teach mode
    '''
    def set_teach_mode_call(self,req):
        if req.enable == True:

            self.ur.set_freedrive(True)
            self.driver_status = 'TEACH'
            return 'SUCCESS - teach mode enabled'
        else:
            self.ur.set_freedrive(False)
            self.driver_status = 'IDLE'
            return 'SUCCESS - teach mode disabled'

    '''
    send a single joint space position
    '''
    def send_q(self,q,acceleration,velocity):
        if self.simulation:
            pt = JointTrajectoryPoint()
            pt.positions = q

            self.pt_publisher.publish(pt)
        else:
            self.ur.movej(q,wait=False,acc=acceleration,vel=velocity)

    '''
    Send a whole sequence of points to a robot...
    that is listening to individual joint states.
    '''
    def send_sequence(self,traj,acceleration=0.5,velocity=0.5):
        q0 = self.q0
        for q in traj:
            self.send_q(q,acceleration,velocity)
            self.set_goal(q)

            #rospy.sleep(0.9*np.sqrt(np.sum((q-q0)**2)))

        if len(traj) > 0:
            self.send_q(traj[-1],acceleration,velocity)
            self.set_goal(traj[-1])
            rate = rospy.Rate(10)
            start_t = rospy.Time.now()

            # wait until robot is at goal
            while not self.at_goal:
                if (rospy.Time.now() - start_t).to_sec() > 10:
                    return 'FAILURE - timeout'
                rate.sleep()

            return 'SUCCESS - moved to pose'

    def handle_tick(self):

        # send out the joint states
        self.q0 = np.array(self.ur.getj())
        self.js_publisher.publish(JointState(
          header=Header(stamp=rospy.Time.now()),
          name=self.joint_names,
          position=self.q0,
          velocity=[0,0,0,0,0,0],
          effort=[0,0,0,0,0,0]))
        self.update_position()

        if self.driver_status in mode.keys():

            if self.driver_status == 'SHUTDOWN':
                self.ur.cleanup()
                self.ur.shutdown()
            elif self.driver_status == 'SERVO':
                pass
            elif self.driver_status == 'IDLE':
                pass
            elif self.driver_status == 'TEACH':
                pass

        else:
            #rospy.logwarn('IIWA mode for %s not specified!'%self.driver_status)
            pass

