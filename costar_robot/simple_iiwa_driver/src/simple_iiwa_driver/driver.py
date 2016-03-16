
import rospy
from costar_robot_msgs.srv import *
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Empty as EmptyService

mode = {'TEACH':'TeachArm', 'SERVO':'MoveArmJointServo', 'SHUTDOWN':'ShutdownArm'}


class SimpleIIWADriver:

    def __init__(self):
        self.teach_mode = rospy.Service('costar/SetTeachMode',SetTeachMode,self.set_teach_mode_call)
        self.servo_mode = rospy.Service('costar/SetServoMode',SetServoMode,self.set_servo_mode_call)
        self.shutdown = rospy.Service('costar/ShutdownArm',EmptyService,self.shutdown_arm_call)
        self.servo = rospy.Service('costar/ServoToPose',ServoToPose,self.servo_to_pose_call)
        self.driver_status = 'IDLE'
        self.status_publisher = rospy.Publisher('costar/DriverStatus',String,queue_size=1000)
        self.iiwa_mode_publisher = rospy.Publisher('interaction_mode',String,queue_size=1000)
        self.pt_publisher = rospy.Publisher('joint_traj_pt_cmd',JointTrajectoryPoint,queue_size=1000)

    def servo_to_pose_call(self,req): 
        if self.driver_status == 'SERVO':
            T = tf_c.fromMsg(req.target)
            a,axis = T.M.GetRotAngle()
            pose = list(T.p) + [a*axis[0],a*axis[1],a*axis[2]]
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
            self.rob.movel(pose,acc=acceleration,vel=velocity)
            return 'SUCCESS - moved to pose'
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
