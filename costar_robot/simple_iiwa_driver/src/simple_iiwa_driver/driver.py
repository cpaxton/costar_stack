
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
        self.driver_status = 'IDLE'
        self.status_publisher = rospy.Publisher('costar/DriverStatus',String,queue_size=1000)
        self.iiwa_mode_publisher = rospy.Publisher('interaction_mode',String,queue_size=1000)
        self.pt_publisher = rospy.Publisher('joint_traj_pt_cmd',JointTrajectoryPoint,queue_size=1000)

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
