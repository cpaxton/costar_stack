import rospy

from costar_robot_msgs import *

class SimpleIIWADriver:

    def __init__(self):
        self.teach_mode = rospy.Service('simple_robot/TeachMode',SetTeachMode,set_teach_mode_call)
        self.servo_mode = rospy.Service('simple_robot/ServoMode',SetServoMode,set_servo_mode_call)
        self.driver_state = 'NONE'

    def set_teach_mode_call(self,req):
        if req.enable == True:
            # self.rob.set_freedrive(True)
            self.driver_status = 'TeachArm'
            return 'SUCCESS - teach mode enabled'
        else:
            # self.rob.set_freedrive(False)
            self.driver_status = 'NONE'
            return 'SUCCESS - teach mode disabled'

    def set_servo_mode_call(self,req):
        if req.mode == 'SERVO':
            self.driver_status = 'MoveArmJointServo'
            return 'SUCCESS - servo mode enabled'
        elif req.mode == 'DISABLE':
            self.driver_status = 'IDLE'
            return 'SUCCESS - servo mode disabled'

    def shutdown_arm_call(self,req):
        pass
