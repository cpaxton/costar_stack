#!/usr/bin/env python

import rospy
import signal
import subprocess
import sys
import time
import unittest

sub = None
q = [0.]*7

def js_cb(msg):
    q = msg.position

def startup():
    gazebo = subprocess.Popen(["roslaunch","iiwa_gazebo","iiwa_gazebo.launch","trajectory:=false"])

    rospy.init_node('test_node')
    rospy.wait_for_service('/gazebo/reset_world',10.)
    time.sleep(10.)
    from sensor_msgs.msg import JointState
    global sub
    sub = rospy.Subscriber('/iiwa/joint_states',JointState,js_cb)

    costar = subprocess.Popen(["roslaunch","costar_bringup","iiwa14_s_model.launch","sim:=True","start_sim:=False"])
    rospy.wait_for_service('/costar/ServoToPose',10.)
    time.sleep(30.)

    return gazebo, costar

class TestIIWA(unittest.TestCase):

    def test_home(self):
        rospy.logwarn('TEST 1: HOME')
        global q
        time.sleep(10.);
        for qq in q:
            self.assertAlmostEqual(qq,0.0,places=2)

    def test_move(self):
        from costar_robot_msgs.srv import SetServoMode, ServoToPose
        srv = rospy.ServiceProxy('/costar/SetServoMode',SetServoMode)
        srv(mode='SERVO')
        move = rospy.ServiceProxy('/costar/ServoToPose',ServoToPose)

        from geometry_msgs.msg import Pose, Point, Quaternion
        goal_pos = Point(*(0.440, 0.021, 0.406))
        goal_rot = Quaternion(*(0.471, 0.547, -0.414, 0.555))
        target = Pose(position=goal_pos, orientation=goal_rot)
        move(target=target,vel=1.0,accel=1.0)

        rospy.logwarn(str(target))

        from tf import TransformListener
        listener = TransformListener()

        rospy.sleep(1.0)
        (trans, rot) = listener.lookupTransform('/base_link', '/endpoint', rospy.Time(0))


def _catch_sigint(g,c):
    g.kill()
    c.kill()
    subprocess.call(['rosnode','kill','--all'])
    suprocess.call(['pkill','-f','gz'])
    sys.exit()

if __name__ == '__main__':
    subprocess.call(['rosnode','kill','--all'])
    subprocess.call(['pkill','-f','gz'])
    time.sleep(5.)
    try:
        g, c = startup()
        catch_sigint = lambda: catch_sigint(g,c)
        signal.signal(signal.SIGINT, catch_sigint)

        unittest.main()

    except Exception, e:
        subprocess.call(['rosnode','kill','--all'])
        suprocess.call(['pkill','-f','gz'])
        g.kill()
        c.kill()
        raise e
    
    subprocess.call(['rosnode','kill','--all'])
    suprocess.call(['pkill','-f','gz'])
    g.kill()
    c.kill()
    rospy.logwarn('DONE')



