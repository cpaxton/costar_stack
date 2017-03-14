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
        goal_pos = Point(*(0.440192456863, 0.0208541713193, 0.406347760719))
        goal_rot = Quaternion(*(0.471173160807, 0.54682704131, -0.413921285917, 0.554657739955))
        target = Pose(position=goal_pos, orientation=goal_rot)

        """
        WHEN USING INSTRUCTOR:

        [ERROR] [WallTime: 1489518153.954650] [75.399000] [[   0.0592987,     0.97447,    0.216547;
            0.0561312,     0.21333,   -0.975366;
            -0.996661,    0.069993,  -0.0420479]
        [    0.440192,   0.0208542,    0.406348]]
        [ERROR] [WallTime: 1489518153.955106] [75.399000] [[   0.0592987,     0.97447,    0.216547;
            0.0561312,     0.21333,   -0.975366;
            -0.996661,    0.069993,  -0.0420479]
        [    0.440192,   0.0208542,    0.406348]]
        [ERROR] [WallTime: 1489518153.955971] [75.399000] target: 
          position: 
            x: 0.440192456863
            y: 0.0208541713193
            z: 0.406347760719
          orientation: 
            x: 0.471173160807
            y: 0.54682704131
            z: -0.413921285917
            w: 0.554657739955
        accel: 0.75
        vel: 0.75
        """

        rospy.logwarn(str(target))

        move(target=target,vel=0.25,accel=0.25)
        rospy.sleep(0.1)
        move(target=target,vel=0.25,accel=0.25)
        rospy.sleep(0.1)
        move(target=target,vel=0.25,accel=0.25)
        rospy.sleep(0.1)

        from tf import TransformListener
        listener = TransformListener()

        rospy.sleep(1.0)
        (trans, rot) = listener.lookupTransform('/base_link', '/endpoint', rospy.Time(0))

        rospy.logwarn("result = %s, %s"%(str(trans),str(rot)))

        self.assertAlmostEqual(trans[0], goal_pos.x, places=2)
        self.assertAlmostEqual(trans[1], goal_pos.y, places=2)
        self.assertAlmostEqual(trans[2], goal_pos.z, places=2)


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



