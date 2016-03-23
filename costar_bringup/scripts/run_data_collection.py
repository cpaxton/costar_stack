#!/usr/bin/env python

import rospy
import tf
from subprocess import Popen
import tf_conversions.posemath as pm
import datetime
import time

def lookup(frame1,frame2):
    global listener
    done = False
    (trans,rot) = (None,None)
    while not done:
        try:
            (trans,rot) = listener.lookupTransform(frame1,frame2,rospy.Time(0))
            done = True
        except Exception, e:
            print e
            rospy.sleep(0.1)

    return (trans,rot)
        
rospy.init_node('run_task_test_master')
listener = tf.TransformListener()
procs = []
final = [[],[],[],[],[],[],[],[]]

#executables = ["simple_test","simple_test","task_test","task_test",
#        "simple_test","simple_test","task_test","task_test"]
#object_name="link"
#object_orientation="horizontal"
#object_orientation="vertical"

object_name="node"
object_orientation="horizontal"

# run "lsusb" to get a list of bus and device_id
# see /opt/ros/indigo/share/openni_launch/launch/openni.launch for details
# see /opt/ros/indigo/share/openni2_launch/launch/openni.launch
# kinect_cmd = ["roslaunch","openni_launch","openni.launch","depth_registration:=true","device_id:=4@043"]
#primesense_cmd = ["roslaunch","openni_launch","openni.launch","depth_registration:=true"] # WORKS! # used this previously: ,"device_id:=1311220031"
primesense_cmd = ["roslaunch","openni2_launch","openni2.launch","depth_registration:=true"] # WORKS! # used this previously: ,"device_id:=1311220031"
kinect2_cmd = ["roslaunch","kinect2_bridge","kinect2_bridge.launch"] # WORKS!

# with xbox 360 kinectv1
#camera_cmds = [('kinect',kinect_cmd),('primesense',primesense_cmd),('kinect2',kinect2_cmd)]

# without xbox 360 kinectv1
camera_cmds = [('primesense',primesense_cmd),('kinect2',kinect2_cmd)]

  

try:
    i = 0
    while True:
        i += 1

        # test_cmd = ["rosrun","grid_plan"] + [executables[test]]
        for device_name,cmd in camera_cmds:


            ''' --------------------- '''
            ''' LAUNCH THE EXPERIMENT '''
            ''' --------------------- '''
            # if there is only one device only launch it at the beginning
            if len(camera_cmds) > 1 or i==0:
                proc = Popen(cmd)
                procs.append(proc)

            rospy.sleep(30.0)

            ''' --------------------- '''
            '''     START PLANNING    '''
            ''' --------------------- '''
            time_str = time.strftime("%Y_%m_%d_%H_%M_%S")
            print "[%d] Starting test... iteration: %s device_name: %s time: %s object_name: %s object_orientation: %s"%((i),device_name,cmd,time_str,object_name,object_orientation)
            test_proc = Popen(["rosbag","record","-a","--duration=5", "--size=150", "--output-name=%s_iter_%s_%s_%s_%s.bag"%(time_str,(i),device_name,object_name,object_orientation)])

            #test_proc.wait() # WAIT UNTIL TEST IS DONE

            rospy.sleep(15.0);



            print "[%d] Done test!"%(i)


            test_proc.terminate()

            # if there is only one device let it keep running
            if len(camera_cmds) > 1:
                proc.terminate()


            rospy.sleep(15.0)
        rospy.sleep(600)

except Exception, e:
    print e
finally:
    for proc in procs:
        proc.poll()
        if proc.returncode is None:
            print " - Killing %s"%(str(proc))
            proc.terminate()

