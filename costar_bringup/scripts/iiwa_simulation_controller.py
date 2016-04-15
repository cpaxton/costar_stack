#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectoryPoint

rospy.init_node('iiwa_simulation_controller_node')

publishers = []

def joint_traj_pt_cb(msg):
    global publishers
    #print publishers
    print msg.positions

    length = min(len(msg.positions),7)
    #print length

    for i in range(0,length):
        #print i
        publishers[i].publish(msg.positions[i])

for i in range(1,8):
    pub_name = "PositionJointInterface_J%d_controller/command"%(i)
    print 'Adding publisher with name "%s"...'%(pub_name)
    publishers.append(rospy.Publisher(pub_name,Float64,queue_size=1000))

sub = rospy.Subscriber("/joint_traj_pt_cmd",JointTrajectoryPoint,joint_traj_pt_cb)

rospy.spin()
