#!/usr/bin/env python

import rospy
import tf
import yaml
from std_srvs.srv import Empty as EmptyService
from costar_component import CostarComponent
from librarian_msgs.srv import *
from librarian_msgs.msg import *
from dmp.srv import *
from dmp.msg import *
import numpy as np
from costar_dmp.srv import *
from geometry_msgs.msg import *
from costar_robot_msgs.srv import ServoToPose

class CostarDMP(CostarComponent):

    def __init__(self):
        self.name = "dmp"
        self.namespace = "/costar/dmp"
        self.collecting = False
        self.dmp_computed = False
        self.tau = 0 # A time constant (in seconds) that will cause the DMPs to replay at the same speed they were demonstrated.

        self.start_rec_srv = self.make_service('start_rec',DmpTeach, self.start_rec_cb)
        self.stop_rec_srv = self.make_service('stop_rec', EmptyService, self.stop_rec_cb)
        self.dmp_move_srv = self.make_service('dmp_move', EmptyService, self.dmp_move_cb)

        self.listener = tf.TransformListener();

        rospy.wait_for_service('/librarian/add_type',5)
        rospy.wait_for_service('/librarian/load',5)
        self.add_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        # self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        # self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)

        rospy.wait_for_service('learn_dmp_from_demo')
        self.lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        rospy.wait_for_service('set_active_dmp')
        self.sad = rospy.ServiceProxy('set_active_dmp',SetActiveDMP)
        rospy.wait_for_service('get_dmp_plan')
        self.gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)

        rospy.wait_for_service('/costar/ServoToPose')
        self.servo_to_pose_service = rospy.ServiceProxy('/costar/ServoToPose', ServoToPose)

        self.folder = 'dmp'
        self.add_type_service(self.folder)

        super(CostarDMP, self).__init__(self.name, self.namespace)


    def tick(self):
        if self.collecting == True:
            try:
                (new_trans,new_rot) = self.listener.lookupTransform('/PSM1_psm_base_link','/PSM1_tool_tip_link_virtual',rospy.Time(0))
                new_rot_euler = tf.transformations.euler_from_quaternion(new_rot);
                self.traj['trans'].append(new_trans);
                self.traj['rot'].append(new_rot_euler);
                print new_trans, new_rot, 
            except Exception, e:
                pass

            print "DMP tick() method called with recording."
        else:
            print "DMP tick() method called without recording. "

    def start_rec_cb(self,req):
        self.traj = {'trans':[], 'rot':[], 'traj_name':[], 'reference_frame':[]}
        self.collecting = True
        self.traj['traj_name']= req.teach_name
        self.traj['reference_frame'] = req.reference_frame
        return []

    def stop_rec_cb(self,req):
        self.collecting = False

        # call dmp service to compute/fit DMP from traj
        demotraj = DMPTraj()
        dims = 7		      # 3 for position and 4 for quaternion                
    	dt = 10               # here we use the same time interval as tick(), which is 10ms
    	K = 100               # K_gain value
    	D = 2.0 * np.sqrt(K)  # D_gain value
    	k_gains = [K]*dims
    	d_gains = [D]*dims
    	num_bases = len(self.traj['trans'])       

        for i in range(len(self.traj['trans'])):
        	pt = DMPPoint();
        	lfd_traj = [self.traj['trans'][i][0],self.traj['trans'][i][1], self.traj['trans'][i][2],
        			self.traj['rot'][i][0], self.traj['rot'][i][1], self.traj['rot'][i][2]]
        	pt.positions = lfd_traj
        	demotraj.points.append(pt)
        	demotraj.times.append(10*i) 

        resp = self.lfd(demotraj, k_gains, d_gains, num_bases) # use service call to compute dmp
        self.tau = resp.tau
        self.sad(resp.dmp_list) # set the latest recorded dmp traj as active dmp

        # save to yaml with:
        self.save_service(id=self.name.strip('/'),type=self.folder,text=yaml.dump(self.traj)) # original teach_traj
        self.save_service(id="dmp_computed",type=self.folder,text=yaml.dump(resp))
        self.dmp_computed = True

        return []

    def dmp_move_cb(self,req):
    	if self.dmp_computed == False:
    		resp = yaml.load(self.load_service(id="dmp_computed",type=self.folder).text)
    		self.tau = resp.tau
    		self.sad(resp.dmp_list)

    	#Now, generate a plan
    	(start_trans,start_rot) = self.listener.lookupTransform('/PSM1_psm_base_link','/PSM1_tool_tip_link_virtual',rospy.Time(0))
    	start_rot_euler = tf.transformations.euler_from_quaternion(start_rot)
    	(end_trans,end_rot) = self.listener.lookupTransform('/PSM1_psm_base_link','/endpoint',rospy.Time(0))
    	end_rot_euler = tf.transformations.euler_from_quaternion(end_rot)
    	
    	start_pose = start_trans + start_rot_euler
    	start_velocity = [0.0,0.0,0.0,0.0,0.0,0.0]
    	end_pose = end_trans + end_rot_euler
    	end_thresh = [0.2,0.2,0.2,0.2,0.2,0.2]
    	t_0 = 0
    	                
    	seg_length = -1          #Plan until convergence to goal
    	tau = 2 * self.tau       #Desired plan should take twice as long as demo
    	dt = 10
    	integrate_iter = 5       #dt is rather large, so this is > 1  
    	plan = self.gdp(start_pose, start_velocity, t_0, end_pose, end_thresh, 
                           	   seg_length, tau, dt, integrate_iter)
    	plan_traj_points = plan.plan.points
    	print(type(plan_traj_points),len(plan_traj_points))
    	for i in range(len(plan_traj_points)):
    		traj_point = plan_traj_points[i]
    		traj_point_position = traj_point.positions

    		pose_msg = Pose()
    		pose_msg.position = Point(*traj_point_position[0:3])
    		pose_msg.orientation = Quaternion(*tf.transformations.quaternion_from_euler(traj_point_position[3],
    								traj_point_position[4],traj_point_position[5]))
    		print(pose_msg)
    		self.servo_to_pose_service(target=pose_msg,accel=1,vel=1) # set 1


    	self.save_service(id="dmp_plan",type=self.folder,text=yaml.dump(plan))

    	return []

