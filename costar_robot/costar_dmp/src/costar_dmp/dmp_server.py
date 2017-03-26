#!/usr/bin/env python

import rospy
import tf
import yaml
from std_srvs.srv import Empty as EmptyService
from costar_component import CostarComponent
from librarian_msgs.srv import *
from librarian_msgs.msg import *

class CostarDMP(CostarComponent):

    def __init__(self):
        self.name = "dmp"
        self.namespace = "/costar/dmp"
        self.collecting = False

        self.start_rec_srv = self.make_service('start_rec',EmptyService, self.start_rec_cb)
        self.stop_rec_srv = self.make_service('stop_rec', EmptyService, self.stop_rec_cb)

        self.listener = tf.TransformListener();

        rospy.wait_for_service('/librarian/add_type',5)
        # rospy.wait_for_service('/librarian/load',5)
        self.add_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        # self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)
        # self.list_service = rospy.ServiceProxy('/librarian/list', librarian_msgs.srv.List)
        # self.delete_service = rospy.ServiceProxy('/librarian/delete', librarian_msgs.srv.Delete)

        self.folder = 'dmp'
        self.add_type_service(self.folder)

        super(CostarDMP, self).__init__(self.name, self.namespace)


    def tick(self):
        if self.collecting == True:
            (new_trans,new_rot) = self.listener.lookupTransform('/PSM1_psm_base_link','/PSM1_tool_tip_link_virtual',rospy.Time(0))
            self.traj['trans'].append(new_trans);
            self.traj['rot'].append(new_rot);

            print "DMP tick() method called with recording."
        else:
            print "DMP tick() method called without recording. "

    def start_rec_cb(self,req):
        self.collecting = True
        # self.traj_name = req.name
        # self.reference_frame = req.refernce_frame
        self.traj = {'trans':[], 'rot':[]}
        return

    def stop_rec_cb(self,req):
        self.collecting = False

        # call dmp service to compute/fit DMP from traj

        # save to yaml with:
        self.save_service(id=self.name.strip('/'),type=self.folder,text=yaml.dump(self.traj))

        return


