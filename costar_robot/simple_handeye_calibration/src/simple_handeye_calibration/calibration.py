import tf
import rospy
import PyKDL as kdl
import yaml

import tf_conversions.posemath as pm

from std_srvs.srv import Empty
from std_srvs.srv import EmptyResponse
from librarian_msgs.srv import *

'''
Python class that provides a simple calibration service

This is model-based calibration:
    - you need an accurate endpoint for the robot
    - you need a known marker on that robot in the camera system
    - you need a known marker for that robot in t

It also consistently broadcasts this frame to TF.
'''
class SimpleHandeyeCalibration:
    
    def __init__(self,
            marker="ar_marker_0",
            camera_link="camera_link",
            ee_marker="ee_marker",
            base_link="base_link",
            listener = None, 
            broadcaster = None):
        
        if broadcaster is None:
            self.broadcaster = tf.TransformBroadcaster()
        else:
            self.broadcaster = broadcaster

        if listener is None:
            self.listener = tf.TransformListener()
        else:
            self.listener = listener
        
        self.ee_marker = ee_marker
        self.base_link = base_link
        self.marker = marker
        self.camera_link = camera_link

        self.id = "%s_to_%s"%(base_link,camera_link)

        self.trans = (0,0,0)
        self.rot = (0,0,0,1)
    
        self.srv = rospy.Service('calibrate',Empty,self.calibrate)
        self.add_type_service = rospy.ServiceProxy('/librarian/add_type', librarian_msgs.srv.AddType)
        self.save_service = rospy.ServiceProxy('/librarian/save', librarian_msgs.srv.Save)
        self.load_service = rospy.ServiceProxy('/librarian/load', librarian_msgs.srv.Load)

        rospy.wait_for_service('/librarian/add_type')
        self.add_type_service(type="handeye_calibration")

        resp = self.load_service(type="handeye_calibration", id=self.id)
        print resp
        if resp.status.result > 0:
          (self.trans, self.rot) = pm.toTf(pm.fromMsg(yaml.load(resp.text)))


    def lookup(self,parent,child):
        return self.listener.lookupTransform(parent,child,rospy.Time(0))

    '''
    compute transform from base to end
    '''
    def calibrate(self,req):

        print "%s <- %s, %s <- %s"%(self.camera_link,self.marker,self.base_link,self.ee_marker)

        T_cm = pm.fromTf(self.lookup(self.camera_link,self.marker))
        T_be = pm.fromTf(self.lookup(self.base_link,self.ee_marker))

        print T_cm
        print T_be
        
        T = T_cm * T_be.Inverse()
        (self.trans, self.rot) = pm.toTf(T)

        print (self.trans, self.rot)


        print self.save_service(
            type="handeye_calibration",
            id=self.id,
            text=yaml.dump(pm.toMsg(T)))

        return EmptyResponse()

    '''
    broadcast static transform from base to camera
    '''
    def spin(self,rate):
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(self.trans,self.rot,rospy.Time.now(),self.base_link,self.camera_link)
            rate.sleep()
