import rospy
import tf
from costar_objrec_msgs.msg import *
import PyKDL as kdl
import tf_conversions.posemath as pm
import numpy as np

class ObjectSymmetryRepublisher:

    def __init__(self,broadcaster=None):
        if broadcaster is None:
            self.broadcaster = tf.TransformBroadcaster()
        else:
            self.broadcaster = broadcaster

        self.sub = rospy.Subscriber('detected_object_list',DetectedObjectList,self.callback)

    def callback(self,msg):
        for obj in msg.objects:

            angle = obj.symmetry.x_rotation
            if angle == 0 and obj.symmetry.x_symmetries > 0:
                angle = 2*np.pi / float(obj.symmetry.x_symmetries)
            for i in range(1,obj.symmetry.x_symmetries):
                # rotate the object
                R = kdl.Frame(kdl.Rotation.RotX(angle*(i)))
                (trans,rot) = pm.toTf(R)
                self.broadcaster.sendTransform(
                        (0,0,0),
                        rot,
                        rospy.Time.now(),
                        "%s::x%d"%(obj.id,i),
                        obj.id)

            angle = obj.symmetry.y_rotation
            if angle == 0 and obj.symmetry.y_symmetries > 0:
                angle = 2*np.pi / float(obj.symmetry.y_symmetries)
            for i in range(1,obj.symmetry.z_symmetries):
                # rotate the object
                R = kdl.Frame(kdl.Rotation.RotY(angle*(i)))
                (trans,rot) = pm.toTf(R)
                self.broadcaster.sendTransform(
                        trans,
                        rot,
                        rospy.Time.now(),
                        "%s::x%d"%(obj.id,i),
                        obj.id)

            angle = obj.symmetry.z_rotation
            if angle == 0 and obj.symmetry.z_symmetries > 0:
                angle = 2*np.pi / float(obj.symmetry.z_symmetries)
            for i in range(1,obj.symmetry.z_symmetries):
                # rotate the object
                R = kdl.Frame(kdl.Rotation.RotZ(angle*(i)))
                (trans,rot) = pm.toTf(R)
                self.broadcaster.sendTransform(
                        trans,
                        rot,
                        rospy.Time.now(),
                        "%s::x%d"%(obj.id,i),
                        obj.id)



