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

    def computeAndSendRotations(self,obj_id,num_symmetries,rotation,kdlRot):
            angle = rotation
            if angle == 0 and num_symmetries > 0:
                angle = 2*np.pi / float(num_symmetries)
            for i in range(1,num_symmetries):
                # rotate the object
                R = kdl.Frame(kdlRot(angle*(i)))
                (trans,rot) = pm.toTf(R)
                self.broadcaster.sendTransform(
                        (0,0,0),
                        rot,
                        rospy.Time.now(),
                        "%s::x%d"%(obj_id,i),
                        obj_id)


    def callback(self,msg):
        for obj in msg.objects:
            self.computeAndSendRotations(obj.id,obj.symmetry.x_symmetries,obj.symmetry.x_rotation,kdl.Rotation.RotX)
            self.computeAndSendRotations(obj.id,obj.symmetry.y_symmetries,obj.symmetry.y_rotation,kdl.Rotation.RotY)
            self.computeAndSendRotations(obj.id,obj.symmetry.z_symmetries,obj.symmetry.z_rotation,kdl.Rotation.RotZ)
