import rospy
import tf
from costar_objrec_msgs.msg import *
import PyKDL as kdl
import tf_conversions.posemath as pm
import numpy as np
from predicator_msgs.msg import *

class ObjectSymmetryRepublisher:

    def __init__(self,broadcaster=None):
        if broadcaster is None:
            self.broadcaster = tf.TransformBroadcaster()
        else:
            self.broadcaster = broadcaster

        self.sub = rospy.Subscriber('detected_object_list',DetectedObjectList,self.callback)
        self.pub = rospy.Publisher('predicator/input',PredicateList,queue_size=1000)
        self.vpub = rospy.Publisher('predicator/valid_input',ValidPredicates,queue_size=1000)

        self.predicator_msg = PredicateList()
        self.valid_predicates = ValidPredicates()

    def computeAndSendRotations(self,obj_id,dim,num_symmetries,rotation,kdlRot):
            names = []

            angle = rotation
            if angle == 0 and num_symmetries > 0:
                angle = 2*np.pi / float(num_symmetries)
            for i in range(1,num_symmetries):
                # rotate the object
                R = kdl.Frame(kdlRot(angle*(i)))
                (trans,rot) = pm.toTf(R)
                obj_symmetry_name = "%s/%s%d"%(obj_id,dim,i)
                self.broadcaster.sendTransform(
                        (0,0,0),
                        rot,
                        rospy.Time.now(),
                        obj_symmetry_name,
                        obj_id)
                names.append(obj_symmetry_name)

            return names

    '''
    handle incoming detected objects message
    '''
    def callback(self,msg):

        self.valid_predicates = ValidPredicates()
        self.PredicateList = PredicateList()

        for obj in msg.objects:
            names = []

            xnames = self.computeAndSendRotations(obj.id,'x',obj.symmetry.x_symmetries,obj.symmetry.x_rotation,kdl.Rotation.RotX)
            ynames = self.computeAndSendRotations(obj.id,'y',obj.symmetry.y_symmetries,obj.symmetry.y_rotation,kdl.Rotation.RotY)
            znames = self.computeAndSendRotations(obj.id,'z',obj.symmetry.z_symmetries,obj.symmetry.z_rotation,kdl.Rotation.RotZ)

            names += xnames
            names += ynames
            names += znames

            print names

    '''
    spin for a while...
    ''' 
    def spin(self,rate=10):
        ros_rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
           ros_rate.sleep()
