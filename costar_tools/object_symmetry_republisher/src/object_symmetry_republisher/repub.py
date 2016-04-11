import rospy
import tf
from costar_objrec_msgs.msg import *


class ObjectSymmetryRepublisher:

    def __init__(self,broadcaster=None):
        if broadcaster is None:
            self.broadcaster = tf.TransformBroadcaster()
        else:
            self.broadcaster = broadcaster

        self.sub = rospy.Subscriber('detected_object_list',DetectedObjectList,self.callback)


    def callback(self,msg)
        print msg
