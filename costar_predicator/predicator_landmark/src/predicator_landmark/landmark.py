import rospy

from predicator_msgs.msg import *
from predicator_msgs.srv import *

import tf
import tf_conversions.posemath as pm
import numpy as np

from costar_objrec_msgs.msg import *

'''
convert detected objects into predicate messages we can query later on
'''
class GetWaypointsService:

    valid_msg = None

    def __init__(self,world="world",service=True,ns=""):
        #self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        #self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        #self.valid_msg = ValidPredicates()
        #self.valid_msg.pheader.source = rospy.get_name()
        #self.valid_msg.predicates.append('class_detected')
        #self.valid_msg.predicates.append('member_detected')

        #self.sub = rospy.Subscriber('/landmark_definitions', LandmarkDefinition, self.callback)
        self.world = world
        self.and_srv = rospy.ServiceProxy('/predicator/match_AND',Query)
        if service:
            self.service = rospy.Service('/predicator/get_waypoints',GetWaypoints,self.get_waypoints_srv)
        self.listener = tf.TransformListener()

        self.detected_objects = rospy.Subscriber(ns + '/detected_object_list',
                                                 DetectedObjectList,
                                                 self.detected_objects_cb)
        self.obj_symmetries = {}


    def detected_objects_cb(self,msg):
        for obj in msg.objects:
            obj.symmetry.z_symmetries = max(1,obj.symmetry.z_symmetries)
            obj.symmetry.y_symmetries = max(1,obj.symmetry.y_symmetries)
            obj.symmetry.x_symmetries = max(1,obj.symmetry.x_symmetries)
            self.obj_symmetries[obj.object_class] = obj.symmetry
        
    def get_waypoints_srv(self,req):
        resp = GetWaypointsResponse()
        (poses, names) = self.get_waypoints(req.frame_type,req.predicates,req.transforms,req.names)

        if poses is None or len(poses) < 1:
            resp.msg = 'FAILURE -- message not found!'
            resp.success = False
        else:
            resp.waypoints.poses = poses
            resp.names = names
            resp.msg = 'SUCCESS -- done!'
            resp.success = True

        return resp

    def get_waypoints(self,frame_type,predicates,transforms,names):
        self.and_srv.wait_for_service()

        if not len(names) == len(transforms):
            raise RuntimeError('Names and transforms provided to landmark get_waypoints must be the same length')
        if not len(names) == 1:
            raise NotImplementedError('Passing a list of multiple transforms to get_waypoints is not yet supported.') 

        type_predicate = PredicateStatement()
        type_predicate.predicate = frame_type
        type_predicate.params = ['*','','']

        print "----"
        print "predicate(s) to match:"
        print predicates
        print "----"
        print "type predicate to match:"
        print type_predicate

        res = self.and_srv([type_predicate]+predicates)

        print "Found matches: " + str(res.matching)
        #print res

        if (not res.found) or len(res.matching) < 1:
          return None

        poses = []
        for tform in transforms:
            poses.append(pm.fromMsg(tform))

        if frame_type not in self.obj_symmetries.keys():
            self.obj_symmetries[frame_type] = ObjectSymmetry()
            self.obj_symmetries[frame_type].z_symmetries = 1
            self.obj_symmetries[frame_type].y_symmetries = 1
            self.obj_symmetries[frame_type].x_symmetries = 1

        # Get only unique rotation matrices from the list of rotation matrices generated from RPY
        quaternion_list = list()
        for i in xrange(0, self.obj_symmetries[frame_type].z_symmetries):
            for j in xrange(0, self.obj_symmetries[frame_type].y_symmetries):
                for k in xrange(0, self.obj_symmetries[frame_type].x_symmetries):
                    theta_z = i * self.obj_symmetries[frame_type].z_rotation
                    theta_y = j * self.obj_symmetries[frame_type].y_rotation
                    theta_x = k * self.obj_symmetries[frame_type].x_rotation
                    rot_matrix = pm.Rotation.RPY(theta_x,theta_y,theta_z)
                    quaternion_list.append(rot_matrix.GetQuaternion())
        quaternion_list = np.array(quaternion_list)
        
        unique_rot_matrix = list()
        # quaternion_list = np.around(quaternion_list,decimals=5)
        # b = np.ascontiguousarray(quaternion_list).view(np.dtype((np.void, quaternion_list.dtype.itemsize * quaternion_list.shape[1])))
        # _, unique_indices = np.unique(b, return_index=True)

        # for index in unique_indices:
        #     unique_rot_matrix.append(  pm.Rotation.Quaternion( *quaternion_list[index].tolist() )  )

        
        unique_brute_force = list()
        for i in xrange(len(quaternion_list)):
            unique = True
            for j in xrange(i, len(quaternion_list)):
                if i == j:
                    continue
                else:
                    if abs(quaternion_list[i].dot(quaternion_list[j])) > 0.99:
                        unique = False
                        # print i, j, ' is not unique', quaternion_list[i], quaternion_list[j]
                        break
            if unique:
                unique_brute_force.append(i)
        # print 'unique brute force: ', len(unique_brute_force)
        # print unique_brute_force

        for index in unique_brute_force:
            unique_rot_matrix.append(  pm.Rotation.Quaternion( *quaternion_list[index].tolist() )  )

        new_poses = []
        new_names = []
        objects = []

        for match in res.matching:
            try:
                (trans,rot) = self.listener.lookupTransform(self.world,match,rospy.Time(0))
                # for (pose, name) in zip(poses,names):

                if frame_type in self.obj_symmetries:
                    for rot_matrix in unique_rot_matrix:
                        tform = pm.Frame(rot_matrix)
                        new_poses.append(pm.toMsg(pm.fromTf((trans,rot)) * tform * poses[0]))
                        new_names.append(match + "/" + names[0] + "/x%fy%fz%f"%(rot_matrix.GetRPY()))

                        #print match, str(match + "/" + names[0] + "/x%fy%fz%f"%(rot_matrix.GetRPY())),
                        # pm.toTf(pm.fromTf((trans,rot)) * tform * poses[0])[0]

                        objects.append(match)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn('Could not find transform from %s to %s!'%(self.world,match))

        return (new_poses, new_names, objects)
        
