import rospy
from costar_objrec_msgs.msg import *
from predicator_msgs.msg import *
import tf
import pyKDL as kdl
import tf_conversions.posemath as pm

'''
Use detected objects to determine some properties of the thing we are building.
Maintains a map saying which links have been found, and whether there is a node attached above or below them.
Also maintains a map of unattached nodes.
If there is a link without an attachment above, our next piece is a node.
Otherwise it's a link.
'''
class AssemblyPredicator:

    valid_msg = None

    def __init__(self, world='/world', err=1e-2):
        self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        self.valid_msg = ValidPredicates()
        self.valid_msg.pheader.source = rospy.get_name()
        self.valid_msg.predicates.append('next_piece')
        self.valid_msg.predicates.append('attached_above')
        self.valid_msg.predicates.append('attached_below')

        self.world = world
        self.err = err

        self.tf_listener = tf.TransformListener()

        self.object_types = ['link_uniform', 'node_uniform']

        self.sub = rospy.Subscriber('/detected_object_list', DetectedObjectList, self.callback)

        self.nodes = {}
        self.links = {}

    def search_position(self, pos, err=1e-2):
        # iterate through the list of nodes
        # if node is found at this position, return its name
        for (name, info) in nodes:
            (frame, connected) = info;
            pass

    def update_assembly(self, objs):

        for obj in objs:
            if obj.object_class == 'node_uniform':
                # add to nodes with position
                frame = kdl.Frame()
                self.nodes[obj.id] = (frame, 0)
                pass

        for obj in objs:
            if obj.object_class == 'link_uniform':
                # check above in z direction
                (trans, rot) = tf.lookupTransform(self.world,
                        obj.id,
                        rospy.Time.now())
                trans_above = trans
                trans[2] += (0.15 / 2)
                node_above = self.search_position(pm.toKDL(trans_above, rot), self.err)

                # check below in z direction
                trans_below = trans
                trans[2] -= (0.15 / 2)
                node_below = self.search_position(pm.toKDL(trans_below, rot), self.err)

                # add to links with position
                connected_above = 0;
                connected_below = 0;
                if node_above is not None:
                    connected_above += self.nodes[node_above][1]
                    self.nodes[node_above][1] += 1
                if node_below is not None:
                    connected_below += self.nodes[node_above][1]
                    self.nodes[node_below][1] += 1
                if node_above is not None and node_below is not None:
                    self.nodes[node_above][1] += connected_below
                    self.nodes[node_below][1] += connected_above
                connected = 1 + connected_above + connected_below;
                links[obj.id] = (pm.toKDL(trans,rot), connected)

            # make sure list of predicates is correct
            if not obj.object_class in self.valid_msg.predicates:
                self.valid_msg.predicates.append(obj.object_class)
            if not obj.object_class in self.valid_msg.assignments:
                self.valid_msg.assignments.append(obj.object_class)
            if not obj.id in self.valid_msg.assignments:
                self.valid_msg.assignments.append(obj.id)
            if not is_det_pred in self.valid_msg.predicates:
                self.valid_msg.predicates.append(is_det_pred)

    def callback(self,msg):
        true_msg = PredicateList()
        true_msg.pheader.source = rospy.get_name()

        self.update_assembly(msg.objects)

        self.pub.publish(true_msg)
        self.vpub.publish(self.valid_msg)

        #print self.valid_msg
        #print true_msg
