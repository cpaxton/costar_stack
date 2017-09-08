import rospy
from costar_structure_msgs.msg import *
from predicator_msgs.msg import *

'''
convert scene structure msg into predicate messages we can query later on
'''

class SceneStructurePublisher:

    valid_msg = None

    def __init__(self):
        self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        self.valid_msg = ValidPredicates()
        self.valid_msg.pheader.source = rospy.get_name()
        self.valid_msg.predicates = ['is_base_structure', 'is_part_of_structure', 'is_top_structure', 'is_free_object']
        self.valid_msg.predicate_length = [1, 1, 1]

        self.sub = rospy.Subscriber('/scene_structure_list', SceneGraph, self.callback)


    def callback(self,msg):
        # rospy.loginfo("Received input msg")
        true_msg = PredicateList()
        true_msg.pheader.source = rospy.get_name()

        for idx, object_id in enumerate(msg.base_objects_id):
            self.valid_msg.assignments.append(object_id)
            structure = msg.structure[idx]

            ps = PredicateStatement()
            ps.predicate = "is_base_structure"
            ps.num_params = 1
            ps.confidence = 1.0
            ps.param_classes.append(object_id)
            ps.params[0] = object_id
            ps.value = True
            true_msg.statements.append(ps)

            if len(structure.nodes_level) > 0:
                ps = PredicateStatement()
                ps.predicate = "is_part_of_structure"
                ps.num_params = 1
                ps.confidence = 1.0
                ps.param_classes.append(object_id)
                ps.params[0] = object_id
                ps.value = len(structure.nodes_level) > 0
                true_msg.statements.append(ps)
            else:
                ps = PredicateStatement()
                ps.predicate = "is_free_object"
                ps.num_params = 1
                ps.confidence = 1.0
                ps.param_classes.append(object_id)
                ps.params[0] = object_id
                ps.value = True
                true_msg.statements.append(ps)

        for structure in msg.structure:
            top_of_structure_idx = len(structure.nodes_level) - 1
            if len(structure.nodes_level) == 0:
                continue
            self.valid_msg.assignments.append(object_id)

            for idx, nodes in enumerate(structure.nodes_level):
                is_top_structure = idx == top_of_structure_idx
                for object_id in nodes.object_names:
                    ps = PredicateStatement()
                    ps.predicate = "is_part_of_structure"
                    ps.num_params = 1
                    ps.confidence = 1.0
                    ps.param_classes.append(object_id)
                    ps.params[0] = object_id
                    ps.value = True
                    true_msg.statements.append(ps)

                    if is_top_structure:
                        ps = PredicateStatement()
                        ps.predicate = "is_top_structure"
                        ps.num_params = 1
                        ps.confidence = 1.0
                        ps.param_classes.append(object_id)
                        ps.params[0] = object_id
                        ps.value = True
                        true_msg.statements.append(ps)
        self.pub.publish(true_msg)
        self.vpub.publish(self.valid_msg)
