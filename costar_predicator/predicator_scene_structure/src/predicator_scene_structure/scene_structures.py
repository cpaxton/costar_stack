import rospy
from costar_structure_msgs.msg import *
from predicator_msgs.msg import *

'''
convert scene structure msg into predicate messages we can query later on
'''

def generatePredicate(predicate_name,frame_id,value):
    ps = PredicateStatement(predicate = predicate_name, num_params = 1, confidence = 1.0, value = value )
    ps.param_classes.append(frame_id)
    ps.params[0] = frame_id
    return ps

class SceneStructurePublisher:

    valid_msg = None

    def __init__(self):
        self.pub = rospy.Publisher('/predicator/input', PredicateList, queue_size=1000)
        self.vpub = rospy.Publisher('/predicator/valid_input', ValidPredicates, queue_size=1000)
        self.valid_msg = ValidPredicates()
        self.valid_msg.pheader.source = rospy.get_name()
        self.valid_msg.predicates = ['is_base_structure', 'is_part_of_structure', 'is_top_structure', 'is_N_stack_high', 'is_not_N_stack_high', 'is_free_object']
        self.valid_msg.predicate_length = [1, 1, 1]

        self.sub = rospy.Subscriber('/scene_structure_list', SceneGraph, self.callback)

    def callback(self,msg):
        # rospy.loginfo("Received input msg")
        true_msg = PredicateList()
        true_msg.pheader.source = rospy.get_name()

        N_stack = None
        if rospy.has_param("/costar/smartmove/N_stack"):
            N_stack = rospy.get_param("/costar/smartmove/N_stack")
        else:
            N_stack = 99

        for idx, object_id in enumerate(msg.base_objects_id):
            renamed_id = object_id.split('/')[-1]
            self.valid_msg.assignments.append(renamed_id)
            structure = msg.structure[idx]

            true_msg.statements.append(generatePredicate("is_base_structure",renamed_id,True))

            if len(structure.nodes_level) > 0:
                true_msg.statements.append(generatePredicate("is_part_of_structure",renamed_id,len(structure.nodes_level) > 0))
                true_msg.statements.append(generatePredicate("is_N_stack_high",renamed_id,len(structure.nodes_level) == N_stack))
                true_msg.statements.append(generatePredicate("is_not_N_stack_high",renamed_id,len(structure.nodes_level) != N_stack))

            else:
                true_msg.statements.append(generatePredicate("is_free_object",renamed_id,True))

        for structure in msg.structure:
            top_of_structure_idx = len(structure.nodes_level) - 1
            if len(structure.nodes_level) == 0:
                continue
            self.valid_msg.assignments.append(renamed_id)

            for idx, nodes in enumerate(structure.nodes_level):
                is_top_structure = idx == top_of_structure_idx
                for object_id in nodes.object_names:
                    renamed_id = object_id.split('/')[-1]
                    true_msg.statements.append(generatePredicate("is_part_of_structure",renamed_id,True))
                    true_msg.statements.append(generatePredicate("is_N_stack_high",renamed_id,len(structure.nodes_level) == N_stack))
                    true_msg.statements.append(generatePredicate("is_not_N_stack_high",renamed_id,len(structure.nodes_level) != N_stack))

                    if is_top_structure:
                        true_msg.statements.append(generatePredicate("is_top_structure",renamed_id,True))
        self.pub.publish(true_msg)
        self.vpub.publish(self.valid_msg)
