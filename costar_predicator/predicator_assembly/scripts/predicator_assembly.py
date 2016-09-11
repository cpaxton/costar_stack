from predicator_assembly import AssemblyPredicator
import rospy

rospy.init_node('predicator_assembly_node')
ap = AssemblyPredicator()

rospy.spin()
