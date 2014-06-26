### ROS imports
import roslib; roslib.load_manifest('predicator_plugins')
import rospy

### Available Classes for 'from beetree import *'
__all__ = ['NodeQueryClosestObjectGUI']
__all__ += ['NodeConditionTestPredicateGUI']

### Sample Nodes
#from sample_nodes import NodeActionSampleGUI
from sample_nodes import NodeQueryClosestObjectGUI
#from sample_nodes import NodeServiceSampleGUI
### Instructor Nodes
from condition_nodes import NodeConditionTestPredicateGUI
