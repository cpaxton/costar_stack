### ROS imports
import roslib; roslib.load_manifest('predicator_core')
import rospy

__all__ = ['Predicator']

### Sample Nodes
from core import Predicator
from core import parse_key
from core import get_key
from core import predicate_to_tuple
