### ROS imports
import roslib; roslib.load_manifest('predicator_core')
import rospy

__all__ = ['Predicator']

### Sample Nodes
from server import *
