### ROS imports
import roslib; roslib.load_manifest('instructor_plugins')
import rospy

### Available Classes for 'from beetree import *'
__all__ = ['NodeActionSampleGUI']
__all__ += ['NodeServiceSampleGUI']
__all__ += ['NodeQuerySampleGUI']
__all__ += ['NodeVariableSampleGUI']
__all__ += ['NodeParamConditionGUI']
__all__ += ['NodeActionTestGUI']
__all__ += ['NodeActionSleepGUI']
__all__ += ['NodeActionWaypointGUI']
# __all__ += ['NodeVariableWaypointGUI']
__all__ += ['NodeActionRelativeWaypointGUI']
__all__ += ['NodeActionGripperOpenGUI']
__all__ += ['NodeActionGripperCloseGUI']
__all__ += ['NodeActionSGripperOpenGUI']
__all__ += ['NodeActionSGripperCloseGUI']
__all__ += ['NodeActionForceZeroGUI']
__all__ += ['NodeActionDetect8020GUI']
__all__ += ['NodeKnowledgeTestGUI']
__all__ += ['NodeActionPedalGUI']
#__all__ += ['NodeActionStopGUI']
__all__ += ['NodeActionPneumaticOpenGUI']
__all__ += ['NodeActionPneumaticCloseGUI']
__all__ += ['NodeActionPneumaticNeutralGUI']
__all__ += ['NodeActionDetectObjectsGUI']
__all__ += ['NodePlanWaypointGUI']
__all__ += ['NodePlanRelativeWaypointGUI']
__all__ += ['NodeActionUpdatePlanningSceneGUI']
__all__ += ['NodeActionRecordDataGUI']
__all__ += ['NodeActionSmartmoveGUI']
__all__ += ['NodePublishMessageGUI']

### Sample Nodes
from sample_nodes import NodeActionSampleGUI
from sample_nodes import NodeQuerySampleGUI
from sample_nodes import NodeVariableSampleGUI
from sample_nodes import NodeServiceSampleGUI
### Instructor Nodes
from instructor_nodes import NodeParamConditionGUI
from instructor_knowledge_test import NodeKnowledgeTestGUI
from instructor_nodes import NodeActionTestGUI
from instructor_nodes import NodeActionSleepGUI
### Other
from instructor_action_waypoint import NodeActionWaypointGUI
from instructor_action_relative_waypoint import NodeActionRelativeWaypointGUI
from instructor_action_gripper import NodeActionGripperOpenGUI
from instructor_action_gripper import NodeActionGripperCloseGUI

from instructor_action_s_gripper import NodeActionSGripperOpenGUI
from instructor_action_s_gripper import NodeActionSGripperCloseGUI
from instructor_action_s_gripper import NodeActionSGripperBasicModeGUI
from instructor_action_s_gripper import NodeActionSGripperScissorModeGUI
from instructor_action_s_gripper import NodeActionSGripperWideModeGUI
from instructor_action_s_gripper import NodeActionSGripperPinchModeGUI

from instructor_update_planning_scene import NodeActionUpdatePlanningSceneGUI

from instructor_detect_objects import NodeActionDetectObjectsGUI
from instructor_publish_message import NodePublishMessageGUI
from record_data import NodeActionRecordDataGUI

from instructor_plan_waypoint import NodePlanWaypointGUI
from instructor_plan_relative_waypoint import NodePlanRelativeWaypointGUI

from instructor_action_detect_8020 import NodeActionDetect8020GUI
from instructor_action_foot_pedal import NodeActionPedalGUI
from instructor_action_stop import NodeActionStopGUI

from instructor_action_smartmove import NodeActionSmartmoveGUI
