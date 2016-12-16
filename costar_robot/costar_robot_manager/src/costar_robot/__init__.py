import commands

def rospack_exists(module_name, module_exist_list, module_missing_list):
	s = commands.getstatusoutput('rospack find ' + module_name)
	if s[0] == 0:
		module_exist_list.append(module_name)
		return True
	else:
		module_missing_list.append(module_name)
		return False

installed_robot_driver = list()
missing_robot_driver = list()

from planning import SimplePlanning

from costar_arm import CostarArm

# check if robot ros module exist/not
if rospack_exists('ur_modern_driver',installed_robot_driver,missing_robot_driver):
	from ur_driver import CostarUR5Driver
if rospack_exists('iiwa_ros',installed_robot_driver,missing_robot_driver):
	from iiwa_driver import CostarIIWADriver
if rospack_exists('dvrk_robot',installed_robot_driver,missing_robot_driver):
	from psm_driver import CostarPSMDriver

from waypoint_manager import WaypointManager

# define modes
from planning import ModeJoints
from planning import ModeCart

if len(installed_robot_driver) > 0:
	print 'Installed robot driver: ', installed_robot_driver
	print 'Missing robot driver: ', missing_robot_driver
else:
	print 'Missing robot driver: ', missing_robot_driver
	raise ImportError('No ros robot driver is installed')