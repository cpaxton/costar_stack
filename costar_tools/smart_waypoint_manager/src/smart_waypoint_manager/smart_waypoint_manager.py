import rospy
from predicator_msgs.srv import *
from predicator_msgs.msg import *
from librarian_msgs.srv import *
from librarian_msgs.msg import *

'''
SmartWaypointManager
This class will create and load smart waypoints from the $LIBRARIAN_HOME/smart_waypoints directory
It also provides ways to use and access these methods
'''
class SmartWaypointManager:


    def __init__(self,world="world"):
        self.get_waypoints_srv = GetWaypointsService(world=world,service=False)
        pass



