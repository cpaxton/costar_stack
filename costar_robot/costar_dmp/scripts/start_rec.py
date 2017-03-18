
import rospy
from std_srvs.srv import Empty as EmptyService

rospy.wait_for_service('costar/dmp/start_rec')
try:
	start_rec = rospy.ServiceProxy('costar/dmp/start_rec',EmptyService)
	resp = start_rec()
except:
	rospy.ServiceException, e:
	print "Service call failed: %s"%e