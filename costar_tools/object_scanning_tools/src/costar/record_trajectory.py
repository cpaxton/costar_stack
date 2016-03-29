import rospy
import rosbag
from sensor_msgs.msg import *
from trajectory_msgs.msg import *

class TrajectoryRecorder:
    
    
    def __init__(self):
        self.sub = rospy.Subscriber('joint_states',JointState,self.cb)
        self.traj = JointTrajectory()
            
    def cb(self, msg):
        pt = JointTrajectoryPoint()
        pt.positions = msg.position
        pt.velocities = msg.velocity
        pt.effort = msg.effort
        self.traj.points.append(pt)
        print self.traj

    def save(self, filename):
        pass
