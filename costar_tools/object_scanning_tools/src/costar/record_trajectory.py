import rospy
import rosbag
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
import yaml

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
        #print self.traj
        print " - %d points"%(len(self.traj.points))

    def save(self, filename):
        with open(filename,'w') as f:
            f.write(yaml.dump(self.traj))
            f.close()
        
        with open(filename,'r') as f:
            print yaml.load(f)
            
