import rospy
import rosbag
from sensor_msgs.msg import *
from trajectory_msgs.msg import *
import yaml
import curses

class TrajectoryRecorder:
    
    last_msg = None
    
    def __init__(self):
        self.sub = rospy.Subscriber('joint_states',JointState,self.cb)
        self.traj = JointTrajectory()
        self.last_msg = None
            
    def cb(self, msg):
        self.last_msg = msg

    def update(self):

        if not self.last_msg is None:
            pt = JointTrajectoryPoint()
            pt.positions = self.last_msg.position
            pt.velocities = self.last_msg.velocity
            pt.effort = self.last_msg.effort
            self.traj.points.append(pt)
            #print self.traj

            self.last_msg = None

    def save(self, filename):
        with open(filename,'w') as f:
            f.write(yaml.dump(self.traj))
            f.close()
        
        with open(filename,'r') as f:
            print yaml.load(f)
            
    def record(self, filename):
        stdscr = curses.initscr()
        count = 0
        stdscr.addstr(0,0,"Press any key to record.")
        rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown():
                count += 1
                key = stdscr.getch()
                self.update()
                stdscr.addstr(count,0," - %d points"%(len(self.traj.points)))
                rate.sleep()
        except rospy.ROSInterruptException, e:
            stdscr.addstr(count+1,0,"Finishing...")
        finally:
            self.save(filename)
            curses.endwin()

    def load(self, filename):
        with open(filename,'r') as f:
            self.traj = yaml.load(f)
        
