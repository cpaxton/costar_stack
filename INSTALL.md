# How to Install CoSTAR

Instructions by Baichuan Jiang

Note: CoSTAR installation has only been tested on ROS Indigo on Ubuntu 14.04 machine. For instructions on Indigo installation, please see [here](http://wiki.ros.org/indigo/Installation/Ubuntu). There is a prototype install script available [here](install_indigo.sh) that you can try out as well.

## Prerequisites

To use the CoSTAR system, you will need to install the following software packages:

* Python (tested version 2.7.12)
* Git (tested version 1.9.1)
* ROS (tested ROS Indigo, Ubuntu 14.04)
* Catkin Build Tools
* OpenCV 2.4 nonfree (for CoSTAR perception only)

You can download all the required packages to use CoSTAR with ROS Indigo from the Ubuntu repositories with this command:

```
<<<<<<< HEAD
sudo apt-get install ros-indigo-fcl ros-indigo-soem ros-indigo-moveit-full liburdfdom-headers-dev ros-indigo-control-msgs ros-indigo-gazebo-ros-control ros-indigo-python-orocos-kdl xdot libccd-dev ros-indigo-ros-control ros-indigo-octomap-msgs ros-indigo-object-recognition-msgs ros-indigo-realtime-tools ros-indigo-soem ros-indigo-gazebo-plugins ros-indigo-pcl-ros ros-indigo-socketcan-interface ros-indigo-rqt-gui ros-indigo-rqt-gui-py ros-indigo-position-controllers ros-indigo-robot-state-publisher ros-indigo-joint-state-controller ros-indigo-robot-state-publisher ros-indigo-joint-trajectory-controller ros-indigo-openni2-launch
=======
sudo apt-get install ros-indigo-fcl ros-indigo-soem ros-indigo-moveit-full liburdfdom-headers-dev ros-indigo-control-msgs ros-indigo-gazebo-ros-control ros-indigo-python-orocos-kdl xdot ros-indigo-ros-control ros-indigo-octomap-msgs ros-indigo-object-recognition-msgs ros-indigo-realtime-tools ros-indigo-soem ros-indigo-gazebo-plugins ros-indigo-pcl-ros ros-indigo-socketcan-interface ros-indigo-rqt-gui ros-indigo-rqt-gui-py ros-indigo-position-controllers ros-indigo-robot-state-publisher ros-indigo-joint-state-controller ros-indigo-robot-state-publisher ros-indigo-joint-trajectory-controller 
>>>>>>> 6e1796daba3a13a666e7f5f26cfdc2d12e7675db
```

## Step 1. Get Packages From Git

We suggest that you download all the required packages before building your catkin workspace. First, change directory to the src directory in your catkin workspace. Then download the main CoSTAR stack and its ROS dependencies from GitHub: 

```
cd path/to/your/catkin_ws/src
git clone https://github.com/cpaxton/costar_stack.git  
git clone https://github.com/SalvoVirga/iiwa_stack.git  
git clone https://github.com/ros-industrial/robotiq.git  
git clone https://github.com/jbohren/rqt_dot.git  
git clone https://github.com/sniekum/ar_track_alvar.git  
git clone https://github.com/sniekum/ar_track_alvar_msgs.git  
git clone https://github.com/gt-ros-pkg/hrl-kdl.git  
git clone https://github.com/cpaxton/xdot.git  
git clone https://github.com/ThomasTimm/ur_modern_driver.git
```

## Step 2. Build catkin workspace

Change directory into catkin workspace folder and run:

```
catkin build
```
 
Note: Please use this command to build your catkin workspace instead of `catkin_make`.

***Debugging:***

CoSTAR is distributed as a single large package. This means that 

* For problems with message: "Assertion failed: check for file existence, but filename (RT_LIBRARY-NOTFOUND) unset.  Message: RT Library." Please clean the catkin build folder and rebuild.  
* For problems in relation to predicator_collision, please use the following command:  
`cd path/to/costar_stack/costar_predicator/predicator_collision`  
`touch CATKIN_IGNORE`
3. For problems with message: "Errors: iiwa_hw:make". Please use the following command:
`cd path/to/iiwa_stack/iiwa_hw`  
`touch CATKIN_IGNORE`
4. For problems with message: "[ERROR] [1474482887.121864954, 0.669000000]: Initializing controller 'joint_state_controller' failed". Please try installing the following packages:  
`sudo apt-get install ros-indigo-joint-state-controller`


## Step 3. Run simulation
[Optional] Checkout an example CoSTAR workspace from github into ~/.costar by running:

```
cd && git clone git@github.com:cpaxton/costar_files.git .costar\
```

Now you can run the simulation with following commands. Please remember to run `source ~/catkin_ws/devel/setup.bash` before executing any of these commands, and consider adding this line to ~/.bashrc.

```
roslaunch iiwa_gazebo iiwa_gazebo.launch trajectory:=false  
roslaunch costar_bringup iiwa14_s_model.launch sim:=true start_sim:=false  
```


*If everything shows up, CoSTAR system is then successfully installed. Enjoy!*

The top should say "Robot Mode: Idle." If you installed the sample workspace, open the Menu (lower right) and click Waypoints. Put the robot into Servo mode, highlight some waypoints, and click Servo to Waypoint (the purple button on the Waypoints popup). Not all the waypoints are guaranteed to work for this robot, but you should be able to get the robot to move.

CoSTAR is currently set up to launch our two testbed systems: a KUKA LBR iiwa 14 with a 3-finger Robotiq gripper and a Universal Robots UR5 with a 2-finger Robotiq gripper. We plan to add some funcitonality to support additional platforms.

If you are interested in supporting another platform or run into other issues trying to run this code, please contact Chris Paxton (cpaxton@jhu.edu).
