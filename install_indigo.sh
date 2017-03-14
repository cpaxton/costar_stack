#!/usr/bin/env sh

sudo apt-get install python-catkin-tools ros-indigo-fcl ros-indigo-soem ros-indigo-moveit-full ros-indigo-universal-robots ros-indigo-ur-msgs liburdfdom-headers-dev ros-indigo-control-msgs ros-indigo-gazebo-ros-control ros-indigo-python-orocos-kdl xdot ros-indigo-ros-control ros-indigo-octomap-msgs ros-indigo-gazebo-plugins ros-indigo-pcl-ros ros-indigo-socketcan-interface ros-indigo-rqt-gui ros-indigo-object-recognition-msgs ros-indigo-realtime-tools ros-indigo-soem ros-indigo-position-controllers ros-indigo-robot-state-publisher ros-indigo-joint-state-controller
 
cd ~
mkdir -p costar_ws/src
cd ~/costar_ws
source /opt/ros/indigo/setup.bash
catkin init
cd ~/costar_ws/src
git clone https://github.com/cpaxton/costar_stack.git  
git clone https://github.com/SalvoVirga/iiwa_stack.git  
git clone https://github.com/ros-industrial/robotiq.git  
git clone https://github.com/jbohren/rqt_dot.git  
git clone https://github.com/sniekum/ar_track_alvar.git  
git clone https://github.com/sniekum/ar_track_alvar_msgs.git  
git clone https://github.com/gt-ros-pkg/hrl-kdl.git  
git clone https://github.com/cpaxton/xdot.git  
git clone https://github.com/ThomasTimm/ur_modern_driver.git

echo "Ignore COSTAR_PERCEPTION until you have installed its dependencies."
touch costar_stack/costar_perception/CATKIN_IGNORE
catkin build --continue
source ../devel/setup.bash
