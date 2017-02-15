#!/usr/bin/env sh

sudo apt-get install python-catkin-tools ros-indigo-fcl ros-indigo-soem ros-indigo-moveit-full liburdfdom-headers-dev ros-indigo-control-msgs ros-indigo-gazebo-ros-control ros-indigo-python-orocos-kdl xdot libccd-dev ros-indigo-ros-control ros-indigo-octomap-msgs ros-indigo-object-recognition-msgs ros-indigo-realtime-tools ros-indigo-soem  
cd $HOME
mkdir -r costar_ws/src
cd costar_ws
catkin init
cd src
git clone https://github.com/cpaxton/costar_stack.git  
git clone https://github.com/SalvoVirga/iiwa_stack.git  
git clone https://github.com/ros-industrial/robotiq.git  
git clone https://github.com/jbohren/rqt_dot.git  
git clone https://github.com/sniekum/ar_track_alvar.git  
git clone https://github.com/sniekum/ar_track_alvar_msgs.git  
git clone https://github.com/gt-ros-pkg/hrl-kdl.git  
git clone https://github.com/cpaxton/xdot.git  
git clone https://github.com/ThomasTimm/ur_modern_driver.git

source ../devel/setup.bash
echo "Ignore COSTAR_PERCEPTION until you have installed its dependencies."
touch costar_stack/costar_perception/CATKIN_IGNORE
catkin build --continue