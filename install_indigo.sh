#!/usr/bin/env sh

 
cd ~
mkdir -p costar_ws/src
cd ~/costar_ws
source /opt/ros/indigo/setup.bash
sudo apt-get update -qq
sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool python-catkin-tools ros-$ROS_DISTRO-catkin
catkin init
cd ~/costar_ws/src
git clone https://github.com/cpaxton/costar_stack.git  
git clone https://github.com/SalvoVirga/iiwa_stack.git  
git clone https://github.com/ros-industrial/robotiq.git  
git clone https://github.com/jbohren/rqt_dot.git  
git clone https://github.com/cpaxton/dmp.git --branch indigo
git clone https://github.com/sniekum/ar_track_alvar.git --branch indigo-devel 
git clone https://github.com/sniekum/ar_track_alvar_msgs.git
git clone https://github.com/gt-ros-pkg/hrl-kdl.git
git clone https://github.com/xqms/ur_modern_driver.git --branch thread_safety
rosdep install -y --from-paths ./ --ignore-src --rosdistro $ROS_DISTRO
echo "Ignore COSTAR_PERCEPTION until you have installed its dependencies."
touch costar_stack/costar_perception/CATKIN_IGNORE
catkin build --continue
source ../devel/setup.bash
