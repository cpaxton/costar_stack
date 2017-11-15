#!/usr/bin/env sh

 
cd ~
mkdir -p costar_ws/src
cd ~/costar_ws
source /opt/ros/indigo/setup.bash
sudo apt-get update -qq

# Set up your basic deps
sudo apt-get install -y python-catkin-pkg python-rosdep python-wstool python-catkin-tools ros-$ROS_DISTRO-catkin

# Set up PCL 1.7.2
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl 
sudo apt-get update 
sudo apt-get install libpcl-all 

# Set up Opencv-nonfree
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update 
sudo apt-get install libopencv-nonfree-dev

catkin init
cd ~/costar_ws/src
git clone https://github.com/cpaxton/costar_stack.git  
git clone https://github.com/cpaxton/iiwa_stack.git  
git clone https://github.com/ros-industrial/robotiq.git  
git clone https://github.com/jhu-lcsr/ObjRecRANSAC.git  
git clone https://github.com/jbohren/rqt_dot.git  
git clone https://github.com/ros-perception/ar_track_alvar.git
git clone https://github.com/gt-ros-pkg/hrl-kdl.git
git clone https://github.com/xqms/ur_modern_driver.git --branch thread_safety
rosdep install -y --from-paths ./ --ignore-src --rosdistro $ROS_DISTRO
$HOME/costar_ws/src/costar_stack/install_bullet.sh
catkin build --continue
source ../devel/setup.bash
