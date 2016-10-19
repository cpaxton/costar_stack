# How to Install CoSTAR

Instructions by Baichuan Jiang

Note: CoSTAR installation has only been tested on ROS Indigo on Ubuntu 14.04 machine. For instructions on Indigo installation, please see [here](http://wiki.ros.org/indigo/Installation/Ubuntu). Some packages, e.g., snap_instructor (user interface) are not open source yet. For JHU users, please go [here](https://git.lcsr.jhu.edu/) and use JHED to login and download. For other users, please contact <cpaxton3@jhu.edu> for help. 


## Prerequisites
To successfully install the CoSTAR system, some software are demanded to be installed as prerequisite: 

* Python (tested version 2.7.12)
* Git (tested version 1.9.1)


## Step 1. Get the Packages

It is suggested to download all required packages before building your catkin workspace. First, change directory to the src directory in your catkin workspace. 

`cd path/to/your/catkin_ws/src`

Download interface package snap_instructor from [LCSR git website](https://git.lcsr.jhu.edu/), or (JHU users) use the following command to download (username and password requested are the same as JHED account):

```
git clone https://git.lcsr.jhu.edu/snap/snap_instructor.git  
git clone https://git.lcsr.jhu.edu/snap/ready_air.git  
```

Download the CoSTAR main package and ROS-dependencies from GitHub: 

```
git clone https://github.com/cpaxton/costar_stack.git  
git clone https://github.com/SalvoVirga/iiwa_stack.git  
git clone https://github.com/ros-industrial/robotiq.git  
git clone https://github.com/futureneer/beetree.git  
git clone https://github.com/jbohren/rqt_dot.git  
git clone https://github.com/sniekum/ar_track_alvar.git  
git clone https://github.com/sniekum/ar_track_alvar_msgs.git  
git clone https://github.com/gt-ros-pkg/hrl-kdl.git  
git clone https://github.com/cpaxton/xdot.git  
git clone https://github.com/ThomasTimm/ur_modern_driver.git
```

Download the required packages from Ubuntu repositories: 

```
sudo apt-get install ros-indigo-fcl  
sudo apt-get install ros-indigo-soem  
sudo apt-get install ros-indigo-moveit-full  
sudo apt-get install liburdfdom-headers-dev  
sudo apt-get install ros-indigo-control-msgs  
sudo apt-get install ros-indigo-gazebo-ros-control  
sudo apt-get install ros-indigo-python-orocos-kdl  
sudo apt-get install ros-indigo-razer-hydra  
sudo apt-get install xdot  
sudo apt-get install libccd-dev  
sudo apt-get install ros-indigo-ros-control  
sudo apt-get install ros-indigo-octomap-msgs  
sudo apt-get install ros-indigo-object-recognition-msgs  
sudo apt-get install ros-indigo-realtime-tools  
sudo apt-get install ros-indigo-soem  
```

If you are using the UR5, download the required package from PyPI:

```
sudo pip install urx
```


## Step 2. Build catkin workspace

Change directory into catkin workspace folder and run:

```
catkin build
```
 
Note: Please use this command to build your catkin workspace instead of `catkin_make`.

***Debugging:***

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
FIrst, download and put the costar_files in ~/.costar using the following commands:

`cd ~`  
`git clone https://git.lcsr.jhu.edu/cpaxton3/costar_files.git`  
`mv costar_files/ .costar`

Now you can run the simulation with following command (Please remember to run `roscore` first. If problem occurs, please run `source devel/setup.bash` or add this line to ~/.bashrc. ):

```
roslaunch iiwa_gazebo iiwa_gazebo.launch trajectory:=false  
roslaunch costar_bringup iiwa14_s_model.launch sim:=true start_sim:=false  
roslaunch instructor_core instructor.launch
```


*If everything shows up, CoSTAR system is then successfully installed. Enjoy!*

