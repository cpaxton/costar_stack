# CoSTAR-dVRK

***A tutorial on enabling CoSTAR to work with dVRK***

This file contains instructions on deploying custom dVRK files and guidelines for system workflow. The aim of this work is to establish a human-robot-collaboration research platform in robotic surgery, based on CoSTAR, a Behavior-Tree based framework for end-user instruction of industrial robots. A more detailed description can be found in our [IROS 2017 Workshop paper](https://smarts.lcsr.jhu.edu/wp-content/uploads/2017/04/costar_in_surgery.pdf). 

[![Video demonstration](https://img.youtube.com/vi/RqQNLZuuRUE/0.jpg)](https://www.youtube.com/watch?v=RqQNLZuuRUE)

## Installation and dVRK deployment

### Start dVRK only

The [da Vinci Research Kit](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki) (dVRK) is an open-source research platform for medical robotics research. The following steps can be skipped if dVRK is already in a working status.

- Disconnect firewire
- Turn on the controllers
- Connect firewire again 

Some general commands to work dVRK include the following:

* Command line input: ls -al /dev/fw*
  * Should return a list of FPGA-QLA information
* Command line input: qladisp 8 9
  * when nothing turns up, try source ~/catkin_ws/devel_release/setup.bash

Now, to start dVRK, follow the instructions below:

* **Command line input: qlacloserelays**
* **Get roscore running**
* **To run single arm without RViz and teleoperation, run**: 
  * rosrun dvrk_robot dvrk_console_json -j /path/to/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-dVRK/console-PSM2.json
  * Use ipython script to drive the arm, such as: 
    * import dvrk
    * p = dvrk.psm('PSM1')
    * import PyKDL
    * p.home()
    * p.insert_tool(0.1)
    * p.dmove(PyKDL.Vector(0.05, 0.0, 0.0))
    * p.move_joint_one(0.05, 0)
    * p.open_jaw()
  * **REMEMBER!** EVERYTIME after stopping the program (changing config files), if you want to start the arms again using other .json or .launch files, you should run qlacloserelays AGAIN. 
* **To run PSM1-MTMR teleoperation, follow the steps below**: 
  * roslaunch dvrk_robot dvrk_master_slave_rviz.launch master:=MTMR slave:=PSM1 config:=/path/to/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/jhu-dVRK/console-MTMR-PSM1-Teleop.json
  * Note: should place PSM1 in a "good" start position, i.e., pose not too strange. 
  * In the dVRK console, click "Home". Wait a second, you should see PSM starting to get powered and MTMR move to home position. Both in real world and in RViz. 
  * Then in dVRK console, click "Start", the teleoperation will be started. Hold MTMR (master teleoperator, right-hand side), pressed COAG foot pedal, you should be able to move the PSM1 teleoperatively. 
  * Click "Stop", then "Off", if you want to stop. 

### Custom file deployment

To facilitate the process of setting up CoSTAR-dVRK, you can just replace or add the following files to dVRK repositories ([cisst-saw](https://github.com/jhu-cisst/cisst-saw) and [dvrk-ros](https://github.com/jhu-dvrk/dvrk-ros)). The files are available in the config_files folder. 

* **In repository: dvrk-ros/**
  * REPLACE: dvrk_robot/launch/dvrk_master_slave_rviz.launch (The CoSTAR part does not work with tf_static, set it to false to use tf frame chain.) 
  * REPLACE: dvrk_robot/launch/dvrk_arm_rviz.launch (using an interactive marker in Rviz) 
  * REPLACE: dvrk_model/rviz_config/MTMR_PSM1.rviz (this includes an interactive marker in Rviz, for dvrk_teleop.launch)
  * ADD: dvrk_model/rviz_config/PSM1_with_marker.rviz (this includes an interactive marker in Rviz) 
* **In repository: cisst-saw/sawIntuitiveResearchKit/**
  * REPLACE: share/console-MTMR-PSM1-Teleop.json (so that the coordinates are defined with respect to RCM point, you can also choose to use the original json config file and publish an identity transformation at /set_base_frame ROS topic, which could also set the reference frame to RCM point) 
  * REPLACE: share/console-MTMR-PSM1-MTML-PSM2-Teleop.json (the same reason as above) 

## User Instruction

### Launch options

There are different launch files for different system configurations. The names of the launch files are self-explanatory. 

* Do simulated single arm manipulation: roslaunch costar_bringup dvrk_psm.launch 
* Do real world single arm manipulation: roslaunch costar_bringup dvrk_real.launch
* Do single arm teleoperation: roslaunch costar_bringup dvrk_teleop.launch
* Do two arm teleoperation (full mode): roslaunch costar_bringup dvrk_teleop_two_arms.launch 

### System workflow for two-arm operation

* Run: roslaunch costar_bringup dvrk_teleop_two_arms.launch
* Wait until all windows are loaded successfully.
* In dVRK console, press "home" button. (For version 1.6.0, press "Power ON", then "Home")
* In dVRK console, press "start" teleoperation button. 
* In CoSTAR Instructor interface, press "TEACH" button on the top. 
* Move to the robot master console, get hold of the master operators, press COAG foot pedal and start      tele-manipulation. 
* When feeling good to record a waypoint, press CLUTCH foot pedal (in the mean time you can move the      master operator to re-adjust the pose for long-range movement). 
* In CoSTAR Instructor interface, click Menu -> Waypoints, you can probably find the time-stamped      waypoint recorded. (You have to click some button on the waypoint window to refresh the list if not seeing the new waypoint. )
* After collecting all waypoints, stop operating, and in CoSTAR Instructor interface, click "SERVO" button on the top. 
* You can build a behavior tree as described in our workshop paper, with the waypoints recorded.
* In CoSTAR Interface, click "EXECUTE" button, then move to the robot master console. 
* Press "COAG" foot pedal to start manipulation, when you feel you need the robot to do its job, press "CAMERA" foot pedal. 
* The video demonstration can be found above. 

### Interactive marker and DMP

We also created an interactive marker in RViz for better instructing the dVRK (i.e., not through master console). DMP stands for dynamic movement primitives, which is an experimental feature for automating trajectory generation. Please refer to the following video demonstrations for more details.

* Interactive marker: [video](https://drive.google.com/open?id=0B_yGdvqsvxSIYUdtTHNPU0hiYjg) 
* DMP: [video](https://drive.google.com/open?id=0B_yGdvqsvxSITDhCRWdZYVlicEk) 

## Contact

For more questions regarding CoSTAR-dVRK, please contact Baichuan Jiang (baichuan@jhu.edu). 