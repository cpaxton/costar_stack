# Costar Robot Manager

This code is the CoSTAR robot manager. It provides simple services for instantiating high-level actions on different robots. It's a work in progress, but has all the features needed for the Hannover Messe demos.

## Dependencies
  - To use a dvrk, install ros package from https://github.com/jhu-dvrk/dvrk-ros
  - To use a ur5, install ros package from https://github.com/ThomasTimm/ur_modern_driver
  - To use a iiwa, install ros package from https://github.com/SalvoVirga/iiwa_stack. OPTIONAL: It is possible to use grl too in addition to iiwa-stack for faster  communication to the iiwa. https://github.com/ahundt/grl

## Useful Commands

  - To launch CoSTAR simulation:  
    roslaunch iiwa_gazebo iiwa_gazebo.launch trajectory:=false  
    roslaunch costar_bringup iiwa14_s_model.launch sim:=true start_sim:=false  
    roslaunch instructor_core instructor.launch
    
  - To launch dvrk rviz:  
    roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM2 config:=/PATH/TO/CATKIN_WS/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM2_KIN_SIMULATED.json
  
  - To launch dvrk console application only:  
    rosrun dvrk_robot dvrk_console_json -j /PATH/TO/CATKIN_WS/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM2_KIN_SIMULATED.json
    
  - To set up the environment for running driver launch script:  
    roslaunch costar_bringup utilities.launch
    
  - To run the robot launch file to start driver (e.g. psm driver):  
    roslaunch costar_robot_manager simple_psm_driver.launch

## TODO

  - UR5 support using the URX package.
  - Test joint space waypoints and movement.
  - Add other motion primitives and trajectory learning.

