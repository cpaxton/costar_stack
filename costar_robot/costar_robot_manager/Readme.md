# Costar Robot Manager

This code is the CoSTAR robot manager. It provides simple services for instantiating high-level actions on different robots. It's a work in progress, but has all the features needed for the Hannover Messe demos.

## Useful Commands

  - To launch CoSTAR simulation:  
    roslaunch iiwa_gazebo iiwa_gazebo.launch trajectory:=false  
    roslaunch costar_bringup iiwa14_s_model.launch sim:=true start_sim:=false  
    roslaunch instructor_core instructor.launch
    
  - To launch dvrk rviz:  
    roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/PATH/TO/CATKIN_WS/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json
  
  - To launch dvrk console application only:  
    rosrun dvrk_robot dvrk_console_json -j /PATH/TO/CATKIN_WS/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json
    
  - To set up the environment for running driver launch script:  
    roslaunch costar_bringup utilities.launch
    
  - To run the robot launch file to start driver (e.g. psm driver):  
    roslaunch costar_robot_manager simple_psm_driver.launch

## TODO

  - UR5 support using the URX package.
  - Test joint space waypoints and movement.
  - Add other motion primitives and trajectory learning.

