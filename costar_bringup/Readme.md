# CoSTAR Bringup

This describes tools and systems for reproducing our implementation of the CoSTAR system on a KUKA LBR iiwa 14. The particular combination of the LBR iiwa and the Robotiq 3-finger gripper gives us a powerful range of capabilities.

## Hand-Eye Calibration

We provide a model-based approach for hand-eye calibration.

We used the Robotiq kit for adapting a 3-finger gripper to a touch IO flange. 

Image of marker placement in the robot:

[Robot image](https://git.lcsr.jhu.edu/cpaxton3/costar_stack/raw/master/costar_bringup/doc/marker_in_hand_robot.jpg)

Image of marker placement in rviz:

[RVIZ image](https://git.lcsr.jhu.edu/cpaxton3/costar_stack/raw/master/costar_bringup/doc/marker_in_hand_rviz.png)

## Simulation

You can easily run the simulation with:

```
roslaunch costar_bringup iiwa_s_model.launch sim:=true
```

It may be helpful to launch different components separately:
```
roslaunch iiwa_gazebo iiwa_gazebo.launch trajectory:=false # simulation
roslaunch costar_bringup sim:=true start_sim:=false
```

## RVIZ

To show a visualization:
```
roslaunch costar_bringup rviz.launch
```

The yellow grid is offset from the camera frame.
