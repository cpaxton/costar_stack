
# Startup Guide

## Starting CoSTAR for the LBR iiwa with GRL

Making sure you are connected to the robot:

```
ping 192.170.10.2 # connection to FRI port for joint states
ping 172.31.1.146 # connection to Robotiq 3-finger gripper to send commands
ping 172.31.1.147 # connection to JAVA port on the robot to send commands
```

How to bring this robot up on our own platform:

```
# core features and UI
roslaunch costar_bringup iiwa14_s_model.launch

# object detection and moveit planning scene
roslaunch sp_segmenter SPServerStructureAssembly.launch
```

You also need to launch the GRL KUKA ROS Driver. GRL is the [Generic Robot Library](https://github.com/ahundt/grl), which provides a low-level control interface for the KUKA LBR.

In particular, `costar_bringup` will launch the robot command driver, the gripper command services, the MoveIt services, Predicator, and Librarian. Note that certain tools from our Hannover and other demos are not available open source due to our IP agreement with Ready Robotics, so you will not be able to duplicate our sanding task with this release of CoSTAR.

## Starting CoSTAR in a Simulation

You can test CoSTAR in a Gazebo simulation. The basic launch procedure for a simulation is to run these commands:

```
roslaunch iiwa_gazebo iiwa_gazebo.launch trajectory:=false
roslaunch costar_bringup iiwa14_s_model.launch sim:=true start_sim:=false
```

You can then use the Instructor UI to move the robot around in simulation. If you want to bring up Gazebo and all the drivers at once instead of separately, launch as:

```
roslaunch costar_bringup iiwa14_s_model.launch sim:=true start_sim:=true
```

