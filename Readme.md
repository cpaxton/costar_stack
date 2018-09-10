# CoSTAR 

***Collaborative System for Task Automation and Recognition***

[![Build Status](https://travis-ci.org/cpaxton/costar_stack.svg?branch=master)](https://travis-ci.org/cpaxton/costar_stack)

CoSTAR is an end-user interface for authoring robot task plans developed at Johns Hopkins University. It includes integrated perception and planning capabilities, plus a Behavior Tree based user interface.

[![CoSTAR Expert User Demonstration](https://img.youtube.com/vi/TPXcWU-5qfM/0.jpg)](https://youtu.be/TPXcWU-5qfM "CoSTAR Expert User Demonstration")

Our goal is to build a system which facilitates end-user instruction of robots to solve a variety of different problems. CoSTAR allows users to program robots to perform complex tasks such as sorting, assembly, and more. Tasks are represented as Behavior Trees. For videos of our system in action, you can check out the [CoSTAR YouTube Channel](https://www.youtube.com/playlist?list=PLF86ez-NVmyEDgpmwpnpM6LyNwtkiWxAf).

To take full advantage of CoSTAR, you will need an RGB-D camera and supported hardware:
  - a KUKA LBR iiwa or Universal Robots UR5
  - a Robotiq 3-finger gripper or 2-finger gripper
  - a [Da Vinci Research Kit](https://github.com/jhu-dvrk/dvrk-ros) -- in development.

This is a project by members of the JHU Laboratory for Computational Sensing and Robotics, namely Chris Paxton, Kel Guerin, Andrew Hundt, and Felix Jonathan. If you find this code useful, please cite:
```
@article{paxton2017costar,
  title={Co{STAR}: Instructing Collaborative Robots with Behavior Trees and Vision},
  author={Paxton, Chris and Hundt, Andrew and Jonathan, Felix and Guerin, Kelleher and Hager, Gregory D},
  journal={Robotics and Automation (ICRA), 2017 IEEE International Conference on},
  note={Available as arXiv preprint arXiv:1611.06145},
  year={2017}
}
```

Interested in contributing? Check out the [development guidelines](docs/development.md), which are a work in progress.

## Installation

Check out [installation instructions](docs/install.md).

We are working on experimental install scripts:
  - [ROS Indigo/Ubuntu 14.04 LTS](install_indigo.sh)
  - [ROS Kinetic/Ubuntu 16.04 LTS](install_kinetic.sh)

## Tests

Run the IIWA test script:
```
rosrun costar_bringup iiwa_test.py
```

It will start gazebo and move the arm to a new position. If this test passes, CoSTAR is set up right.

There is a more detailed [startup guide](docs/startup.md).

## CoSTAR Packages

  * [Bringup](costar_bringup/Readme.md): launch files, RVIZ configurations, et cetera
  * [Component](costar_component/Readme.md): generic tools for non-specific CoSTAR components.
  * [Instructor](costar_instructor/Readme.md): user interface
  * [Librarian](costar_librarian/Readme.md): file management
  * [Predicator](costar_predicator/Readme.md): robot knowledge management
  * [Perception](costar_perception/Readme.md): semantic segmentation and object detection via [SP Segmenter](https://github.com/jhu-lcsr/sp_segmenter)
    * [Training the color segmenter](docs/collect_data.md): a quick overview of how you can train the color segmenter.
    * [Sequential Scene Parsing](costar_perception/sequential_scene_parsing): use bulletphysics and previous scene to improve the pose accuracy of objects. <br> Paper: [Temporal and Physical Reasoning for Perception-Based Robotic Manipulation](https://arxiv.org/abs/1710.03948)
  * [Robot](costar_robot/Readme.md): utilities and services allowing high-level control of the robot and integrating these behaviors into the UI. Contains the `CostarArm` component.
  * Gripper: utilities for integrating different grippers into UI
 
For more information on how to collect data for the "block stacking" task, check out [the block stacking data collection notes](docs/collect_data.md)

### [costar_tools](costar_tools/Readme.md)

- data collection with an rgbd camera
    - [Object on Table Segmenter](costar_tools/object_on_table_segmenter/README.md): Utility for dataset collection with depth cameras. It provides a simple process for defining regions of a scene that are table, object, robot etc and generates files accordingly.
- Locating AR Tag markers with a known shape in an image
    - [alvar_data_collection](costar_tools/alvar_data_collection/README.md): utilities to define the black and white printed AR tags we use for [ar_track_alvar](https://github.com/ros-perception/ar_track_alvar), which define the positions and orientations of objects in space (see the video above).

More minor utilities:

- making changes to robot path planning scenes with [MoveIt](https://moveit.ros.org/)
    - [moveit_collision_environment](costar_tools/moveit_collision_environment/README.md): Publishes a [MoveIt](https://moveit.ros.org/) planning scene that contains the collision object and table that is detected via TF frames defined for those objects.
    - To visualize the current configuration run `roslaunch ur5_moveit_config moveit_rviz.launch`
- handling symmetrical objects
  - [object_symmetry_republisher](costar_tools/object_symmetry_republisher/Readme.md): Takes in object information from perception (for example, sp_segmenter) and outputs poses for possible symmetries of that object.
    
### Sister repositories

These are repositories that have been integrated with costar_stack, though not necessarily required depending on your setup.
Also see the [.travis.yml](.travis.yml) in this repository for additional repositories that have been used with costar_stack.

- [https://github.com/ahundt/robotics_setup/blob/master/costar.sh](robotics_setup) - automated installation script
- [github.com/jhu-lcsr-forks/universal_robot](https://github.com/jhu-lcsr-forks/universal_robot) UR5 robot model
- [handeye_calib_camodocal](https://github.com/jhu-lcsr/handeye_calib_camodocal): Used to determine where the camera and robot base are when they are moved around.
- https://github.com/cpaxton/iiwa_stack.git kuka iiwa robot model
- https://github.com/ros-industrial/robotiq.git  
- https://github.com/jhu-lcsr/ObjRecRANSAC.git  
- https://github.com/jbohren/rqt_dot.git  
- https://github.com/ros-perception/ar_track_alvar.git
- https://github.com/gt-ros-pkg/hrl-kdl.git
- https://github.com/xqms/ur_modern_driver.git --branch thread_safety


## Contact

CoSTAR is maintained by Chris Paxton (cpaxton@jhu.edu).

Other core contributors include:
  * Felix Jonathan
  * Andrew Hundt
