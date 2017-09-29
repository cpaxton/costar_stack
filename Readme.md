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

Interested in contributing? Check out the [development guidelines](docs/development.md)

## Installation

Check out [installation instructions](docs/install.md).

We are working on experimental install scripts:
  - [ROS Indigo/Ubuntu 14.04 LTS](install_indigo.sh)
  - [ROS Kinetic/Ubuntu 16.04 LTS](install_kinetic.sh) -- not yet supported

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
  * [Robot](costar_robot/Readme.md): utilities and services allowing high-level control of the robot and integrating these behaviors into the UI. Contains the `CostarArm` component.
  * Gripper: utilities for integrating different grippers into UI
  * Tools: packages used for data collection, maintaining MoveIt planning scene, and other purposes

## Contact

CoSTAR is maintained by Chris Paxton (cpaxton@jhu.edu).

Other core contributors include:
  * Felix Jonathan
  * Andrew Hundt
