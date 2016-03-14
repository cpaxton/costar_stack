# CoSTAR 

***Collaborative System for Task Automation and Recognition***

These are the tools and utilities we created to get the CoSTAR project up and off the ground.

## Packages

  * Bringup: launch tools
  * Librarian: file management
  * Predicator: robot knowledge management
  * Gripper: utilities for integrating different grippers into UI
  * Tools: packages used for different aspects of the UI

## Bringup

## Gripper

  * ***Simple S Model Server***: This is a part of our CoSTAR UI -- cross platform robot graphical user interface for teaching complex behaviors to industrial robots. This wraps a couple simple 3-finger gripper commands, which we can then expose as UI behaviors.=

## Librarian

Provide simple interface for managing files used by CoSTAR programs.

### File System

- **Types:** high level categories; use these to group definitions of different items/variables used in a CoSTAR workspace.
- **Items:** items are represented as files, and can be saved/loaded as ROS parameters.

### Using Librarian

#### Launch Files

The `librarian_bringup` package contains launch files for Librarian. As such, you can launch the librarian core easily with:

```
roslaunch librarian_bringup core.launch
```

To change the location of the folder Librarian stores and reads files from, simply change the `librarian_root` parameter:

```
roslaunch librarian_bringup core.launch librarian_root:=~/.costar/
```

#### Running Librarian

Start `librarian_core` with the directory you want to use for saving/loading files.

#### Provided Services

### Troubleshooting


## Predicator

## Contact

CoSTAR is maintained by Chris Paxton (cpaxton3@jhu.edu)
