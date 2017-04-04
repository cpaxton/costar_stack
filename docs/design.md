
# Architecture Design

CoSTAR takes a page from most ROS system design.

### Arm

This component is extended to implement functionality for specific robots:
  - IIWA
  - UR5
  - DVRK

The `Arm` component has a few associated helper classes, created to isolate complex functionality:
  - `SimplePlanning`, which wraps a MoveIt trajectory interface and creates a few other things.
  - `InverseKinematicsUR5`, which as its name implies does inverse kinemetics for our UR5. It is a closed-form IK solver, and is fairly specific to our robot as a result.

### Gripper

The gripper is configured so that we can set it into different modes, depending on what gripper is currently available.


## Predicator


## Operations

## Instructor

Instructor is our BT-based UI. It is closely tied with [beetree](external/beetree/README.md).
