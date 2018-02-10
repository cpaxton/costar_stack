cmake_minimum_required(VERSION 2.8.3)
project(predicator_scene_structure)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  costar_objrec_msgs
  predicator_core
  predicator_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES predicator_detected_objects
#  CATKIN_DEPENDS costar_objrec_msgs predicator_core predicator_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
