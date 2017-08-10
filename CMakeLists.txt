cmake_minimum_required(VERSION 2.8.3)
project(color_nn_segmenter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  PCL
  OpenCV
  pcl_conversions
  std_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES color_nn_segmenter
#  CATKIN_DEPENDS message_generation message_runtime pcl_conversions pcl_ros sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ library

add_library(ColorSegmenter src/color_nn_segmenter.cpp)

## Declare a C++ executable
add_executable(pcl_color_segmenter src/main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above

target_link_libraries(
 ColorSegmenter ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${BOOST_LIBRARIES}
)

target_link_libraries(
 pcl_color_segmenter ColorSegmenter ${catkin_LIBRARIES}
)
