cmake_minimum_required(VERSION 2.8.3)
project(object_on_table_segmenter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  PCL
  tf
  tf_conversions
  pcl_ros
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PlaneSegmenter RosPlaneSegmenter
  CATKIN_DEPENDS tf tf_conversions pcl_ros sensor_msgs 
  DEPENDS PCL Boost
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
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(PlaneSegmenter src/plane_segmenter.cpp)

add_library(RosPlaneSegmenter src/ros_plane_segmenter.cpp)

## Declare a C++ executable
add_executable(object_on_table_segmenter src/object_on_table_segmenter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(PlaneSegmenter ${PCL_LIBRARIES})

target_link_libraries(RosPlaneSegmenter PlaneSegmenter ${Boost_LIBRARIES} ${catkin_LIBRARIES})

target_link_libraries(object_on_table_segmenter
   RosPlaneSegmenter
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

install(TARGETS RosPlaneSegmenter PlaneSegmenter
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})
