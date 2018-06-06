cmake_minimum_required(VERSION 2.8.3)
project(color_nn_segmenter)

# Enable C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support, or our tests failed to detect it correctly. Please use a different C++ compiler or report this problem to the developers.")
endif()

if(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  ADD_DEFINITIONS(-fPIC)
endif(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")

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
  object_on_table_segmenter
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
