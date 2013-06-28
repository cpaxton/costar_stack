# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(xdot)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
else()
  # Load catkin and all dependencies required for this package
  # TODO: remove all from COMPONENTS that are not catkin packages.
  find_package(catkin REQUIRED COMPONENTS)

  # TODO: fill in what other packages will need to use this package
  ## DEPENDS: system dependencies of this project that dependent projects also need
  ## CATKIN_DEPENDS: catkin_packages dependent projects also need
  ## INCLUDE_DIRS: 
  ## LIBRARIES: libraries you create in this project that dependent projects also need
  catkin_package(
      DEPENDS wxpython graphviz
      CATKIN_DEPENDS # TODO
      INCLUDE_DIRS # TODO include
      LIBRARIES # TODO
  )

  catkin_python_setup()

  install(PROGRAMS scripts/dot_viewer.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
