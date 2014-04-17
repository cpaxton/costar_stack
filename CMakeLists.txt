cmake_minimum_required(VERSION 2.8.3)
project(xdot)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED)

catkin_package(
  DEPENDS wxpython graphviz
)

catkin_python_setup()

install(PROGRAMS scripts/dot_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
