cmake_minimum_required(VERSION 2.8.3)
project(xdot)
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(FILES scripts/dot_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
