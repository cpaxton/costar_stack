cmake_minimum_required(VERSION 2.8)
project(xdot)
find_package(catkin REQUIRED)
catkin_project(xdot)

install(FILES scripts/dot_viewer.py
  DESTINATION ${PROJECT_LIBEXEC_DESTINATION})
