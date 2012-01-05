cmake_minimum_required(VERSION 2.8)
project(xdot)
find_package(catkin)

install(FILES scripts/dot_viewer.py
  DESTINATION share/xdot/bin
  )
