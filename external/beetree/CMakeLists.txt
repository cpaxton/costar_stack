cmake_minimum_required(VERSION 2.8.3)
project(beetree)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_dot
)

catkin_python_setup()
catkin_package()

