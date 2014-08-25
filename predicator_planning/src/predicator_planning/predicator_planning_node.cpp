// ROS
#include <ros/ros.h>

// for debugging
#include <iostream>

// predicator stuff
#include "predicator.h"
#include "planning_tool.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "predicator_planning_node");

  predicator_planning::PredicateContext pc(true);

  unsigned int max_iter = 1000u;
  unsigned int children = 10u;
  double step = 0.02;

  predicator_planning::Planner planner(&pc, max_iter, children, step, 1.0);

  // define spin rate
  ros::Rate rate(30);

  // start main loop
  while(ros::ok()) {
    ros::spinOnce();
    pc.tick();
    rate.sleep();
  }

  pc.cleanup();
}
