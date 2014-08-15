// ROS
#include <ros/ros.h>

// for debugging
#include <iostream>

// predicator stuff
#include "predicator.h"

int main(int argc, char **argv) {

  ros::init(argc, argv, "predicator_planning_node");

  predicator_planning::PredicateContext pc(true);

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
