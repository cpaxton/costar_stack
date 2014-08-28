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

  int max_iter = 5000u;
  double step = 0.10;
  double chance = 0.30;
  double skip_distance = 0.75;
  double search_volume = 0.50;

  ros::NodeHandle nh("~");
  nh.param("max_iter", max_iter, int(5000));
  nh.param("step", step, double(0.10));
  nh.param("chance", chance, double(0.30));
  nh.param("skip_distance", skip_distance, double(0.75));
  nh.param("search_volume", search_volume, double(0.50));

  predicator_planning::Planner planner(&pc, (unsigned int)max_iter, step, chance, skip_distance, search_volume);

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
