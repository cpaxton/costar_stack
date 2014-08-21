#ifndef _PLANNING_TOOL
#define _PLANNING_TOOL

// include ROS headers
#include <ros/ros.h>

#include <predicator_msgs/PredicateStatement.h>
#include <predicator_planning/PredicatePlan.h>

#include "predicator.h"

namespace predicator_planning {


  struct Planner {

    // context contains information about the world and will produce new predicates
    PredicateContext * context;
    ros::ServiceServer planServer;

    unsigned int max_iter; // maximum iterations to attempt to find destination
    unsigned int children; // number of children to create at each step
    double step; // distance to move

    Planner(PredicateContext *context, unsigned int max_iter, unsigned int children, double step);

    bool plan(predicator_planning::PredicatePlan::Request &req,
              predicator_planning::PredicatePlan::Response &res);

  };


}

#endif
