#include "predicator.h"
#include "planning_tool.h"

#include <boost/bind/bind.hpp>

namespace predicator_planning {

  Planner::Planner(PredicateContext *_context, unsigned int _max_iter, unsigned int _children, double _step) :
    context(_context), max_iter(_max_iter), children(_children), step(_step)
  {
    ros::NodeHandle nh;
    planServer = nh.advertiseService <
      predicator_planning::PredicatePlan::Request,
      predicator_planning::PredicatePlan::Response>
        ("/predicator/plan",
         boost::bind(&Planner::plan, *this, _1, _2));
  }

  bool Planner::plan(predicator_planning::PredicatePlan::Request &req,
                     predicator_planning::PredicatePlan::Response &res)
  {

    // get starting states
    // these are the states as recorded in the context
    // they will be updated as we go on if this takes a while -- might be bad
    std::vector<RobotState *> starting_states = context->states;

    // find the index of the current robot state
    for (RobotModelPtr &ptr: context->robots) {
      std::cout << ptr->getName() << std::endl;
    }

    // loop over 
    for (unsigned int iter = 0; iter < max_iter; ++iter) {
      // either generate a starting position at random or...
      // step in a direction from a "good" position (as determined by high heuristics)

    }


    return true;
  }


}
