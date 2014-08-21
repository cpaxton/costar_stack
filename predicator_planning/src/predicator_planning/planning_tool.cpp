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


    for (unsigned int iter = 0; iter < max_iter; ++iter) {

    }

  }


}
