#include "predicator.h"
#include "planning_tool.h"

// joint model group -- which joints are we solving for?
#include <moveit/robot_model/joint_model_group.h>

// standard libraries for random
#include <cstdlib>

namespace predicator_planning {

  Planner::Planner(PredicateContext *_context, unsigned int _max_iter, unsigned int _children, double _step, double _chance) :
    context(_context), max_iter(_max_iter), children(_children), step(_step), chance(_chance)
  {
    ros::NodeHandle nh;
    planServer = nh.advertiseService("predicator/plan", &Planner::plan, this);
    nh.param("verbosity", verbosity, 0);
  }

  // default constructor
  Planner::SearchPose::SearchPose() : state(NULL), count_best(0), parent(NULL), child(NULL) {}

  // initialize parents, variables
  Planner::SearchPose::SearchPose(std::deque<SearchPose *> &search, RobotState *_state) :
    count_best(0), parent(NULL), child(NULL), state(_state)
  {
    double shortest_dist = 999999.;
    for(unsigned int i = 0; i < search.size(); ++i) {
      // compare distance
      double dist = search[i]->state->distance(*state);
      if (parent == NULL || dist < shortest_dist) {
        parent = search[i];
      }
    }
  }

  // update with information from the context
  void Planner::SearchPose::update(PredicatePlan::Request &req, PredicateContext *context) {

  }

  bool Planner::plan(predicator_planning::PredicatePlan::Request &req,
                     predicator_planning::PredicatePlan::Response &res)
  {

    // get starting states
    // these are the states as recorded in the context
    // they will be updated as we go on if this takes a while -- might be bad
    std::vector<RobotState *> starting_states = context->states;

    // this is the list of states we are searching in
    std::deque<SearchPose *> search;

    // find the index of the current robot state
    unsigned int idx = 0;
    for (RobotModelPtr &ptr: context->robots) {
      if (ptr->getName().compare(req.robot) == 0) {
        std::cout << "Robot \"" << ptr->getName() << "\" found at index=" << idx << std::endl;
        break;
      }
      ++idx;
    }

    // get the robots' group
    moveit::core::JointModelGroup *group = NULL;
    if (context->robots[idx]->hasJointModelGroup(req.group)) {
        group = context->robots[idx]->getJointModelGroup(req.group);
    } else {
      ROS_ERROR("Unable to get group %s for robot %s!", req.group.c_str(), req.robot.c_str());
      return false;
    }

    RobotState **next = new RobotState *[children];
    for (unsigned int i = 0; i < children; ++i) {
      next[i] = new RobotState(context->robots[idx]);
      next[i]->setToRandomPositionsNearBy(group, *starting_states[idx], step);
    }

    // loop over 
    for (unsigned int iter = 0; iter < max_iter; ++iter) {
      // either generate a starting position at random or...
      // step in a direction from a "good" position (as determined by high heuristics)
      
      double choose_op = (double)rand() / (double)RAND_MAX;
      if(choose_op > chance) {
        // spawn children from the thing with the highest heuristic
 
        for (unsigned int c = 0; c < children; ++c) {

          RobotState *rs = new RobotState(context->robots[idx]);
          rs->setToRandomPositions();

          // find the BEST state and step from there
          // best being defined as "the most matching predicates and highest heuristics"
          
        }

      } else {
        // case 2: choose a random position
        RobotState *rs = new RobotState(context->robots[idx]);
        rs->setToRandomPositions();

        // find the nearest state to this step
        // then step in the direction of this state rs
        // NOTE: should be interpolating from some other state, not the initial one here
        RobotState *step_rs = new RobotState(context->robots[idx]);
        starting_states[idx]->interpolate(*rs, step, *step_rs, group);
        //search.push_back(step_rs);

        // add to the list of states
        // then delete
        delete rs;
      }
    }

    // clean up
    for (unsigned int i = 0; i < children; ++i) {
      delete next[i];
    }
    delete next;

    return true;
  }
}
