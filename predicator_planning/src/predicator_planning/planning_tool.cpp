#include "predicator.h"
#include "planning_tool.h"

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

// joint model group -- which joints are we solving for?
#include <moveit/robot_model/joint_model_group.h>

// standard libraries for random
#include <cstdlib>
#include <unordered_set>

namespace predicator_planning {

  Planner::Planner(PredicateContext *_context, unsigned int _max_iter, unsigned int _children, double _step, double _chance) :
    context(_context), max_iter(_max_iter), children(_children), step(_step), chance(_chance)
  {
    ros::NodeHandle nh;
    planServer = nh.advertiseService("predicator/plan", &Planner::plan, this);
    nh.param("verbosity", verbosity, 0);
  }

  // default constructor
  Planner::SearchPose::SearchPose() : state(NULL), count_best(0), parent(NULL), child(NULL), cost(0) {}

  // initialize parents, variables
  Planner::SearchPose::SearchPose(std::deque<SearchPose *> &search, RobotState *_state) :
    count_best(0), parent(NULL), child(NULL), state(_state), cost(0)
  {
    double shortest_dist = 999999.;
    for(unsigned int i = 0; i < search.size(); ++i) {
      // compare distance
      double dist = search[i]->state->distance(*state);
      if (parent == NULL || dist < shortest_dist) {
        parent = search[i];
        cost = parent->cost + dist;
      }
    }
  }

  // update with information from the context
  bool Planner::SearchPose::checkPredicates(PredicatePlan::Request &req, PredicateContext *context, unsigned int idx, bool &goals) {

    // get starting states
    // these are the states as recorded in the context
    // they will be updated as we go on if this takes a while -- might be bad
    std::vector<RobotState *> states = context->states;
    states[idx] = state; // set to this state

    std::vector<double> all_heuristics(context->numHeuristics());
    PredicateList list;

    context->updateRobotStates();

    context->addCollisionPredicates(list, all_heuristics, states);
    context->addGeometryPredicates(list, all_heuristics, states);
    context->addReachabilityPredicates(list, all_heuristics, states);

    std::unordered_set<PredicateStatement, Hash, Equals> lookup;

    for (PredicateStatement &ps: list.statements) {
      //ROS_INFO("%s(%s,%s,%s)",ps.predicate.c_str(),
      //         ps.params[0].c_str(),
      //         ps.params[1].c_str(),
      //         ps.params[2].c_str());
      lookup.insert(ps);
    }

    // check requirements
    for (PredicateStatement &ps: req.required_true) {
      if (lookup.find(ps) == lookup.end()) {
        return false;
      } else {
        ROS_INFO("found goal state for predicate %s(%s,%s,%s)",ps.predicate.c_str(),
               ps.params[0].c_str(),
               ps.params[1].c_str(),
               ps.params[2].c_str());
      }
    }
    for (PredicateStatement &ps: req.required_false) {
      if (lookup.find(ps) != lookup.end()) {
        return false;
      }
    }

    goals = true;

    // compute predicate stuff right here
    for (PredicateStatement &ps: req.goal_true) {
      // look at the predicates
      // get a heuristic value from context

      double val = context->getHeuristic(ps, all_heuristics);

      // supdate this pose based on that
      heuristics.push_back(val);

      if(lookup.find(ps) == lookup.end()) {
        goals = false;
      }
    }
    for (PredicateStatement &ps: req.goal_false) {
      // look at the predicates
      // get a heuristic value from context
      double val = -1 * context->getHeuristic(ps, all_heuristics);

      // update this pose based on that
      heuristics.push_back(val);
      if(lookup.find(ps) != lookup.end()) {
        goals = false;
      }
    }

    std::cout << "values = [";
    if(goals == false) {
      for (double &d: heuristics) {
        std::cout << (1.0 * d) << ", ";
      }
    }
    std::cout << "]" << std::endl;

    return true;
  }

  bool Planner::plan(predicator_planning::PredicatePlan::Request &req,
                     predicator_planning::PredicatePlan::Response &res)
  {

    ROS_INFO("Received planning request.");

    if(context->heuristic_indices.find(createStatement("near", 0, "wam/wrist_palm_link", "peg1/peg_top_link"))
       != context->heuristic_indices.end())
    {
      ROS_INFO("FOUND NEAR");
      ROS_INFO("INDEX = %u", context->heuristic_indices[createStatement("near", 0, "wam/wrist_palm_link", "peg1/peg_top_link")]);
    }

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
      ROS_ERROR("Unable to get group \"%s\" for robot \"%s\"!", req.group.c_str(), req.robot.c_str());
      return false;
    }
    ROS_INFO("Found group \"%s\" for robot \"%s\".", req.group.c_str(), req.robot.c_str());

    SearchPose *first = new SearchPose();
    bool goals_found = false;
    first->state = new RobotState(*context->states[idx]);
    first->checkPredicates(req, context, idx, goals_found);
    search.push_back(first);

    ROS_INFO("Added first state.");

    // loop over 
    for (unsigned int iter = 0; iter < max_iter && !res.found; ++iter) {
      // either generate a starting position at random or...
      // step in a direction from a "good" position (as determined by high heuristics)

      double choose_op = (double)rand() / (double)RAND_MAX;
      std::cout << "Iteration " << iter << ": "; //, random value = " << choose_op << std::endl;
      if(choose_op > chance) {
        // spawn children from the thing with the highest heuristic

        RobotState *rs = new RobotState(context->robots[idx]);
        rs->setToRandomPositions();

        // find the BEST state and step from there
        // best being defined as "the most matching predicates and highest heuristics"

        delete rs;

      } else {
        // case 2: choose a random position
        RobotState *rs = new RobotState(context->robots[idx]);
        rs->setToRandomPositions();

        // find the nearest state to this step
        // then step in the direction of this state rs
        // NOTE: should be interpolating from some other state, not the initial one here
        SearchPose *new_sp = new SearchPose(search, rs);

        new_sp->parent->state->interpolate(*rs, step, *rs, group);

        goals_found = false;

        if (new_sp->checkPredicates(req, context, idx, goals_found) && new_sp->state->satisfiesBounds()) {
          search.push_back(new_sp);
        } else {
          delete new_sp->state;
          delete new_sp;
        }

        res.found = goals_found;
      }

      res.iter = iter;

      if (res.found == true) {
        break;
      }
    }

    // get a path from the last thing we added
    // by going backwards to the start
    std::deque<RobotState *> path;
    SearchPose *cur = *search.rbegin();
    path.push_front(cur->state);
    while (cur->parent != NULL) {
      // instead of finding parents we may want to find the closest node
      // or the node with the least distance (COST) within some threshold
      cur = cur->parent;
      path.push_front(cur->state);
    }

    // now go forward over the list
    for(std::deque<RobotState *>::const_iterator it = path.begin();
        it != path.end();
        ++it)
    {
      const double *positions = (*it)->getVariablePositions();
      trajectory_msgs::JointTrajectoryPoint pt;
      pt.positions.assign(positions, positions + (*it)->getVariableCount());
      res.path.points.push_back(pt);
    }
    res.path.joint_names = path[0]->getVariableNames();


    // clean up
    for (unsigned int i = 0; i < search.size(); ++i) {
      delete search[i]->state;
      delete search[i];
    }

    std::cout << "returning" << std::endl;
    return true;
  }
}
