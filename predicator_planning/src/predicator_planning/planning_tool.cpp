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

  Planner::Planner(PredicateContext *_context, unsigned int _max_iter, double _step, double _chance, double _skip) :
    context(_context), max_iter(_max_iter), step(_step), chance(_chance), skip_distance(_skip)
  {
    ros::NodeHandle nh;
    planServer = nh.advertiseService("predicator/plan", &Planner::plan, this);
    nh.param("verbosity", verbosity, 0);
  }

  // default constructor
  Planner::SearchPose::SearchPose() : state(NULL), count_best(0), parent(NULL), child(NULL), cost(0), count_met(0), hsum(0.) {}

  // initialize parents, variables
  Planner::SearchPose::SearchPose(std::deque<SearchPose *> &search, RobotState *_state) :
    count_best(0), parent(NULL), child(NULL), state(_state), cost(0), count_met(0), hsum(0.)
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

    //if (!state->satisfiesBounds()) {
    //  return false;
    //}

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

      if(lookup.find(ps) == lookup.end()) {
        goals = false;
      } else if (val < 0) {
        val = 0;
      }

      // update this pose based on that
      heuristics.push_back(val);
    }
    for (PredicateStatement &ps: req.goal_false) {
      // look at the predicates
      // get a heuristic value from context
      double val = -1 * context->getHeuristic(ps, all_heuristics);

      if(lookup.find(ps) != lookup.end()) {
        goals = false;
      } else if (val < 0) {
        val = 0;
      }

      // update this pose based on that
      heuristics.push_back(val);
    }

    std::cout << "values = [";
    for (double &d: heuristics) {
      std::cout << d << ", ";
      if (d >= 0) {
        ++count_met;
      } else {
        hsum += d;
      }
    }
    std::cout << "]";
    if (goals == true) {
      std::cout << " (MEETS GOALS)";
    }
    std::cout << std::endl;

    return true;
  }

  bool Planner::plan(predicator_planning::PredicatePlan::Request &req,
                     predicator_planning::PredicatePlan::Response &res)
  {

    ROS_INFO("Received planning request.");

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

    if (idx >= context->robots.size()) {
      ROS_ERROR("Could not find robot \"%s\"!", req.robot.c_str());
      return false;
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
      std::cout << "Iteration " << iter << "(" << (choose_op > chance) << ") :";
      if(choose_op > chance) {
        // spawn children from the thing with the highest heuristic
        // first, iterate over all poses
        // track the best number of goals met and best values for unmet goals
        SearchPose *best = NULL;
        unsigned int best_met = 0;
        double best_hsum = 99999.9;
        for (std::deque<SearchPose *>::iterator it = search.begin();
             it != search.end();
             ++it) {
          if (((*it)->count_met > best_met) ||
               (((*it)->count_met == best_met) && (*it)->hsum > best_hsum) ||
              best == NULL)
          {
            best = *it;
            best_hsum = best->hsum;
            best_met = best->count_met;
            //ROS_INFO("expanding from sum=%f, count=%u", best_hsum, best_met);
          }
        }

        // find the BEST state and step from there
        // best being defined as "the most matching predicates and highest heuristics"
        RobotState *rs = new RobotState(context->robots[idx]);
        rs->setToRandomPositionsNearBy(group, *best->state, 0.50);
        SearchPose *new_sp = new SearchPose(search, rs);
        new_sp->parent->state->interpolate(*rs, step, *rs, group);

        // check and add or delete
        if (new_sp->checkPredicates(req, context, idx, goals_found)) {
          search.push_back(new_sp);
          res.found = goals_found;
        } else {
          ROS_INFO("Deleting illegal state.");
          delete new_sp->state;
          delete new_sp;
        }


      } else {
        // case 2: choose a random position
        RobotState *rs = new RobotState(context->robots[idx]);
        rs->setToRandomPositions(group);

        // find the nearest state to this step
        // then step in the direction of this state rs
        // NOTE: should be interpolating from some other state, not the initial one here
        SearchPose *new_sp = new SearchPose(search, rs);

        new_sp->parent->state->interpolate(*rs, step, *rs, group);

        goals_found = false;

        if (new_sp->checkPredicates(req, context, idx, goals_found)) {
          search.push_back(new_sp);
          res.found = goals_found;
        } else {
          ROS_INFO("Deleting illegal state.");
          delete new_sp->state;
          delete new_sp;
        }
      }

      res.iter = iter;

      if (res.found == true) {
        break;
      }
    }

    SearchPose *cur = *search.rbegin();
    if(res.found != true) {
      ROS_INFO("selecting final node based on goal performance");
      SearchPose *best = NULL;
      unsigned int best_met = 0;
      double best_hsum = 99999.9;
      for (std::deque<SearchPose *>::iterator it = search.begin();
           it != search.end();
           ++it) {
        if (((*it)->count_met > best_met) ||
            (((*it)->count_met == best_met) && (*it)->hsum > best_hsum) ||
            best == NULL)
        {
          best = *it;
          best_hsum = best->hsum;
          best_met = best->count_met;
        }
      }

      // take the best pose
      cur = best;
    }

    ROS_INFO("selected final node");

    // get a path from the last thing we added
    // by going backwards to the start
    std::deque<RobotState *> path;
    //path.push_front(cur->state);
    //cur = cur->parent;
    int i = 0; 
    while (cur != NULL) {
      ++i;
      // instead of finding parents we may want to find the closest node
      // or the node with the least distance (COST) within some threshold
      path.push_front(cur->state);

      int j = i;
      SearchPose *parent = cur;
      SearchPose *next = parent;
      while (parent->parent != NULL) {
        ++j;
        parent = parent->parent;

        double dist = cur->state->distance(*parent->state, group);

        std::cout << "(" << i << ", " << j << ") ";
        std::cout << cur->hsum;

        //if(cur->parent != NULL) {
        std::cout << ", dist = ";
        std::cout << dist;
        //}
        std::cout << std::endl; 

        if (dist < skip_distance) {
          cur->parent = parent->parent;
        }
      }
      cur = cur->parent;
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
