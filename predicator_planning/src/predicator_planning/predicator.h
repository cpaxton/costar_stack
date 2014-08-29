#ifndef _PREDICATOR_PLANNING
#define _PREDICATOR_PLANNING

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <XmlRpcValue.h>

// for debugging
#include <iostream>

// stl
#include <vector>
#include <set>

// MoveIt!
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

// joint states
#include <sensor_msgs/JointState.h>

// predicator
#include <predicator_msgs/PredicateList.h>
#include <predicator_msgs/PredicateStatement.h>
#include <predicator_msgs/ValidPredicates.h>

// boost includes
#include <boost/bind/bind.hpp>

#include "utility.hpp"

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;
using collision_detection::CollisionRobot;

namespace predicator_planning {

  typedef std::unordered_map<predicator_msgs::PredicateStatement,
          unsigned int,
          predicator_planning::Hash,
          predicator_planning::Equals> heuristic_map_t;

  /*
   * joint_state_callback()
   * Update the robot state variable values
   */
  void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state);

  /*
   * createStatement()
   * Simple helper function to create predicates with
   */
  PredicateStatement createStatement(std::string predicate, double value, std::string param1, std::string param2, std::string param3 = "");
 

  /**
   * Predicate
   * Simple internal representation of a predicate
   */
  struct Predicate {
    std::string predicate;
    std::string param1;
    std::string param2;
    std::string param3;

    Predicate(std::string predicate, std::string param1, std::string param2, std::string param3);
  };

  /**
   * PredicateContext
   *
   */
  struct PredicateContext {
    std::vector<std::string> frames; // frames that we will look at for geometry predicates
    std::vector<RobotModelPtr> robots; // robot models
    std::vector<RobotState *> states; // configured robot states
    std::vector<PlanningScene *> scenes;
    std::vector<ros::Subscriber> subs;

    double rel_x_threshold;
    double rel_y_threshold;
    double rel_z_threshold;
    double near_2d_threshold;
    double near_3d_threshold;

    tf::TransformListener listener;

    double padding; // how much padding do we give robot links?
    int verbosity; // how much should get printed for debugging purposes

    /*
     * heuristic_indices
     * Stores the locations in a double array of the indices for different features (heuristics)
     */
    heuristic_map_t heuristic_indices;

    std::map<std::string, std::string> floating_frames;
    std::string world_frame;


    ros::Publisher pub;
    ros::Publisher vpub;

    // valid predicates message
    predicator_msgs::ValidPredicates pval;

    /**
     * Create a PredicateContext()
     * Sets up space, collision robots, etc.
     * This will produce the low-level world predicates
     */
    PredicateContext(bool publish);

    /**
     * numHeuristics()
     */
    size_t numHeuristics() const;

    /**
     * updateWaypoints()
     * Get the list of waypoints from predicator
     * These are for reachability I guess
     */
    void updateWaypoints();

    /**
     * cleanup()
     * Delete memory allocated for robot states and subscribers
     */
    void cleanup();

    /**
     * tick()
     * Run one iteration of the predicator computations 
     */
    void tick();

    /**
     * updatRobotStates()
     * make sure base frames are up to date
     * some objects, such as free-floating robots (aka the ring) need to be updated by TF
     * not sure why this doesn't work naturally
     */
    void updateRobotStates();

    /**
     * addCollisionPredicates()
     * main collision checking loop
     * checks for all pairs of objects, determines collisions and distances
     * publishes the relationships between all of these objects
     *
     * @param idx is the index of a particular PlanningScene.
     * When doing planning, we don't really need to recompute all of the world collisions, just the ones that might be changing.
     */
    void addCollisionPredicates(PredicateList &list, std::vector<double> &heuristics, const std::vector<RobotState *> &states, unsigned int idx=~0);

    /**
     * addGeometryPredicates()
     * compute the set of geometry predicates
     */
    void addGeometryPredicates(PredicateList &list, std::vector<double> &heuristic, const std::vector<RobotState *> &states);

    /**
     * addReachabilityPredicates()
     * compute whether or not we can reach certain points or waypoints
     */
    void addReachabilityPredicates(PredicateList &list, std::vector<double> &heuristics, const std::vector<RobotState *> &states);

    /**
     * getLinkTransform
     * Check to see if this is in the list of floating transfoms
     * If so, compose with TF frame
     */
    Eigen::Affine3d getLinkTransform(const RobotState *state, const std::string &linkName) const;


    /**
     * updateIndices()
     * Records where the values we can use as heuristics are going to be stored.
     * May also look at things like waypoints, etc.
     */
    void updateIndices();

    /**
     * getHeuristic
     * Looks up a score from a vector of possible values
     */
    double getHeuristic(const PredicateStatement &pred, const std::vector<double> &heuristics) const;
  };
}

#endif
