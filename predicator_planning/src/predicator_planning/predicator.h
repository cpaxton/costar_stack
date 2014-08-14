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

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;
using collision_detection::CollisionRobot;

namespace predicator_planning {

  /**
   * PredicateContext
   *
   */
  struct PredicateContext {
    std::vector<RobotModelPtr> robots;
    std::vector<RobotState *> states;
    std::vector<PlanningScene *> scenes;
    std::vector<ros::Subscriber> subs;
    tf::TransformListener listener;
    double padding; // how much padding do we give robot links?
    int verbosity; // how much should get printed for debugging purposes
    std::map<std::string, std::string> floating_frames;


    std::string world_frame;

    void initialize(bool publish);

    /**
     * cleanup()
     * Delete memory allocated for robot states and subscribers
     */
    void cleanup();
  };
}
