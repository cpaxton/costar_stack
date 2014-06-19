// ROS
#include <ros/ros.h>
#include <XmlRpcValue.h>

// for debugging
#include <iostream>

// stl
#include <vector>

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

// boost includes
#include <boost/bind/bind.hpp>

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;
using collision_detection::CollisionRobot;

std::vector<RobotModelPtr> robots;
std::vector<RobotState *> states;
std::vector<ros::Subscriber> subs;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state) {

}

/**
 * cleanup()
 * Delete memory allocated for robot states and subscribers
 */
void cleanup() {
  for (typename std::vector<RobotState *>::iterator it = states.begin();
       it != states.end();
       ++it)
  {
    delete *it;
  }
}

int main(int argc, char **argv) {

  double padding; // how much padding do we give robot links?

  ros::init(argc, argv, "predicator_robot_interaction_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_tilde("~");

  XmlRpc::XmlRpcValue descriptions;
  XmlRpc::XmlRpcValue topics;

  if(nh_tilde.hasParam("description_list")) {
    nh_tilde.param("description_list", descriptions, descriptions);
  } else {
    ROS_ERROR("No list of robot description parameters!");
    exit(-1);
  }

  if(nh_tilde.hasParam("joint_state_topic_list")) {
    nh_tilde.param("topic_list", topics, topics);
  } else {
    ROS_ERROR("No list of joint state topics!");
    exit(-1);
  }

  if(descriptions.size() != topics.size()) {
    ROS_WARN("An unequal number of joint state and robot topics was provided!");
  }

  for(unsigned int i = 0; i < descriptions.size(); ++i) {
    std::string desc;
    std::string topic;

    if(descriptions[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
      desc = static_cast<std::string>(descriptions[i]);
    } else {
      ROS_WARN("Description %u was not of type \"string\"!", i);
      continue;
    }

    // create a robot model with state desc
    robot_model_loader::RobotModelLoader robot_model_loader(desc);
    robot_model::RobotModelPtr model = robot_model_loader.getModel();

    robots.push_back(model);

    if(i < topics.size() && topics[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
      topic = static_cast<std::string>(topics[i]);

      RobotState *state = new RobotState(model);
      states.push_back(state);

      // create the subscriber
      subs.push_back(nh.subscribe<sensor_msgs::JointState>
                     (topic, 1000,
                      boost::bind(joint_state_callback, _1, state)));
    }
  }

  ros::Rate rate(30);
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  cleanup();
}


