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
std::vector<PlanningScene *> scenes;
std::vector<ros::Subscriber> subs;

/**
 * joint_state_callback()
 * Update the robot state variable values
 */
void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg, RobotState *state) {
  state->setVariableValues(*msg);
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

  for (typename std::vector<PlanningScene *>::iterator it = scenes.begin();
       it != scenes.end();
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
    nh_tilde.param("joint_state_topic_list", topics, topics);
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
      std::cout << "Robot Description parameter name: " << desc << std::endl;
    } else {
      ROS_WARN("Description %u was not of type \"string\"!", i);
      continue;
    }

    // create a robot model with state desc
    robot_model_loader::RobotModelLoader robot_model_loader(desc);
    robot_model::RobotModelPtr model = robot_model_loader.getModel();
    PlanningScene *scene = new PlanningScene(model);

    robots.push_back(model);
    scenes.push_back(scene);

    RobotState *state = new RobotState(model);
    states.push_back(state);

    if(i < topics.size() && topics[i].getType() == XmlRpc::XmlRpcValue::TypeString) {
      topic = static_cast<std::string>(topics[i]);
      std::cout << "JointState topic name: " << topic << std::endl;

      // create the subscriber
      subs.push_back(nh.subscribe<sensor_msgs::JointState>
                     (topic, 1000,
                      boost::bind(joint_state_callback, _1, state)));
    } else {
      ROS_WARN("no topic corresponding to description %s!", desc.c_str());
    }
  }

  // define spin rate
  ros::Rate rate(30);

  // start main loop
  while(ros::ok()) {
    ros::spinOnce();

    unsigned int i = 0;
    for(typename std::vector<PlanningScene *>::iterator it1 = scenes.begin();
        it1 != scenes.end();
        ++it1, ++i)
    {
      collision_detection::CollisionRobotConstPtr robot1 = (*it1)->getCollisionRobot();
      typename std::vector<PlanningScene *>::iterator it2 = it1;
      unsigned int j = i+1;
      for(++it2; it2 != scenes.end(); ++it2, ++j) {

        collision_detection::CollisionRobotConstPtr robot2 = (*it2)->getCollisionRobot();

        collision_detection::CollisionRequest req;
        collision_detection::CollisionResult res;

        // force an update
        // source: https://groups.google.com/forum/#!topic/moveit-users/O9CEef6sxbE
        states[i]->update(true);
        states[j]->update(true);
        robot1->checkOtherCollision(req, res, *states[i], *robot2, *states[j]);

        double dist = robot1->distanceOther(*states[i], *robot2, *states[j]);

        std::cout << "(" << robot1->getRobotModel()->getName()
          << ", " << robot2->getRobotModel()->getName()
          << ": Distance to collision: " << dist << std::endl;
      }
    }

    rate.sleep();
  }

  cleanup();
}


