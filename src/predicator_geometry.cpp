#include <ros/ros.h>

// --------------------------------------
// Data types and other input
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <map>
#include <string>


// --------------------------------------
// Semantic Core Functionality
#include "predicate.h"

using std::map;
using std::string;
using geometry_msgs::Pose; // object base pose
using sensor_msgs::JointState; // for robot arms and stuff

// register callbacks for the different objects we want to read
// read in the urdf models of the different objects as well

map<string, Pose> obs;

using namespace predicator;

int main(int argc, char **argv) {

  ros::init(argc, argv,"predicator_geometry_node");
  ros::NodeHandle nh_tilde("~");

  std::string input_topic;
  std::string output_topic;
  
  nh_tilde.param("predicate_output_topic", output_topic, std::string("/predicator/input"));
  nh_tilde.param("planning_scene_topic", input_topic, std::string("/planning_scene"));

  GeometryParser gp(input_topic, output_topic);
}
