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
  
}
