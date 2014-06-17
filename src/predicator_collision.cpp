
/**
 * Based on MoveIt C++ tutorial
 * https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/src/planning_scene_tutorial.cpp
 */


// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

// joint states
#include <sensor_msgs/JointState.h>

// predicator
#include <predicator_msgs/PredicateList.h>

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;


// state of the robot generating these predicates
robot_state::RobotState *state;

// planning scene for collisions
planning_scene::PlanningScene *scene;

/**
 * joint_state_callback()
 * Read in a joint position and add it to stuff.
 * This keeps the robot state model updated.
 */
void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg) {
  //state->setStateValues(*msg);
  state->setVariableValues(*msg);
  scene->setCurrentState(*state);
}

/**
 * planning_scene_callback()
 * Planning scene callback. Reads in planning messages to keep the local
 * planning state up-to-date so we can accurately return predicates.
 */
void planning_scene_callback(const moveit_msgs::PlanningScene::ConstPtr &msg) {
  scene->usePlanningSceneMsg(*msg);
}

/**
 * main()
 * Creates a collision predicator node.
 * This node needs to publish a set of predicates (as per the PredicateList message type).
 * The list will go to the predicate core service / "Predicator Core"
 */
int main (int argc, char **argv) {

  std::string robot_description_param;

  ros::init(argc, argv, "predicator_robot_collision_node");
  
  ros::NodeHandle nh;
  ros::NodeHandle nh_tilde("~");

  nh_tilde.param("robot_description_param", robot_description_param, std::string("robot_description"));

  ros::Subscriber js_sub = nh.subscribe("/joint_states", 1000, joint_state_callback);
  ros::Subscriber ps_sub = nh.subscribe("/planning_scene", 1000, planning_scene_callback);
  ros::Publisher pub = nh.advertise<predicator_msgs::PredicateList>("predicator/input", 1000);

  // load model files
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description_param);
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  scene = new PlanningScene(kinematic_model);


  //void 	usePlanningSceneMsg (const moveit_msgs::PlanningScene &scene)
  //void 	setCurrentState (const moveit_msgs::RobotState &state)
  

  // create robot state, and then update it with the joint states
  // RobotState (const robot_model::RobotModelConstPtr &kinematic_model)
  state = new robot_model::RobotState(kinematic_model);


  ros::Rate rate(30);
  while(ros::ok()) {
    ros::spinOnce();

    // get updates
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();  

    scene->checkCollision(collision_request, collision_result, *state, acm);

    // create output message and send it
    predicator_msgs::PredicateList msg;
    msg.header.frame_id = ros::this_node::getName();
    pub.publish(msg);

    rate.sleep();
  }

  // cleanup
  delete scene;
  delete state;
}
