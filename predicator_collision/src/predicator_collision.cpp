
/**
 * Based on MoveIt C++ tutorial
 * https://github.com/ros-planning/moveit_pr2/blob/hydro-devel/pr2_moveit_tutorials/planning/src/planning_scene_tutorial.cpp
 */


// ROS
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// for debugging
#include <iostream>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

// joint states
#include <sensor_msgs/JointState.h>

// predicator
#include <predicator_msgs/PredicateList.h>
#include <predicator_msgs/PredicateStatement.h>

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
  int call_get_planning_scene; // mode for retrieving the planning scene
  int world_only; // only show collisions to the world
  int verbosity; // how much nonsense should we print out
  double padding; // how much padding do we give robot links?

  ros::init(argc, argv, "predicator_robot_collision_node");

  ros::NodeHandle nh;
  ros::NodeHandle nh_tilde("~");

  nh_tilde.param("robot_description_param", robot_description_param, std::string("/robot_description"));

  /* get_planning_scene
   * 0: don't get planning scene
   * 1: get planning scene with a moveit get_planning_scene call
   * 2: get planning scene with a gazebo planning scene plugin call
   */
  nh_tilde.param("get_planning_scene", call_get_planning_scene, 1);
  nh_tilde.param("world_collisions_only", world_only, 1);
  nh_tilde.param("verbosity", verbosity, 0);
  nh_tilde.param("padding", padding, 0.01);

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

  std::cout << call_get_planning_scene << std::endl;
  if (call_get_planning_scene == 1) {
    // manually call service to set up planning scene state
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    moveit_msgs::GetPlanningScene req;
    req.request.components.components = ~0;

    client.call(req);
    scene->setPlanningSceneMsg(req.response.scene);
  } else if (call_get_planning_scene == 2) {
    if(verbosity > 0) {
      std::cout << "!!! force publish planning scene" << std::endl;
    }
    //send a request to republish all the important information
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/publish_planning_scene");
    std_srvs::Empty req;
    client.call(req);
  }

  std::string name = scene->getRobotModel()->getName();

  // get updates
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  collision_request.contacts = true;
  collision_request.distance = true;
  if(verbosity > 0) {
    collision_request.verbose = true;
  } else {
    collision_request.verbose = false;
  }
  collision_request.cost = true;
  collision_request.max_contacts = 1000;

  // ignore all self collisions
  collision_detection::AllowedCollisionMatrix acm = scene->getAllowedCollisionMatrix();


  ros::Rate rate(30);
  while(ros::ok()) {
    // set robot padding so that we actually detect collisions
    scene->getCollisionRobotNonConst()->setPadding(padding);
    //scene->getCollisionRobotNonConst()->setScale(30000.0);
    scene->propogateRobotPadding();

    ros::spinOnce();

    if(world_only) {
      collision_result.clear();
      scene->checkSelfCollision(collision_request, collision_result, *state, acm);
      for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); 
          it != collision_result.contacts.end(); 
          ++it)
      {
        acm.setEntry(it->first.first, it->first.second, true);    
        std::cout << "Ignoring: " << it->first.first << " --> " << it->first.second << std::endl;
      }
    }

    collision_result.clear();

    // check collisions
    scene->checkCollision(collision_request, collision_result, *state, acm);

    if (verbosity > 0) {
      std::cout << "Distance = " << collision_request.distance << std::endl;
      std::cout << "Cost Sources = " << collision_result.cost_sources.size() << std::endl;
    }

    // create predicate list
    predicator_msgs::PredicateList msg;
    for(collision_detection::CollisionResult::ContactMap::const_iterator it = collision_result.contacts.begin(); 
        it != collision_result.contacts.end(); 
        ++it)
    {
      // look at the contents of the map and print them out for now?
      //std::cout << it->first.first << ", " << it->first.second << std::endl;
      predicator_msgs::PredicateStatement ps;
      ps.predicate = "touching";
      ps.num_params = 2;
      ps.params[0] = it->first.first;
      ps.params[1] = it->first.second;
      msg.statements.push_back(ps);
    }

    if (verbosity > 0) {
      ROS_INFO_STREAM(name << " current state is " << (state->satisfiesBounds() ? "valid" : "not valid"));
      ROS_INFO_STREAM(name << " current state is " << (collision_result.collision ? "in" : "not in") << " collision");
    }

    if(collision_result.collision) {
      predicator_msgs::PredicateStatement ps_collision;
      ps_collision.predicate = "in_collision";
      ps_collision.num_params = 1;
      ps_collision.params[0] = name;
      msg.statements.push_back(ps_collision);
    }

    if(state->satisfiesBounds()) {
      predicator_msgs::PredicateStatement ps_valid;
      ps_valid.predicate = "valid_position";
      ps_valid.num_params = 1;
      ps_valid.params[0] = name;
      msg.statements.push_back(ps_valid);
    }

    // create output message and send it
    msg.pheader.source = ros::this_node::getName();
    pub.publish(msg);

    // ... and sleep
    rate.sleep();
  }

  // cleanup
  delete scene;
  delete state;
}
