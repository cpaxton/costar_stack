
#include <ros/ros.h>

// --------------------------------------
// MoveIt types and functions
//#include <moveit_ros/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/PlanningSceneWorld.h>
#include <moveit_msgs/CollisionObject.h>

// --------------------------------------
// Data types and other input
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <map>
#include <string>

namespace predicator {

  /**
   * Entity
   * Defines an unknown object in the world.
   */
  class Entity {
  private:
    std::string name;

  public:
    const std::string &getName();
  };

  
  /**
   * Volume
   * Defines a region of space we are interested in for some reason.
   */
  class Volume {

  };

  /**
   * KnownEntity
   * Defines an object in the world. This is based on an URDF of that object.
   * 
   */
  class KnownEntity {
  };

  /**
   * Observation
   * Defines an unknown point cloud in the world.
   */
  class Observation {

  };

  /**
   * SceneListener
   * Listens for PlanningScene updates and deals with them.
   */
  class SceneListener {
    private:

    public:
      void PlanningSceneCallback(moveit_msgs::PlanningScene::ConstPtr &msg);

      SceneListener(const std::string &topic = "/planning_scene");
  };
}
