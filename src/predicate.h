
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
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>

// --------------------------------------
// Predicator stuff
#include <predicator_msgs/PredicateStatement.h>
#include <predicator_msgs/PredicateList.h>

#include <map>
#include <string>

namespace predicator {


  /**
   * GeometryParser
   * Listens for PlanningScene updates and deals with them.
   * Includes a lot of 
   */
  class GeometryParser {
    private:
      ros::Subscriber sub_; // listen to incoming planning scenes
      ros::Publisher pub_; // publish predicate information

      ros::NodeHandle nh_; // node handle for extra stuff

    protected:
      void addMesh(const std::string &id, const shape_msgs::Mesh &mesh);

      void addPrimitive(const std::string &id, const shape_msgs::SolidPrimitive &primitive);

      /**
       * updateCollisionObject()
       * Called once for each CollisionObject method.
       * If operation == add or append, it will add these entities to that object.
       * Else, 
       */
      void updateCollisionObject(const moveit_msgs::CollisionObject &co);

    public:
      void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr &msg);

      /**
       * GeometryParser()
       * Constructor, creates a subscriber and a publisher among other things
       */
      GeometryParser(const std::string &planning_topic, const std::string &predicate_topic);

  };

  /**
   * Entity
   * Defines an unknown object in the world.
   */
  class CollisionEntity {
  private:
    std::string id;

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
}
