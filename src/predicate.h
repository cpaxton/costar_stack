
#include <ros/ros.h>

// --------------------------------------
// FCL stuff
#include <fcl/data_types.h> // Triangles, other simple data types
#include <fcl/shape/geometric_shapes.h> // spheres, cylinders, etc.
#include <fcl/BVH/BVH_model.h>

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

  typedef fcl::BVHModel<fcl::OBBRSS> Model;

  class CollisionEntity;

  /**
   * GeometryParser
   * Listens for PlanningScene updates and deals with them.
   * 
   * NOTE: this should also connect to MoveIt, for convenience functions describing robot/world interactions.
   * Right now, there are no names sent for individual links in world objects on the PlanningScene topic.
   * This could cause trouble if we want to identify specific parts of the objects by name.
   */
  class GeometryParser {
  private:
    ros::Subscriber sub_; // listen to incoming planning scenes
    ros::Publisher pub_; // publish predicate information

    ros::NodeHandle nh_; // node handle for extra stuff

  protected:

    // set of all collision entities
    std::map<std::string, CollisionEntity> entities_;

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
   * Consists of Collision Links, which are smaller objects.
   */
  class CollisionEntity {
  private:
    std::string id_; // what identifies this object?
    bool enabled_; // are we tracking predicates?
    bool open_; // are we still reading links?

    geometry_msgs::Pose pose_;

  protected:
    Model * model;

  public:
    const std::string &getId() const;

    CollisionEntity(const std::string &id = "");

    void setId(const std::string &new_id);

    /**
     * setEnabled()
     * Set the object to be enabled in the world.
     * Is the object enabled? AKA, should we report predicates?
     */
    void setEnabled(bool enabled);

    /**
     * isEnabled()
     * Is the object enabled? AKA, should we report predicates?
     */
    bool isEnabled() const;

    /**
     * isOpen()
     * Is this open to add more links to it?
     */
    const bool isOpen() const;

    /**
     * open()
     * Set this object to accept new links
     */
    void open();

    /**
     * close()
     * Stop adding new links
     */
    void close();

    /**
     * purge()
     * Delete the underlying structure(s) 
     */
    void purge();

    /**
     * addLink()
     * Append a single sub model to the object.
     */
    void addLink(const std::vector<fcl::Vec3f> &vertices, const std::vector<fcl::Triangle> &triangles);
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
