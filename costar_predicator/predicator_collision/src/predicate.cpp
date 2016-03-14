#include "predicate.h"

namespace predicator {

  /**
   * updateCollisionObject()
   * Called once for each CollisionObject method.
   * If operation == add or append, it will add these entities to that object.
   * Else, it can move the pose associated with that object.
   */
  void GeometryParser::updateCollisionObject(const moveit_msgs::CollisionObject &co) {
    ROS_INFO("updating collision object \"%s\", command=%u", co.id.c_str(), co.operation);

    // check to see if we should be adding it or what
    if(co.operation == moveit_msgs::CollisionObject::ADD) {
      entities_[co.id] = CollisionEntity(co.id);
    } else if(co.operation == moveit_msgs::CollisionObject::REMOVE) {
      entities_[co.id].setEnabled(false);
      entities_[co.id].purge(); // delete the unused model
      return;
    } else if (entities_.find(co.id) == entities_.end()) {
      entities_[co.id] = CollisionEntity(co.id);
    }

    entities_[co.id].setEnabled(true);
    entities_[co.id].setId(co.id);


    // create a bunch of entities!
    if (entities_[co.id].isEnabled()) {

      if (!entities_[co.id].isOpen()) {
        entities_[co.id].open();
      }

      // ------------------------------------------------------
      // add the mesh or update it
      // ------------------------------------------------------
      for (unsigned int i = 0; i < co.meshes.size(); ++i) {
        // generate a BVH model from the shapes
        // generate a mesh from triangles and points

        std::vector<fcl::Triangle> triangles;
        std::vector<fcl::Vec3f> vertices;

        // ------------------------------------------------------
        // Triangle mesh
        // ------------------------------------------------------
        for(std::vector<shape_msgs::MeshTriangle>::const_iterator tr = co.meshes[i].triangles.begin();
            tr != co.meshes[i].triangles.end();
            ++tr)
        {
          fcl::Triangle fcl_tr(tr->vertex_indices[0],
                               tr->vertex_indices[1],
                               tr->vertex_indices[2]);
          triangles.push_back(fcl_tr);
        }

        // ------------------------------------------------------
        // Actual vertex coordinates
        // ------------------------------------------------------
        for(std::vector<geometry_msgs::Point>::const_iterator pt = co.meshes[i].vertices.begin();
            pt != co.meshes[i].vertices.end();
            ++pt)
        {
          fcl::Vec3f vec;
          vec[0] = pt->x;
          vec[1] = pt->y;
          vec[2] = pt->z;

          vertices.push_back(vec);
        }

        // add link to an object
        entities_[co.id].addLink(vertices, triangles);
      }
    }
  }

  CollisionEntity::CollisionEntity(const std::string &id) : id_(id) {
    // do whatever to create this thing
  }

  /**
   * isOpen()
   * Is this open to add more links to it?
   */
  const bool CollisionEntity::isOpen() const { return open_; }

  /**
   * open()
   * Set this object to accept new links
   */
  void CollisionEntity::open() {
    if (model == NULL) {
      model = new Model(); // create the model
    }
    model->beginModel();
  }

  /**
   * purge()
   * Delete the underlying structure(s) 
   */
  void CollisionEntity::purge() {
    if(model != NULL) {
      delete model;
    }
    model = NULL;
  }

  /**
   * addLink()
   * Append a single sub model to the object.
   */
  void CollisionEntity::addLink(const std::vector<fcl::Vec3f> &vertices, const std::vector<fcl::Triangle> &triangles) {
    model->addSubModel(vertices, triangles);
  }

  /**
   * close()
   * Stop adding new links
   */
  void CollisionEntity::close() {
    model->endModel();
  }

  // -------------------------------------------------------------------------
  // Getters and Setters for basic CollisionEntity properties
  // -------------------------------------------------------------------------
  void CollisionEntity::setEnabled(bool enabled) { enabled_ = enabled; }
  bool CollisionEntity::isEnabled() const { return enabled_; }
  void CollisionEntity::setId(const std::string &new_id) { id_ = new_id; }
  const std::string &CollisionEntity::getId() const { return id_; }
  // -------------------------------------------------------------------------

  /**
  */
  void GeometryParser::planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr &msg) {
    for(std::vector<moveit_msgs::CollisionObject>::const_iterator it = msg->world.collision_objects.begin();
        it != msg->world.collision_objects.end();
        ++it) {
      updateCollisionObject(*it);
    }
  }

  /**
  */
  GeometryParser::GeometryParser(const std::string &planning_topic,
                                 const std::string &predicate_topic)
    : nh_()
  {
    // advertize a topic with predicate information
    pub_ = nh_.advertise<predicator_msgs::PredicateList>(predicate_topic, 1000);

    // call something to initialize the planning scene

    // subscribe to planning messages
    sub_ = nh_.subscribe(planning_topic, 1000, &GeometryParser::planningSceneCallback, this);
  }


}
