#ifndef SCENE_PHYSICS_SUPPORT_H
#define SCENE_PHYSICS_SUPPORT_H

#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btBoxCollision.h>
#include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/graph_utility.hpp>

#include "physics_world_parameters.h"
#include "utility.h"

struct scene_support_vertex_properties
{
    std::string object_id_;
    btCollisionObject* collision_object_;
    btTransform object_pose_;
    double support_contributions_;
    double penetration_distance_;
    double colliding_volume_;
    bool ground_supported_;
    double stability_penalty_;

    scene_support_vertex_properties(): 
    	object_id_(""),
    	collision_object_(NULL),
    	support_contributions_(1.),
    	penetration_distance_(0.),
        colliding_volume_(0.),
    	ground_supported_(false),
        stability_penalty_(0)
    {};
};

struct scene_support_edge_properties
{
    std::map<int, std::set<int> > interecting_volume_index_;

    bool collision_pair_exists(const int &shape_index_1, const int &shape_index_2 )
    {
        if (keyExistInConstantMap(shape_index_1, interecting_volume_index_))
        {
            const std::set<int> &index_set = getContentOfConstantMap(shape_index_1, interecting_volume_index_);
            return index_set.find(shape_index_2) != index_set.end();
        }
        else
            return false;
    }

    void add_pair(const int &shape_index_1, const int &shape_index_2)
    {
        this->interecting_volume_index_[shape_index_1].insert(shape_index_2);
    }
};

// typedef double edge_support_contribution;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, scene_support_vertex_properties, scene_support_edge_properties> SceneSupportGraph;
typedef boost::graph_traits<SceneSupportGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<SceneSupportGraph>::edge_descriptor edge_t;

std::string getObjectIDFromCollisionObject(const btCollisionObject* object);

void assignAllConnectedToParentVertices(SceneSupportGraph &input_graph, const vertex_t &parent_vertex);

btAABB getCollisionAABB(const btCollisionObject* obj, const btManifoldPoint &pt, const bool &is_body_0, int &shape_index);

double getIntersectingVolume(const btAABB &shapeAABB_a, const btAABB &shapeAABB_b);

SceneSupportGraph generateObjectSupportGraph(btDynamicsWorld *world, 
    std::map<std::string, vertex_t> &vertex_map,
    const btScalar &time_step,  const btVector3 &gravity, 
    const bool &debug_mode = false);



#endif