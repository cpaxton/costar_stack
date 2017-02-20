#ifndef SCENE_PHYSICS_SUPPORT_H
#define SCENE_PHYSICS_SUPPORT_H

#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <boost/graph/adjacency_list.hpp>
// #include <boost/graph/graph_utility.hpp>

#include "physics_world_parameters.h"

struct scene_support_vertex_properties
{
    std::string object_id_;
    btCollisionObject* collision_object_;
    double support_contributions_;
    bool ground_supported_;

    scene_support_vertex_properties(): object_id_(""), collision_object_(NULL),support_contributions_(1.),ground_supported_(false)
    {};
};

// typedef double edge_support_contribution;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, scene_support_vertex_properties> SceneSupportGraph;
typedef boost::graph_traits<SceneSupportGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<SceneSupportGraph>::edge_descriptor edge_t;


std::string getObjectIDFromCollisionObject(const btCollisionObject* object);

void assignAllConnectedToParentVertices(SceneSupportGraph &input_graph, const vertex_t &parent_vertex);

SceneSupportGraph generateObjectSupportGraph(btDynamicsWorld *world, const btScalar &time_step);

#endif