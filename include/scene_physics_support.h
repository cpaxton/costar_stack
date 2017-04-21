#ifndef SCENE_PHYSICS_SUPPORT_H
#define SCENE_PHYSICS_SUPPORT_H

#include <iostream>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/Gimpact/btBoxCollision.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/breadth_first_search.hpp>

// #include <boost/graph/graph_utility.hpp>
// #include <boost/graph/connected_components.hpp> // This is only for undirected graph

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
    
    // This should always be positive, otherwise, we need to reverse the edge direction
    double total_normal_force_;

    scene_support_edge_properties(): total_normal_force_(0.)
    {};

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

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, scene_support_vertex_properties, scene_support_edge_properties> SceneSupportGraph;
typedef boost::graph_traits<SceneSupportGraph>::vertex_descriptor vertex_t;
typedef boost::graph_traits<SceneSupportGraph>::edge_descriptor edge_t;

class label_writer {
public:
    label_writer(SceneSupportGraph &graph) : graph_(graph) {}
    void operator()(std::ostream& out, const vertex_t& v) const 
    {
        out << "[label=\"" << graph_[v].object_id_ << "\"]";
    }
    void operator()(std::ostream& out, const edge_t& e) const 
    {
        const vertex_t &source = boost::source(e,graph_);
        const vertex_t &target = boost::target(e,graph_);
        out << graph_[source].object_id_ << " -> " << graph_[target].object_id_ << std::endl;
    }
private:
    SceneSupportGraph graph_;
};


// class OrderedVertexVisitor : public boost::default_bfs_visitor
// {
// public:
//     OrderedVertexVisitor() : 
//         vertex_visit_index_(0),
//         vertex_visit_order_map_(new std::map<vertex_t, std::size_t>() ),
//         vertex_visit_order_(new std::vector<vertex_t>() )
//     {}

//     void discover_vertex(vertex_t v, const SceneSupportGraph& g)
//     {
//         (*vertex_visit_order_map_)[v] = vertex_visit_index_++;
//         vertex_visit_order_->push_back(v);
//     }

//     std::vector<vertex_t> getVertexVisitList() const
//     {
//         return *(this->vertex_visit_order_);
//     }

//     std::map<vertex_t,std::size_t> getVertexVisitMap() const
//     {
//         return *(this->vertex_visit_order_map_);
//     }
// private:
//     std::size_t vertex_visit_index_;
//     boost::shared_ptr<  std::map<vertex_t, std::size_t> > vertex_visit_order_map_;
//     boost::shared_ptr<  std::vector<vertex_t> > vertex_visit_order_;
// };

class OrderedVertexVisitor
{
public:
    void setDataFromDistanceVector(const std::vector<std::size_t> &distances, const vertex_t &parent_vertex);
    std::vector<vertex_t> getVertexVisitList() const
        { return vertex_visit_order_; }
    std::map<std::size_t, std::vector<vertex_t> > getVertexVisitOrderByDistances() const
        { return vertex_visit_order_by_distances_;}
    std::map<vertex_t, std::size_t> getVertexDistanceMap() const
        { return vertex_distance_map_; }
    std::map<vertex_t,std::size_t> getVertexVisitMap() const
        { return vertex_visit_order_map_; }

private:
    std::map<vertex_t, std::size_t> vertex_visit_order_map_;
    std::map<vertex_t, std::size_t> vertex_distance_map_;
    std::map<std::size_t, std::vector<vertex_t> > vertex_visit_order_by_distances_;
    std::vector<vertex_t> vertex_visit_order_;
};


// will produce order of vertices correction starting the background
OrderedVertexVisitor getOrderedVertexList(SceneSupportGraph &input_graph, const vertex_t &parent_vertex);

void assignAllConnectedToParentVertices(SceneSupportGraph &input_graph, const vertex_t &parent_vertex);

btAABB getCollisionAABB(const btCollisionObject* obj, const btManifoldPoint &pt, const bool &is_body_0, int &shape_index);

double getIntersectingVolume(const btAABB &shapeAABB_a, const btAABB &shapeAABB_b);

SceneSupportGraph generateObjectSupportGraph(btDynamicsWorld *world, 
    std::map<std::string, vertex_t> &vertex_map,
    const btScalar &time_step,  const btVector3 &gravity, 
    const bool &debug_mode = false);


#endif