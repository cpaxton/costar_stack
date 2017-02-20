#include "scene_physics_support.h"

std::string getObjectIDFromCollisionObject(const btCollisionObject* object)
{
    if (object->getUserPointer() != NULL)
        return *(std::string*)object->getUserPointer();
    else
        return std::string("unrecognized_object");
}

void assignAllConnectedToParentVertices(SceneSupportGraph &input_graph, const vertex_t &parent_vertex)
{
    boost::graph_traits<SceneSupportGraph>::out_edge_iterator out_i, out_end;
    for (tie(out_i, out_end) = boost::out_edges(parent_vertex, input_graph); out_i != out_end; ++out_i) 
    {
        vertex_t connected_vertex = boost::target(*out_i, input_graph);
        if (parent_vertex == connected_vertex) return;
        else
        {
            scene_support_vertex_properties &support_property = input_graph[connected_vertex];
            support_property.ground_supported_ = true;

            std::cerr << support_property.object_id_ << " support contribution= " 
                << support_property.support_contributions_ << std::endl;
            // recursively call the connected vertex to assign the leaf of vertex connected to the parent

            // @TODO Check whether I need to recurvisely do this or not here
            // assignAllConnectedToParentVertices(input_graph, connected_vertex);
        }
    }
}

SceneSupportGraph generateObjectSupportGraph(btDynamicsWorld *world, const btScalar &time_step)
{
    // std::cerr << "Creating support graph\n";
    // make the graph with number of vertices = number of collision objects
	SceneSupportGraph scene_support_graph(world->getNumCollisionObjects());
    std::map<std::string, vertex_t> vertex_map;
    double dTime_times_gravity = time_step * GRAVITY_MAGNITUDE * SCALING;

    // std::cerr << "Adding vertices\n";
    for (std::size_t i = 0; i < world->getNumCollisionObjects(); i++)
    {
        scene_support_vertex_properties new_vertex_property;
        // std::cerr << "Getting collision object\n";
        new_vertex_property.collision_object_ = world->getCollisionObjectArray()[i];
        // std::cerr << "Getting vertex object id\n";
        new_vertex_property.object_id_ = getObjectIDFromCollisionObject(new_vertex_property.collision_object_);
        // std::cerr << "Add vertex\n";
        vertex_t new_vertex = boost::add_vertex(scene_support_graph);
        // std::cerr << "Assign vertex property\n";
        scene_support_graph[new_vertex] = new_vertex_property;
        // std::cerr << "Assign vertex map\n";
        vertex_map[new_vertex_property.object_id_] = new_vertex;
    }

    // std::cerr << "Checking collisions and adding edges\n";
	int numManifolds = world->getDispatcher()->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold =  world->getDispatcher()->getManifoldByIndexInternal(i);
        const btCollisionObject* obj_0 = contactManifold->getBody0();
        const btCollisionObject* obj_1 = contactManifold->getBody1();

        // all objects should be a btRigidBody
        if (obj_0->getInternalType() == 2 && obj_1->getInternalType() == 2)
        {
            btRigidBody *lower_obj, *upper_obj;
            if (obj_0->getWorldTransform().getOrigin().getZ() < obj_1->getWorldTransform().getOrigin().getZ())
            {
                lower_obj = (btRigidBody*)obj_0; 
                upper_obj = (btRigidBody*)obj_1;
            }
            else
            {
                lower_obj = (btRigidBody*) obj_1; 
                upper_obj = (btRigidBody*) obj_0;
            }

            // assign the graph direction to the supporting object
            vertex_t object_u,supported_object;
            object_u = vertex_map[getObjectIDFromCollisionObject(lower_obj)];
            supported_object = vertex_map[getObjectIDFromCollisionObject(upper_obj)];
            edge_t new_edge; bool add_edge_success;
            boost::tie(new_edge,add_edge_success) = boost::add_edge(object_u,supported_object,scene_support_graph);
            if (add_edge_success)
            {
                // std::cerr << "Edge created\n";
                btScalar totalImpact = 0.;
                for (int p = 0; p < contactManifold->getNumContacts(); p++)
                {  totalImpact += contactManifold->getContactPoint(p).m_appliedImpulse;  }
                // std::cerr << "Total impact between " << getObjectIDFromCollisionObject(lower_obj) <<
                //     " and " << getObjectIDFromCollisionObject(upper_obj) << "= " << totalImpact << std::endl;

                // update the support contribution based on
                // the supporting forces relative to the supported object mass

                // impulse: kg m/s; support_contribution = impulse/(dT * mass * gravity)
                scene_support_graph[object_u].support_contributions_ += totalImpact*upper_obj->getInvMass()/dTime_times_gravity;
            }
            else
            {
                std::cerr << "Error adding edge?\n";
            }
        }
    }
    // std::cerr << "Assigning ground supported vertices\n";
    // Assign vertices that are supported by ground
    vertex_t ground_vertex = vertex_map["background"];
    scene_support_graph[ground_vertex].ground_supported_ = true;
    std::cerr << "Background support contribution= " << scene_support_graph[ground_vertex].support_contributions_ << std::endl;
    assignAllConnectedToParentVertices(scene_support_graph,ground_vertex);
    // std::cerr << "Returning graph\n";
    return scene_support_graph;
}