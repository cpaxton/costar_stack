#include <iostream>

#include "sequential_scene_parsing.h"
#include "utility.h"


SceneGraph::SceneGraph(Image input, Image background_image)
{
	this->addBackground(background_image);
	// this->processImage(input);
	this->physics_engine_ready_ = false;
}

void SceneGraph::setPhysicsEngine(PhysicsEngine* physics_engine)
{
	if (this->debug_messages_) std::cerr <<"Setting physics engine into the scene graph.\n";
	this->physics_engine_ = physics_engine;

	// Check if physics engine exist
	this->physics_engine_ready_ = (physics_engine_ != NULL);
}

void SceneGraph::addBackground(Image background_image)
{
	if (this->debug_messages_) std::cerr <<"Adding background into the scene graph.\n";
	
	// add background
	this->object_label_.push_back("g");
	this->background_label_ = "g";
	this->object_point_cloud_["g"] = background_image;
	btTransform identity; identity.setIdentity();
	this->object_instance_parameter_["g"] = identity;
	
	// search for plane of support
	size_t background_point_size = background_image->points.size();
	if (background_point_size < 100)
	{
		// input is already a plane convex hull. Get the plane coefficient directly
		Eigen::Vector3f origin(background_image->points[0].x, 
			background_image->points[0].y, 
			background_image->points[0].z);

		size_t p1_index = background_point_size / 3;
		Eigen::Vector3f p1(background_image->points[p1_index].x, 
			background_image->points[p1_index].y, 
			background_image->points[p1_index].z);

		size_t p2_index = background_point_size* 2 / 3;
		Eigen::Vector3f p2(background_image->points[p2_index].x, 
			background_image->points[p2_index].y, 
			background_image->points[p2_index].z);

		Eigen::Vector3f o_p1 = origin - p1,
						o_p2 = origin - p2;
		Eigen::Vector3f normal = o_p1.cross(o_p2);
		float coeff = - normal.dot(origin);

		this->physics_engine_->addBackgroundPlane(convertEigenToBulletVector(normal), btScalar(coeff));
	}
}

void SceneGraph::addNewObjectTransforms(const std::vector<ObjectWithID> &objects)
{
	this->physics_engine_->resetObjects();
	if (this->debug_messages_) std::cerr <<"Adding new objects into the scene graph.\n";
	this->physics_engine_->addObjects(objects);
}

std::map<std::string, ObjectParameter> SceneGraph::getCorrectedObjectTransform()
{
	if (this->debug_messages_) std::cerr <<"Getting corrected object transform from the scene graph.\n";
	
	return this->physics_engine_->getUpdatedObjectPose();
	// std::map<std::string, ObjectParameter> result_eigen_pose;
	// for (std::map<std::string, btTransform>::const_iterator it = this->rigid_body_.begin(); 
	// 	it != this->rigid_body_.end(); ++it)
	// {
	// 	result_eigen_pose[it->first] = convertBulletToEigenTransform(it->second);
	// }
	// return result_eigen_pose;
}

void SceneGraph::setDebugMode(bool debug)
{
	this->debug_messages_ = debug;
}

SequentialSceneGraph::SequentialSceneGraph(Image background_image)
{
	SceneGraph background_scene;
	background_scene.addBackground(background_image);
	sequential_scene_graph_.push_back(background_scene);
}

DecisionVector SequentialSceneGraph::BDLinear(Image I, SceneGraph G_minus, SceneGraph G_bar, DecisionVector D, std::vector<std::string> O)
{
	if (O.size() == 0) return D;
	else
	{
		std::string &p = O.at(0);
		
	}
}