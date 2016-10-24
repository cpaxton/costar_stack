#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "sequential_scene_parsing.h"
#include "utility.h"


SceneGraph::SceneGraph(Image input, Image background_image)
{
	this->addBackground(background_image);
	// this->processImage(input);
	this->physics_engine_ready_ = false;
}

void SceneGraph::setPhysicsEngine(PhysicsEngineWRender* physics_engine)
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
	
	// // search for plane of support
	// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// // Create the segmentation object
	// pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// // Optional
	// seg.setOptimizeCoefficients (true);
	// // Mandatory
	// seg.setModelType (pcl::SACMODEL_PLANE);
	// seg.setMethodType (pcl::SAC_RANSAC);
	// seg.setDistanceThreshold (0.01);

	// seg.setInputCloud (background_image);
	// seg.segment (*inliers, *coefficients);

	// if (inliers->indices.size () == 0)
	// {
	// 	std::cerr <<"Could not estimate a planar model for the given dataset.";
	// 	return;
	// }
	// Eigen::Vector3f normal(coefficients->values[0],coefficients->values[1], coefficients->values[2]);
	// float coeff = coefficients->values[3];

	// if (this->debug_messages_) std::cerr << "Background(plane) normal: "<< normal.transpose() <<", coeff: " << coeff << std::endl;
	// this->physics_engine_->addBackgroundPlane(convertEigenToBulletVector(normal), btScalar(coeff));
	std::vector<btVector3> convex_plane_points;
	convex_plane_points.reserve(background_image->size());
	for (std::size_t i = 0; i < background_image->size(); i++)
	{
		btVector3 convex_hull_point(background_image->points[i].x,background_image->points[i].y,background_image->points[i].z);
		convex_plane_points.push_back(convex_hull_point);
	}
	this->physics_engine_->addBackgroundPlane(convex_plane_points);
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