#include <iostream>

// For plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

// For creating mesh from point cloud input
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/conversions.h>

#include "sequential_scene_parsing.h"
#include "utility.h"


SceneGraph::SceneGraph(ImagePtr input, ImagePtr background_image)
{
	this->addBackground(background_image);
	// this->processImage(input);
	this->physics_engine_ready_ = false;
}

// void SceneGraph::setPhysicsEngine(PhysicsEngine* physics_engine)
void SceneGraph::setPhysicsEngine(PhysicsEngine* physics_engine)
{
	if (this->debug_messages_) std::cerr <<"Setting physics engine into the scene graph.\n";
	this->physics_engine_ = physics_engine;

	// Check if physics engine exist
	this->physics_engine_ready_ = (physics_engine_ != NULL);
	if (this->physics_engine_ready_)
	{
		this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME + RUN_UNTIL_HAVE_SUPPORT_GRAPH);
	}
}

void SceneGraph::addBackground(ImagePtr background_image, int mode)
{
	if (this->debug_messages_) std::cerr <<"Adding background into the scene graph.\n";
	
	// add background
	// this->object_label_.push_back("g");
	this->background_label_ = "g";
	this->object_point_cloud_["g"] = background_image;
	btTransform identity; identity.setIdentity();
	this->object_instance_parameter_["g"] = identity;

	// search for plane of support
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<ImagePoint> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (background_image);
	seg.segment (*inliers, *coefficients);
	Eigen::Vector3f normal(coefficients->values[0],coefficients->values[1], coefficients->values[2]);
	btVector3 normal_bt = convertEigenToBulletVector(normal);
	float coeff = coefficients->values[3];

	Eigen::Vector4f cloud_centroid;
	pcl::compute3DCentroid(*background_image,cloud_centroid);
	btVector3 bt_cloud_centroid(cloud_centroid[0],cloud_centroid[1],cloud_centroid[2]);

	switch (mode)
	{
		case BACKGROUND_PLANE:
		{
			if (inliers->indices.size () == 0)
			{
				std::cerr <<"Could not estimate a planar model for the given dataset.";
				return;
			}

			if (this->debug_messages_) std::cerr << "Background(plane) normal: "<< normal.transpose() <<", coeff: " << coeff << std::endl;
			this->physics_engine_->addBackgroundPlane(normal_bt * SCALING, -btScalar(coeff * SCALING),bt_cloud_centroid * SCALING);
			break;
		}
		case BACKGROUND_HULL:
		{
			std::vector<btVector3> convex_plane_points;
			convex_plane_points.reserve(background_image->size());
			for (std::size_t i = 0; i < background_image->size(); i++)
			{
				btVector3 convex_hull_point(background_image->points[i].x,background_image->points[i].y,background_image->points[i].z);
				convex_plane_points.push_back(convex_hull_point * SCALING);
			}
			this->physics_engine_->addBackgroundConvexHull(convex_plane_points, normal_bt * SCALING);
			break;
		}
		case BACKGROUND_MESH:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr background_no_color (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*background_image,*background_no_color);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (background_no_color);
			n.setInputCloud(background_no_color);
			n.setSearchMethod (tree);
			n.setKSearch (20);
			n.compute (*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields (*background_no_color, *normals, *cloud_with_normals);
			//* cloud_with_normals = cloud + normals

			// Create search tree*
			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud (cloud_with_normals);

			pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
			pcl::PolygonMesh triangles;

			// Set the maximum distance between connected points (maximum edge length)
			gp3.setSearchRadius (0.025);

			// Set typical values for the parameters
			gp3.setMu (2.5);
			gp3.setMaximumNearestNeighbors (100);
			gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			gp3.setMinimumAngle(M_PI/18); // 10 degrees
			gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			gp3.setNormalConsistency(false);

			// Get result
			gp3.setInputCloud (cloud_with_normals);
			gp3.setSearchMethod (tree2);
			gp3.reconstruct (triangles);

			// Additional vertex information
			std::vector<int> parts = gp3.getPartIDs();
			std::vector<int> states = gp3.getPointStates();

			pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2 (triangles.cloud,*mesh_cloud);
			btTriangleMesh* trimesh = new btTriangleMesh();
			
			for (std::vector<pcl::Vertices>::const_iterator it = triangles.polygons.begin(); it != triangles.polygons.end(); ++it )
			{
				trimesh->addTriangle( 
						pclPointToBulletVector(mesh_cloud->points[ it->vertices[0] ]) * SCALING,
						pclPointToBulletVector(mesh_cloud->points[ it->vertices[1] ]) * SCALING,
						pclPointToBulletVector(mesh_cloud->points[ it->vertices[2] ]) * SCALING 
					);
			}
			this->physics_engine_->addBackgroundMesh(trimesh,normal_bt * SCALING, bt_cloud_centroid * SCALING);

			break;
		}
		default:
			break;
	}
}

void SceneGraph::addScenePointCloud(ImagePtr scene_image)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_coordinates_only(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*scene_image, *point_coordinates_only);
	data_probability_check_.setPointCloudData(point_coordinates_only);
}

void SceneGraph::addNewObjectTransforms(const std::vector<ObjectWithID> &objects)
{
	this->physics_engine_->resetObjects();
	object_label_.reserve(objects.size());
	for (std::vector<ObjectWithID>::const_iterator it = objects.begin(); it != objects.end(); ++it)
	{
		object_label_.push_back(it->getID());
	}

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

void SceneGraph::getUpdatedSceneSupportGraph()
{
	this->scene_support_graph_ = this->physics_engine_->getUpdatedSceneGraph(this->vertex_map_);
}

void SceneGraph::setObjectHypothesesMap(std::map<std::string, std::vector<ObjectParameter> > &object_hypotheses_map)
{
	this->object_hypotheses_map_ = object_hypotheses_map;
}

double SceneGraph::evaluateObjectProbability(const std::string &object_label)
{
	vertex_t object_in_graph = this->vertex_map_[object_label];
	scene_support_vertex_properties &object_physics_status = this->scene_support_graph_[object_in_graph];
	btTransform &object_pose = object_physics_status.object_pose_;
	const double &stability_probability = object_physics_status.stability_penalty_;
	double support_probability = getObjectSupportContribution(object_physics_status);
	double collision_probability = getObjectCollisionPenalty(object_physics_status);
	double object_data_compliance = this->data_probability_check_.getConfidence(object_label, object_pose);
	double object_total_probability = object_data_compliance * support_probability * stability_probability * collision_probability;
	std::cerr << object_label << " probability: " << object_total_probability << std::endl;

	return object_total_probability;
}

bool SceneGraph::evaluateObjectHypothesis(const std::string &object_label, const btTransform &object_pose_hypothesis)
{
	this->physics_engine_->prepareSimulationForOneTestHypothesis(object_label, object_pose_hypothesis);
	this->getUpdatedSceneSupportGraph();
	return this->evaluateObjectProbability(object_label);
}

void SceneGraph::evaluateAllObjectHypothesisProbability()
{
	for (std::map<std::string, std::vector<ObjectParameter> >::const_iterator it = this->object_hypotheses_map_.begin();
		it != this->object_hypotheses_map_.end(); ++it)
	{
		const std::string &object_label = it->first;
		const std::vector<ObjectParameter> &object_pose_hypotheses = it->second;
		for (std::vector<ObjectParameter>::const_iterator it2 = object_pose_hypotheses.begin();
			it2 != object_pose_hypotheses.end(); ++it2)
		{
			const ObjectParameter &object_pose = *it2;
			this->evaluateObjectHypothesis(object_label, object_pose);
		}
	}
}


