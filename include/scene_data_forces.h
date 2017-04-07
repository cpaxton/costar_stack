#ifndef SCENE_PHYSICS_DATA_FORCES
#define SCENE_PHYSICS_DATA_FORCES

#include <iostream>
#include <btBulletDynamicsCommon.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "utility.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;

class FeedbackDataForcesGenerator
{
public:
	FeedbackDataForcesGenerator() : forces_magnitude_coefficient_per_point_(0.5), max_point_distance_threshold_(0.01)
	{}

	void applyFeedbackForces(btRigidBody &object, const std::string &model_name);

	// Generate feedback central forces and torque based on model distance to the cloud
	std::pair<btVector3, btVector3>  generateDataForce(const btTransform &object_pose, const std::string &model_name);
	void setSceneData(PointCloudXYZPtr scene_data);

	// Input sampled point cloud from the mesh
	void setModelCloud(const PointCloudXYZPtr mesh_surface_sampled_cloud, const std::string &model_name);
	void setForcesParameter(const btScalar &forces_magnitude_per_point, const btScalar &max_point_distance_threshold);

private:
	// PointCloudXYZPtr generatePointCloudXYZFromCollisionObject(const btCollisionObject &object_collision_shape);

	PointCloudXYZPtr scene_data_;
	pcl::KdTreeFLANN<pcl::PointXYZ> scene_data_tree_;
	std::map<std::string, PointCloudXYZPtr> model_cloud_map_;
	btScalar forces_magnitude_coefficient_per_point_;
	btScalar max_point_distance_threshold_;
};

#endif