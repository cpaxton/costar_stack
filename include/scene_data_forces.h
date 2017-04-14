#ifndef SCENE_PHYSICS_DATA_FORCES
#define SCENE_PHYSICS_DATA_FORCES

#include <iostream>
#include <btBulletDynamicsCommon.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
// #include <pcl/recognition/ransac_based/trimmed_icp.h>
#include "utility.h"

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

// Mode for generating the feedback force based on point pair
enum FeedbackForceMode {
	// Find the closest point to each mesh point for all mesh point
	CLOSEST_POINT, 

	// Find the ICP correspondence from the mesh to the scene points for every frame
	FRAME_BY_FRAME_ICP_CORRESPONDENCE,

	// Use cached icp result for generating feedback force instead of doing icp for every frame
	CACHED_ICP_CORRESPONDENCE
};

class FeedbackDataForcesGenerator
{
public:
	FeedbackDataForcesGenerator();
	
	void setFeedbackForceMode(int mode);
	void applyFeedbackForces(btRigidBody &object, const std::string &model_name);

	// Generate feedback central forces and torque based on model distance to the cloud
	std::pair<btVector3, btVector3>  generateDataForce(
		const btRigidBody &object, const std::string &model_name);
	void setSceneData(PointCloudXYZPtr scene_data);

	// Input sampled point cloud from the mesh
	void setModelCloud(const PointCloudXYZPtr mesh_surface_sampled_cloud, const std::string &model_name);
	void setForcesParameter(const btScalar &forces_magnitude_per_point, const btScalar &max_point_distance_threshold);
	void resetCachedIcpResult();
	void updateCachedIcpResultMap(const btRigidBody &object, 
		const std::string &model_name);
	void manualSetCachedIcpResultMapFromPose(const btRigidBody &object, 
		const std::string &model_name);

	int force_data_model_;

private:
	std::pair<btVector3, btVector3> calculateDataForceFromCorrespondence(
	const PointCloudXYZ input_cloud, const PointCloudXYZ target_cloud, const btVector3 &object_cog) const;
	PointCloudXYZPtr doICP(const PointCloudXYZPtr input_cloud) const;
	std::pair<btVector3, btVector3> generateDataForceWithClosestPointPair(PointCloudXYZPtr input_cloud,
		const btTransform &object_pose) const;
	std::pair<btVector3, btVector3> generateDataForceWithICP(PointCloudXYZPtr input_cloud,
		const btTransform &object_pose) const;
	
	std::pair<btVector3, btVector3> generateDataForceWithSavedICP(PointCloudXYZPtr input_cloud,
		const btTransform &object_pose, const std::string object_id);

	bool have_scene_data_;
	PointCloudXYZPtr scene_data_;
	pcl::KdTreeFLANN<pcl::PointXYZ> scene_data_tree_;
	std::map<std::string, PointCloudXYZPtr> model_cloud_map_;
	std::map<std::string, PointCloudXYZPtr> model_cloud_icp_result_map_;
	btScalar forces_magnitude_coefficient_;
	btScalar max_point_distance_threshold_;
	int max_icp_iteration_;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
};

#endif