#include "scene_data_forces.h"
#include <pcl/io/pcd_io.h>

FeedbackDataForcesGenerator::FeedbackDataForcesGenerator() : 
	have_scene_data_(false), force_data_model_(CACHED_ICP_CORRESPONDENCE), 
	forces_magnitude_coefficient_(0.5), max_point_distance_threshold_(0.01),
	max_icp_iteration_(10)
{
	icp_.setMaximumIterations(max_icp_iteration_);
}

void FeedbackDataForcesGenerator::setFeedbackForceMode(int mode)
{
	force_data_model_ = mode;
}

void FeedbackDataForcesGenerator::applyFeedbackForces(btRigidBody &object, const std::string &model_name)
{
	if (!this->have_scene_data_)
	{
		std::cerr << "Cannot generate data force, since input scene cloud is not available yet.\n";
	}

	if (keyExistInConstantMap(model_name, model_cloud_map_))
	{
		std::pair<btVector3, btVector3> force_and_torque = this->generateDataForce(object, 
			model_name);
		
		object.clearForces();
		object.applyGravity();
		object.applyCentralForce(force_and_torque.first);
		object.applyTorque(force_and_torque.second);
		// std::cerr << *(std::string*)object.getUserPointer() << " gets " << force_and_torque.first.norm()/SCALING << "N and " 
		// 	<< force_and_torque.second.norm() << " Nm.\n";
	}
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForce(const btRigidBody &object, 
	const std::string &model_name)
{
	const btTransform &object_pose = object.getCenterOfMassTransform();
	btTransform object_real_pose = rescaleTransformFromPhysicsEngine(object_pose);
	Eigen::Transform <float,3,Eigen::Affine > object_pose_eigen = convertBulletToEigenTransform<float>(object_real_pose);
	PointCloudXYZPtr transformed_object_mesh_cloud (new PointCloudXYZ);
	pcl::transformPointCloud(*(getContentOfConstantMap(model_name,model_cloud_map_)),
		*transformed_object_mesh_cloud, object_pose_eigen);

	switch(force_data_model_)
	{
		case CLOSEST_POINT:
			return this->generateDataForceWithClosestPointPair(transformed_object_mesh_cloud, object_real_pose);
			break;
		case FRAME_BY_FRAME_ICP_CORRESPONDENCE:
			return this->generateDataForceWithICP(transformed_object_mesh_cloud, object_real_pose);
			break;
		case CACHED_ICP_CORRESPONDENCE:
		{
			std::string object_id = getObjectIDFromCollisionObject(&object);
			return this->generateDataForceWithSavedICP(transformed_object_mesh_cloud, object_real_pose, object_id);
			break;
		}
		default:
			std::cerr << "Unrecognized data force model. \n";
			return std::pair<btVector3, btVector3>();
	}
}

void FeedbackDataForcesGenerator::updateCachedIcpResultMap(const btRigidBody &object, 
	const std::string &model_name)
{
	std::string object_id = getObjectIDFromCollisionObject(&object);
	const btTransform &object_pose = object.getCenterOfMassTransform();
	btTransform object_real_pose = rescaleTransformFromPhysicsEngine(object_pose);
	Eigen::Transform <float,3,Eigen::Affine > object_pose_eigen = convertBulletToEigenTransform<float>(object_real_pose);
	PointCloudXYZPtr transformed_object_mesh_cloud (new PointCloudXYZ);
	pcl::transformPointCloud(*(getContentOfConstantMap(model_name,model_cloud_map_)),
		*transformed_object_mesh_cloud, object_pose_eigen);

	model_cloud_icp_result_map_[object_id] = this->doICP(transformed_object_mesh_cloud);
}


void FeedbackDataForcesGenerator::manualSetCachedIcpResultMapFromPose(const btRigidBody &object, 
	const std::string &model_name)
{
	std::string object_id = getObjectIDFromCollisionObject(&object);
	const btTransform &object_pose = object.getCenterOfMassTransform();
	btTransform object_real_pose = rescaleTransformFromPhysicsEngine(object_pose);
	Eigen::Transform <float,3,Eigen::Affine > object_pose_eigen = convertBulletToEigenTransform<float>(object_real_pose);
	PointCloudXYZPtr transformed_object_mesh_cloud (new PointCloudXYZ);
	pcl::transformPointCloud(*(getContentOfConstantMap(model_name,model_cloud_map_)),
		*transformed_object_mesh_cloud, object_pose_eigen);

	model_cloud_icp_result_map_[object_id] = transformed_object_mesh_cloud;
}

void FeedbackDataForcesGenerator::resetCachedIcpResult()
{
	model_cloud_icp_result_map_.clear();
}

void FeedbackDataForcesGenerator::setSceneData(PointCloudXYZPtr scene_data)
{
	if (scene_data->size() > 0)
	{
		this->have_scene_data_ = true;
		this->scene_data_ = scene_data;
		this->scene_data_tree_.setInputCloud(scene_data);
	}
	else
	{
		std::cerr << "Input scene data has no points.\n";
		this->have_scene_data_ = false;
	}
}

void FeedbackDataForcesGenerator::setModelCloud(const PointCloudXYZPtr mesh_surface_sampled_cloud, 
	const std::string &model_name)
{
	if (mesh_surface_sampled_cloud->size() > 0)
	{
		this->model_cloud_map_[model_name] =  mesh_surface_sampled_cloud;
	}
	else
	{
		std::cerr << "Input model surface cloud for ["<< model_name <<"] has no points.\n";
	}
}

void FeedbackDataForcesGenerator::setForcesParameter(const btScalar &forces_magnitude_per_point, 
	const btScalar &max_point_distance_threshold)
{
	this->forces_magnitude_coefficient_ = forces_magnitude_per_point;
	this->max_point_distance_threshold_ = max_point_distance_threshold;
	// make this into parameter
	this->icp_.setMaxCorrespondenceDistance(0.005);
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForceWithClosestPointPair(
	PointCloudXYZPtr input_cloud, const btTransform &object_pose) const
{
	btVector3 total_forces, torque;
	const btVector3 &object_cog = object_pose.getOrigin();
	PointCloudXYZ nearest_point_correspondence_cloud;
	nearest_point_correspondence_cloud.height = input_cloud->height;
	nearest_point_correspondence_cloud.width = input_cloud->width;
	nearest_point_correspondence_cloud.is_dense = input_cloud->is_dense;
	nearest_point_correspondence_cloud.resize(input_cloud->size());

	for (int i = 0; i < input_cloud->size(); i++)
	{
		std::vector< int > k_indices(1);
		std::vector< float > distances(1);
		this->scene_data_tree_.nearestKSearch(*input_cloud, i, 1, k_indices, distances);
		nearest_point_correspondence_cloud.points[i] = scene_data_->points[k_indices[0]];
	}

	return this->calculateDataForceFromCorrespondence(*input_cloud, nearest_point_correspondence_cloud, object_cog);
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForceWithICP(PointCloudXYZPtr input_cloud,
	const btTransform &object_pose) const
{
	PointCloudXYZPtr icp_result = doICP(input_cloud);
	const btVector3 &object_cog = object_pose.getOrigin();
	return this->calculateDataForceFromCorrespondence(*input_cloud, *icp_result, object_cog);
}


std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForceWithSavedICP(PointCloudXYZPtr input_cloud,
	const btTransform &object_pose, const std::string object_id)
{
	if (object_id == "unrecognized_object")
	{
		std::cerr << "Unrecognized object id.\n";
		return std::pair<btVector3, btVector3>();
	}
	// do ICP if the ICP result of the object id has not recorded yet
	if (!keyExistInConstantMap(object_id, model_cloud_icp_result_map_))
	{
		model_cloud_icp_result_map_[object_id] = this->doICP(input_cloud);
	}

	PointCloudXYZPtr icp_result = model_cloud_icp_result_map_[object_id];
	const btVector3 &object_cog = object_pose.getOrigin();
	return this->calculateDataForceFromCorrespondence(*input_cloud, *icp_result, object_cog);
}

PointCloudXYZPtr FeedbackDataForcesGenerator::doICP(const PointCloudXYZPtr input_cloud) const
{
	PointCloudXYZPtr icp_result(new PointCloudXYZ);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp = icp_;
	// icp.setMaximumIterations(this->max_icp_iteration_);
	// icp.setMaxCorrespondenceDistance(this->max_point_distance_threshold_);
	icp.setInputSource(input_cloud);
	icp.setInputTarget(scene_data_);
	icp.align(*icp_result);
	return icp_result;
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::calculateDataForceFromCorrespondence(
	const PointCloudXYZ input_cloud, const PointCloudXYZ target_cloud, const btVector3 &object_cog) const
{
	btVector3 total_forces, total_torque;
	int counter = 1;
	int max_cloud_size = input_cloud.size() > target_cloud.size() ? target_cloud.size() : input_cloud.size();
	for (int i = 0; i < max_cloud_size; i++)
	{
		const pcl::PointXYZ &point = input_cloud.points[i];
		const pcl::PointXYZ &target_point = target_cloud.points[i];
		btVector3 force_direction(target_point.x - point.x,
								  target_point.y - point.y,
								  target_point.z - point.z);
		double distance = force_direction.norm();
		if (distance < this->max_point_distance_threshold_)
		{
			counter++;
			// spring-like attraction force for point correspondence
			// the attraction force = force_magnitude_coefficient * distance**2
			btVector3 attraction_force = forces_magnitude_coefficient_ * distance * force_direction;
			total_forces += attraction_force;
			// total_torque += (btVector3(point.x, point.y, point.z) - object_cog).cross(attraction_force);
		}
	}
	
	btVector3 avg_forces = total_forces / counter;

	for (int i = 0; i < max_cloud_size; i++)
	{
		const pcl::PointXYZ &point = input_cloud.points[i];
		const pcl::PointXYZ &target_point = target_cloud.points[i];
		btVector3 force_direction(target_point.x - point.x,
								  target_point.y - point.y,
								  target_point.z - point.z);
		double distance = force_direction.norm();
		if (distance < this->max_point_distance_threshold_)
		{
			btVector3 attraction_force = forces_magnitude_coefficient_ * distance * force_direction;
			total_torque += (btVector3(point.x, point.y, point.z) - object_cog).cross(attraction_force - avg_forces);
		}
	}

	total_forces *= SCALING;
	total_torque *= SCALING * SCALING;

	return std::make_pair(total_forces, total_torque);
}

