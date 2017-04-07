#include "scene_data_forces.h"

void FeedbackDataForcesGenerator::applyFeedbackForces(btRigidBody &object, const std::string &model_name)
{
	if (keyExistInConstantMap(model_name, model_cloud_map_))
	{
		std::pair<btVector3, btVector3> force_and_torque = this->generateDataForce(object.getCenterOfMassTransform(), 
			model_name);
		object.applyCentralForce(force_and_torque.first);
		object.applyTorque(force_and_torque.second);
		// std::cerr << *(std::string*)object.getUserPointer() << " gets " << force_and_torque.first.norm()/SCALING << "N and " 
		// 	<< force_and_torque.second.norm() << " Nm.\n";
	}
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForce(const btTransform &object_pose, 
	const std::string &model_name)
{
	btTransform object_real_pose = rescaleTransformFromPhysicsEngine(object_pose);
	Eigen::Transform <float,3,Eigen::Affine > object_pose_eigen = convertBulletToEigenTransform<float>(object_real_pose);
	PointCloudXYZPtr transformed_object_mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*model_cloud_map_[model_name],*transformed_object_mesh_cloud,object_pose_eigen);
	btVector3 total_forces, torque;
	btVector3 body_cog_position = object_real_pose.getOrigin();
	int counter = 0;
	double avg_distance = 0;

	for (int i = 0; i < transformed_object_mesh_cloud->size(); i++)
	{
		std::vector< int > k_indices(1);
		std::vector< float > distances(1);
		this->scene_data_tree_.nearestKSearch(*transformed_object_mesh_cloud, i, 1, k_indices, distances);
		if (distances[0] < max_point_distance_threshold_)
		{
			counter++;avg_distance += distances[0];
			const pcl::PointXYZ &point = transformed_object_mesh_cloud->points[i];
			const pcl::PointXYZ &nearest_point_result = scene_data_->points[k_indices[0]];
			btVector3 force_direction(point.x - nearest_point_result.x,
									  point.y - nearest_point_result.y,
									  point.z - nearest_point_result.z);
			// the attraction force = distance**2 * force_magnitude_coefficient
			btVector3 point_attraction_force = - (forces_magnitude_coefficient_per_point_ * distances[0]) * force_direction;
			torque += (btVector3(point.x, point.y, point.z) - body_cog_position).cross(point_attraction_force);
			total_forces += point_attraction_force;
		}
	}
	total_forces *= SCALING;
	torque *= SCALING * SCALING;

	// std::cerr << counter << "/" << transformed_object_mesh_cloud->size() 
	// 	<< " points are used to calculate data forces. Average distance:" << avg_distance / counter << std::endl;
	return std::make_pair(total_forces, torque);
}

void FeedbackDataForcesGenerator::setSceneData(PointCloudXYZPtr scene_data)
{
	this->scene_data_ = scene_data;
	this->scene_data_tree_.setInputCloud(scene_data);
}

void FeedbackDataForcesGenerator::setModelCloud(const PointCloudXYZPtr mesh_surface_sampled_cloud, 
	const std::string &model_name)
{
	this->model_cloud_map_[model_name] =  mesh_surface_sampled_cloud;
}

void FeedbackDataForcesGenerator::setForcesParameter(const btScalar &forces_magnitude_per_point, 
	const btScalar &max_point_distance_threshold)
{
	this->forces_magnitude_coefficient_per_point_ = forces_magnitude_per_point;
	this->max_point_distance_threshold_ = max_point_distance_threshold;
}
