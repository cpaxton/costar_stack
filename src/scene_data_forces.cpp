#include "scene_data_forces.h"
#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

int idx = 0;
inline
btVector3 attractionForceModel(const btVector3 &force_vector, 
	const btScalar &distance, const btScalar &distance_threshold)
{
	// y = -(x - 0)( x - 2 * distance_threshold )
	// dy/dx = 2x - x * 2 distance_thres = 0
	// x = distance_thres
	// max_y = distance_threshold * distance_threshold
	// y_reg = - x * (x - 2*distance_threshold)/distance_threshold^2
	return force_vector * (2*distance_threshold - distance)/(distance_threshold*distance_threshold);
}

FeedbackDataForcesGenerator::FeedbackDataForcesGenerator() : 
	debug_(false),
	have_scene_data_(false), force_data_model_(CACHED_ICP_CORRESPONDENCE), 
	percent_gravity_max_correction_(0.5), max_point_distance_threshold_(0.01),
	max_icp_iteration_(10)
{
	icp_.setMaximumIterations(max_icp_iteration_);
}

bool FeedbackDataForcesGenerator::setModelDirectory(const std::string &model_directory)
{
	namespace fs = boost::filesystem;
	fs::path directory_path(model_directory);
	if (fs::is_directory(directory_path))
	{
		model_directory_ = model_directory;
		return true;
	}
	else
	{
		std::cerr << "Error, object model directory: " << model_directory << " does not exist.\n";
		return false;
	}
}

bool FeedbackDataForcesGenerator::setModelCloud(const std::string &model_name)
{
	namespace fs = boost::filesystem;
	fs::path directory_path(model_directory_);
	fs::path model_path = directory_path / model_name;
	model_path.replace_extension(".pcd");

	if (fs::exists(model_path))
	{
		pcl::PCDReader reader;
		pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>());
		if (reader.read(model_path.string(),*surface_cloud) == 0)
		{
			this->setModelCloud(surface_cloud, model_name);
			std::cerr << "Model point cloud loaded successfully\n";
			return true;
		}
		else
		{
			std::cerr << "Failed to load " << model_name << " model point cloud\n";
		}
	}
	else
	{
		std::cerr << "Error: '" << model_path << "' does not exist.\n";	 
	}
	return false;
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
	
	if (percent_gravity_max_correction_ == 0.0)
	{
		return;
	}

	if (keyExistInConstantMap(model_name, model_cloud_map_))
	{
		std::pair<btVector3, btVector3> force_and_torque = this->generateDataForce(object, 
			model_name);
		
		if (model_forces_scale_map_.find(model_name) == model_forces_scale_map_.end())
		{
			model_forces_scale_map_[model_name] = object.getInvMass() * percent_gravity_max_correction_ * gravity_force_per_point_[model_name];
		}

		object.clearForces();
		object.applyGravity();
		object.applyCentralForce(force_and_torque.first * model_forces_scale_map_[model_name]);
		object.applyTorque(force_and_torque.second * model_forces_scale_map_[model_name]);
		// std::cerr << *(std::string*)object.getUserPointer() << " gets " << force_and_torque.first.norm()/SCALING << "N and " 
		// 	<< force_and_torque.second.norm() << " Nm.\n";
	}
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForce(const btRigidBody &object, 
	const std::string &model_name)
{
	std::string object_id = getObjectIDFromCollisionObject(&object);
	if (object_id == "unrecognized_object")
	{
		std::cerr << "Unrecognized object id.\n";
		return std::pair<btVector3, btVector3>();
	}

	btTransform object_real_pose;
	PointCloudXYZPtr transformed_object_mesh_cloud = this->getTransformedObjectCloud(object, 
		model_name, object_real_pose);

	switch(force_data_model_)
	{
		case CLOSEST_POINT:
			return this->generateDataForceWithClosestPointPair(transformed_object_mesh_cloud, object_real_pose);
			break;
		case FRAME_BY_FRAME_ICP_CORRESPONDENCE:
			return this->generateDataForceWithICP(transformed_object_mesh_cloud, object_real_pose, object_id);
			break;
		case CACHED_ICP_CORRESPONDENCE:
		{
			return this->generateDataForceWithSavedICP(transformed_object_mesh_cloud, object_real_pose, object_id);
			break;
		}
		default:
			std::cerr << "Unrecognized data force model. \n";
			return std::pair<btVector3, btVector3>();
	}
}

void FeedbackDataForcesGenerator::updateCachedIcpResultMap(const PointCloudXYZPtr icp_result, 
	const std::string &object_id)
{
	if (icp_result->empty())
	{
		if (debug_)std::cerr << "ERROR, trying to update cloud of object [" << object_id << "] with empty cloud\n";
		return;
	}
	model_cloud_icp_result_map_[object_id] = icp_result;
	icp_result_confidence_map_[object_id] = getIcpConfidenceResult(icp_result);
	// std::string filename = "result_icp_"+object_id+"_"+boost::lexical_cast<std::string>(idx++)+".pcd";
	// boost::replace_all(filename,"seg/","");
	// pcl::io::savePCDFile(filename,*model_cloud_icp_result_map_[object_id],true);
}

double FeedbackDataForcesGenerator::getIcpConfidenceResult(const std::string &model_name, const btTransform &object_pose)
{
	btTransform dummy_transform;
	if (!keyExistInConstantMap(model_name,model_cloud_map_))
	{
		std::cerr << "ERROR, model name '" << model_name << "' does not exist in the database.\n";
		std::cerr << "Attempting to load Unrecognized model\n";
		bool load_success = this->setModelCloud(model_name);
		if (!load_success)
		{
			return 0;
		}
	}

	PointCloudXYZPtr dummy =  getTransformedObjectCloud(object_pose, model_name, dummy_transform);
	return getIcpConfidenceResult(dummy);
}

void FeedbackDataForcesGenerator::updateCachedIcpResultMap(const btRigidBody &object, 
	const std::string &model_name)
{
	std::string object_id = getObjectIDFromCollisionObject(&object);
	PointCloudXYZPtr transformed_object_mesh_cloud = this->getTransformedObjectCloud(object, model_name);

	PointCloudXYZPtr icp_result = this->doICP(transformed_object_mesh_cloud);
	this->updateCachedIcpResultMap(icp_result, object_id);
}


void FeedbackDataForcesGenerator::manualSetCachedIcpResultMapFromPose(const btRigidBody &object, 
	const std::string &model_name)
{
	std::string object_id = getObjectIDFromCollisionObject(&object);
	PointCloudXYZPtr transformed_object_mesh_cloud = this->getTransformedObjectCloud(object, model_name);

	this->updateCachedIcpResultMap(transformed_object_mesh_cloud, object_id);
	// std::string filename = "manual_"+object_id+"_"+boost::lexical_cast<std::string>(idx)+".pcd";
	// boost::replace_all(filename,"seg/","");
	// if (!model_cloud_icp_result_map_[object_id]->empty())pcl::io::savePCDFile(filename,*model_cloud_icp_result_map_[object_id],true);
}

void FeedbackDataForcesGenerator::resetCachedIcpResult()
{
	model_cloud_icp_result_map_.clear();
	icp_result_confidence_map_.clear();
}

void FeedbackDataForcesGenerator::removeCachedIcpResult(const std::string &object_id)
{
	model_cloud_icp_result_map_.erase(object_id);
}

void FeedbackDataForcesGenerator::setSceneData(PointCloudXYZPtr scene_data)
{
	if (!scene_data->empty())
	{
		this->have_scene_data_ = true;

		this->scene_data_ = scene_data;
		// make a new scene data tree to ensure safer variable deletion
		scene_data_tree_ = pcl::KdTreeFLANN<pcl::PointXYZ>();
		this->scene_data_tree_.setInputCloud(scene_data);
		// pcl::io::savePCDFile("scene_data_"+boost::lexical_cast<std::string>(idx)+".pcd",*scene_data,true);
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
		PointCloudXYZPtr downsampled_scene_data(new PointCloudXYZ());
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud(mesh_surface_sampled_cloud);
		sor.setLeafSize(0.003f,0.003f,0.003f);
		sor.filter(*downsampled_scene_data);
		this->model_cloud_map_[model_name] =  downsampled_scene_data;
		this->gravity_force_per_point_[model_name] = SCALED_GRAVITY_MAGNITUDE / mesh_surface_sampled_cloud->size();
	}
	else
	{
		std::cerr << "Input model surface cloud for ["<< model_name <<"] has no points.\n";
	}
}

void FeedbackDataForcesGenerator::setForcesParameter(const btScalar &forces_magnitude_per_point, 
	const btScalar &max_point_distance_threshold)
{
	this->percent_gravity_max_correction_ = forces_magnitude_per_point;
	this->max_point_distance_threshold_ = max_point_distance_threshold;
	// make this into parameter
	this->icp_.setMaxCorrespondenceDistance(0.005);
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForceWithClosestPointPair(
	PointCloudXYZPtr input_cloud, const btTransform &object_pose) const
{
	btVector3 total_forces, torque;
	const btVector3 &object_cog = object_pose.getOrigin();
	// std::cerr << "Generating correspondence cloud\n";
	PointCloudXYZPtr nearest_point_correspondence_cloud = generateCorrespondenceCloud(input_cloud);
	// std::cerr << "Calculate data force\n";
	return this->calculateDataForceFromCorrespondence(input_cloud, nearest_point_correspondence_cloud, object_cog);
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForceWithICP(PointCloudXYZPtr input_cloud,
	const btTransform &object_pose, const std::string &object_id)
{
	PointCloudXYZPtr icp_result = doICP(input_cloud);
	this->updateCachedIcpResultMap(icp_result, object_id);
	const btVector3 &object_cog = object_pose.getOrigin();
	const double &icp_confidence = icp_result_confidence_map_[object_id];
	return this->calculateDataForceFromCorrespondence(input_cloud, icp_result, object_cog, icp_confidence);
}


std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::generateDataForceWithSavedICP(PointCloudXYZPtr input_cloud,
	const btTransform &object_pose, const std::string &object_id)
{
	// do ICP if the ICP result of the object id has not recorded yet
	if (!keyExistInConstantMap(object_id, model_cloud_icp_result_map_))
	{
		return this->generateDataForceWithICP(input_cloud, object_pose, object_id);
	}

	PointCloudXYZPtr icp_result = model_cloud_icp_result_map_[object_id];
	const btVector3 &object_cog = object_pose.getOrigin();
	const double &icp_confidence = icp_result_confidence_map_[object_id];
	return this->calculateDataForceFromCorrespondence(input_cloud, icp_result, object_cog, icp_confidence);
}

PointCloudXYZPtr FeedbackDataForcesGenerator::doICP(const PointCloudXYZPtr input_cloud) const
{
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp = icp_;

	PointCloudXYZPtr cloud_around_initial_estimate(new PointCloudXYZ());
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(input_cloud);
	feature_extractor.compute();

	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	// Eigen::Affine3f transform;
	// transform.prerotate(rotational_matrix_OBB);

	Eigen::Vector3f min_cropbox_pt(min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
	Eigen::Vector3f max_cropbox_pt(max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);

	// enlarge the box by max_point_distance_threshold
	Eigen::Vector3f offset(max_point_distance_threshold_,max_point_distance_threshold_,max_point_distance_threshold_);

	pcl::CropBox<pcl::PointXYZ> cropbox_filter;
	cropbox_filter.setInputCloud(scene_data_);
	cropbox_filter.setMin((min_cropbox_pt - offset).homogeneous());
	cropbox_filter.setMax((max_cropbox_pt + offset).homogeneous());
	// cropbox_filter.setTransform(transform.inverse());
	cropbox_filter.setTranslation(Eigen::Vector3f(position_OBB.x,position_OBB.y,position_OBB.z));
	cropbox_filter.setRotation(rotational_matrix_OBB.eulerAngles(2,1,0));

	cropbox_filter.filter(*cloud_around_initial_estimate);
	
	std::vector< int > unused;
	pcl::removeNaNFromPointCloud(*cloud_around_initial_estimate,*cloud_around_initial_estimate,unused);
	// std::cerr << "Cropboxed cloud size: " << cloud_around_initial_estimate->size() << std::endl;
	PointCloudXYZPtr icp_result(new PointCloudXYZ());
	// if (!cloud_around_initial_estimate->size() > 100)
	// 	pcl::io::savePCDFile("icp_"+boost::lexical_cast<std::string>(idx)+"_cropbox.pcd",*cloud_around_initial_estimate,true);
	if (!cloud_around_initial_estimate->size() < 100)
	{
		return icp_result;
	}

	icp.setInputSource(input_cloud);
	icp.setInputTarget(cloud_around_initial_estimate);
	// icp.setMaximumIterations(this->max_icp_iteration_);
	// icp.setMaxCorrespondenceDistance(this->max_point_distance_threshold_);

	icp.align(*icp_result);
	if (debug_)std::cerr << "has converged:" << icp.hasConverged() << " score: " <<  icp.getFitnessScore() << std::endl;
	return icp_result;
}

std::pair<btVector3, btVector3> FeedbackDataForcesGenerator::calculateDataForceFromCorrespondence(
	const PointCloudXYZPtr input_cloud, const PointCloudXYZPtr target_cloud, const btVector3 &object_cog,
	const double &icp_confidence) const
{
	// NOTE: DO NOT USE FILTERED CORRESPONDENCE CLOUD FOR THIS METHOD, SINCE THIS METHOD ASSUMES THAT
	// THE POINT INDICES ARE ALIGNED BETWEEN INPUT CLOUD AND TARGET CLOUD
	// FOR ICP PAIR, THE RESULTING FORCES NEED TO BE SCALED APPROPRIATELY

	btVector3 total_forces, total_torque;
	int counter = 1;
	int max_cloud_size = input_cloud->size() > target_cloud->size() ? target_cloud->size() : input_cloud->size();
	
	if (max_cloud_size == 0)
	{
		if (debug_)std::cerr << "Input cloud or target cloud size is 0\n";
		return std::pair<btVector3, btVector3>();
	}
	
	for (int i = 0; i < max_cloud_size; i++)
	{
		const pcl::PointXYZ &point = input_cloud->points[i];
		const pcl::PointXYZ &target_point = target_cloud->points[i];
		btVector3 force_vector(target_point.x - point.x,
							   target_point.y - point.y,
							   target_point.z - point.z);
		double distance = force_vector.norm();
		if (distance < 2 * this->max_point_distance_threshold_)
		{
			btVector3 attraction_force = attractionForceModel(force_vector, 
				distance, this->max_point_distance_threshold_);
			total_forces += attraction_force;
			total_torque += (btVector3(point.x, point.y, point.z) - object_cog).cross(attraction_force);	
		}
	}

	total_forces *= icp_confidence;
	total_torque *= icp_confidence * SCALING;

	return std::make_pair(total_forces, total_torque);
}


PointCloudXYZPtr FeedbackDataForcesGenerator::generateCorrespondenceCloud(PointCloudXYZPtr input_cloud, 
	const bool &filter_distance, const double &max_squared_distance) const
{
	// Get the correspondence cloud to the input scene
	bool use_filter_distance = (max_squared_distance > 0 && filter_distance);

	PointCloudXYZPtr nearest_point_correspondence_cloud(new PointCloudXYZ);
	nearest_point_correspondence_cloud->height = input_cloud->height;
	nearest_point_correspondence_cloud->is_dense = input_cloud->is_dense;
	nearest_point_correspondence_cloud->reserve(input_cloud->size());
	
	float bad_pt = std::numeric_limits<float>::quiet_NaN();

	pcl::PointXYZ nan_point(bad_pt,bad_pt,bad_pt);

	for (int i = 0; i < input_cloud->size(); i++)
	{
		std::vector< int > k_indices(1);
		std::vector< float > distances(1);

		// any neighbors are found within appropriate distance
		if (
			(this->scene_data_tree_.nearestKSearch(*input_cloud, i, 1, k_indices, distances) > 0) &&
			(!use_filter_distance || (use_filter_distance && distances[0] < max_squared_distance))
			)
		{
			nearest_point_correspondence_cloud->points.push_back(scene_data_->points[k_indices[0]]);
		}
		else
		{
			nearest_point_correspondence_cloud->points.push_back(nan_point);
		}
	}

	nearest_point_correspondence_cloud->width = nearest_point_correspondence_cloud->size();
	// pcl::io::savePCDFile("icp_res_correspondence.pcd",*nearest_point_correspondence_cloud,true);
	return nearest_point_correspondence_cloud;
}

double FeedbackDataForcesGenerator::getIcpConfidenceResult(const PointCloudXYZPtr icp_result,
	const double &voxel_size) const
{
	// If voxel size used is 3mm, the max distance need to be around 0.5 * (3 * sqrt(3)) mm.
	PointCloudXYZPtr nearest_point_correspondence_cloud = generateCorrespondenceCloud(icp_result, true, voxel_size * voxel_size * 1.5);
	return double(nearest_point_correspondence_cloud->size())/icp_result->size();
}

PointCloudXYZPtr FeedbackDataForcesGenerator::getTransformedObjectCloud(const std::string &model_name, const btTransform &object_real_pose) const
{
	Eigen::Transform <float,3,Eigen::Affine > object_pose_eigen = convertBulletToEigenTransform<float>(object_real_pose);
	PointCloudXYZPtr transformed_object_mesh_cloud (new PointCloudXYZ);
	if (keyExistInConstantMap(model_name,model_cloud_map_))
	{
		pcl::transformPointCloud(*(getContentOfConstantMap(model_name,model_cloud_map_)),
			*transformed_object_mesh_cloud, object_pose_eigen);
	}
	else
	{
		std::cerr << "ERROR, model name '" << model_name << "' does not exist in the database. Fail to transform the object cloud.\n";
	}
	return transformed_object_mesh_cloud;
}

PointCloudXYZPtr FeedbackDataForcesGenerator::getTransformedObjectCloud(const btRigidBody &object, 
		const std::string &model_name) const
{
	btTransform dummy_transform;
	return this->getTransformedObjectCloud(object, model_name, dummy_transform);
}

PointCloudXYZPtr FeedbackDataForcesGenerator::getTransformedObjectCloud(const btRigidBody &object, 
	const std::string &model_name, btTransform &object_real_pose) const
{
	const btTransform &object_pose = object.getCenterOfMassTransform();
	return getTransformedObjectCloud(object_pose,model_name,object_real_pose);
}

PointCloudXYZPtr FeedbackDataForcesGenerator::getTransformedObjectCloud(const btTransform &object_pose, 
	const std::string &model_name, btTransform &object_real_pose) const
{
	object_real_pose = rescaleTransformFromPhysicsEngine(object_pose);
	return getTransformedObjectCloud(model_name, object_real_pose);
}
