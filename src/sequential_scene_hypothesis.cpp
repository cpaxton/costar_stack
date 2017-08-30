#include "sequential_scene_hypothesis.h"

SceneObservation::SceneObservation(const SceneHypothesis &final_scene_hypothesis,
	const OneFrameSceneHypotheses &input, 
	const std::map<std::string, std::string> &object_label_class_map) : 
		best_scene_hypothesis_(final_scene_hypothesis),
		scene_hypotheses_list_(input),
		object_label_class_map_(object_label_class_map), is_empty(false)
{
	std::sort(this->scene_hypotheses_list_.begin(),this->scene_hypotheses_list_.end(),compare_greater);
	// this->best_scene_hypothesis_ = this->scene_hypotheses_list_[0];
}

SceneObservation::SceneObservation(const SceneHypothesis &final_scene_hypothesis,
	const std::map<std::string, std::string> &object_label_class_map) : 
	best_scene_hypothesis_(final_scene_hypothesis),
	object_label_class_map_(object_label_class_map), is_empty(false)
{
	scene_hypotheses_list_.push_back(final_scene_hypothesis);
}

SequentialSceneHypothesis::SequentialSceneHypothesis() : minimum_data_probability_threshold_(0.1),
	max_static_object_translation_(0.0075), max_static_object_rotation_(5.)
{
	// These action implies these objects are not present in previous scene
	num_of_hypotheses_to_add_each_action_[INVALID_ACTION] = 0;
	num_of_hypotheses_to_add_each_action_[ADD_OBJECT] = 0;
	num_of_hypotheses_to_add_each_action_[REMOVE_OBJECT] = 0;

	num_of_hypotheses_to_add_each_action_[PERTURB_OBJECT] = 10;
	num_of_hypotheses_to_add_each_action_[STATIC_OBJECT] = 3;
	num_of_hypotheses_to_add_each_action_[SUPPORT_RETAINED_OBJECT] = 10;
	max_static_object_rotation_ *= boost::math::constants::pi<double>()/180.;

	// default kinect intrinsic parameter
	this->setCameraMatrix(554.254691191187,554.254691191187,320.5,240.5);
}

void SequentialSceneHypothesis::setCurrentDataSceneStructure(
	const SceneHypothesis &current_best_data_scene_structure)
{
	this->current_best_data_scene_structure_ = current_best_data_scene_structure;
}

void SequentialSceneHypothesis::setPreviousSceneObservation(const SceneObservation &previous_scene)
{
	this->previous_scene_observation_ = previous_scene;
}

// void SequentialSceneHypothesis::setObjRansacTool(ObjRecRANSACTool &data_probability_check)
// {
// 	this->data_probability_check_ = &data_probability_check;
// }

void SequentialSceneHypothesis::setConfidenceCheckTool(FeedbackDataForcesGenerator &data_probability_check)
{
	this->data_probability_check_ = &data_probability_check;
}

void SequentialSceneHypothesis::setStaticObjectThreshold(const double &translation_meter, 
	const double &rotation_degree)
{
	max_static_object_translation_ = translation_meter;
	max_static_object_rotation_ = rotation_degree * boost::math::constants::pi<double>()/180.;
}

void SequentialSceneHypothesis::setCameraMatrix(const double &fx, const double &fy, const double &cx, const double &cy)
{
	Eigen::Matrix3d camera_intrinsic;
	camera_intrinsic(0,0) = fx;
	camera_intrinsic(1,1) = fy;
	camera_intrinsic(0,2) = cx;
	camera_intrinsic(1,2) = cy;
	camera_intrinsic(2,2) = 1;
	Eigen::Matrix<double,3,4> camera_extrinsic;
	camera_extrinsic << 
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0; 
	backprojection_matrix_ = camera_intrinsic * camera_extrinsic;
}


void SequentialSceneHypothesis::setSceneData(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud)
{
	this->scene_point_cloud_ = scene_point_cloud;
	// backproject point cloud to image
	Eigen::MatrixXd pixel_matrix = backprojectCloudtoPixelCoordinate(scene_point_cloud);
	this->scene_image_pixel_ = generate2dMatrixFromPixelCoordinate(pixel_matrix);
}

void SequentialSceneHypothesis::setObjDatabasePtr(ObjectDatabase * obj_database)
{
	this->obj_database_ = obj_database;
}

void SequentialSceneHypothesis::setPhysicsEngine(PhysicsEngine* physics_engine)
{
	this->physics_engine_ = physics_engine;
}

Eigen::MatrixXd SequentialSceneHypothesis::generate2dMatrixFromPixelCoordinate(const Eigen::MatrixXd &pixel_matrix)
{
	Eigen::MatrixXd result = Eigen::MatrixXd::Zero(640,480);
	for (std::size_t idx = 0; idx < pixel_matrix.cols(); ++idx)
	{
		// fill the scene image with the depth information
		int x = pixel_matrix(0,idx);
		int y = pixel_matrix(1,idx);
		const double &z = pixel_matrix(2,idx);

		if (x >= 640 || y >= 480 || x < 0 || y < 0) continue;

		double &current_pixel_value = result(x,y);
		if (z == 0 || (current_pixel_value != 0 && current_pixel_value < z) ) continue;
		current_pixel_value = z;
	}
	return result;
}

Eigen::MatrixXd SequentialSceneHypothesis::backprojectCloudtoPixelCoordinate(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud)
{
	Eigen::MatrixXd point_cloud_in_matrix(4,input_cloud->size());
	std::size_t idx = 0;
	for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = input_cloud->begin();
		it != input_cloud->end(); ++it, ++idx)
	{
		point_cloud_in_matrix(0,idx) = it->x;
		point_cloud_in_matrix(1,idx) = it->y;
		point_cloud_in_matrix(2,idx) = it->z;
		point_cloud_in_matrix(3,idx) = 1;
	}

	Eigen::MatrixXd result_pixel_matrix = backprojection_matrix_ * point_cloud_in_matrix;
	
	for (std::size_t idx = 0; idx < input_cloud->size(); ++idx)
	{
		if (result_pixel_matrix(2,idx) == 0) continue;

		result_pixel_matrix(0,idx) /= result_pixel_matrix(2,idx);
		result_pixel_matrix(1,idx) /= result_pixel_matrix(2,idx);
		// std::cerr << result_pixel_matrix(0,idx) << " " << result_pixel_matrix(1,idx)
		// << " " << result_pixel_matrix(2,idx) << std::endl; 	
	}

	return result_pixel_matrix;
}

int SequentialSceneHypothesis::updateRemovedObjectStatus(const std::string &object_name,
	const std::string &model_name, const btTransform &object_pose)
{
	enum objectRemovedStatus{REMOVED, NOT_REMOVED, EPHEMERAL};

	// object is not detected by pose estimator
	std::cerr << object_name;
	// bool object_visible = checkObjectVisible(model_name,object_pose);
	// if (object_visible)
	// {
	// 	std::cerr << " is visible, thus it will not be removed.\n";
	// 	return NOT_REMOVED;
	// }
	// else
	// {
	// 	std::cerr << " is not visible,";
	// }

	bool object_obstructed = checkObjectObstruction(model_name,object_pose);
	
	if (!object_obstructed)
	{
		std::cerr << object_name << " is not obstructed, but also not visible, thus it will be removed.\n";
		return REMOVED;
	}

	bool object_replaced = checkObjectReplaced(object_name,model_name,object_pose);
	if (!object_replaced) return EPHEMERAL;
	else return REMOVED;
}

bool SequentialSceneHypothesis::checkObjectVisible(const std::string &model_name, const btTransform &object_pose)
{
	// visible if minimum 5% of the surface is visible
	std::cerr << " visibility: " << this->data_probability_check_->getIcpConfidenceResult(model_name, object_pose);
	return (this->data_probability_check_->getIcpConfidenceResult(model_name, object_pose) > 0.20);
}

bool SequentialSceneHypothesis::checkObjectObstruction(const std::string &model_name, const btTransform &object_pose)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud = this->data_probability_check_->getTransformedObjectCloud(model_name,
		rescaleTransformFromPhysicsEngine(object_pose));
	if (model_cloud->size() == 0)
	{
		std::cerr << " has invalid object model name input:'" << model_name << "'\n. Fail to check occlusion.\n";
		return false;
	}
	Eigen::MatrixXd backprojected_object_cloud = backprojectCloudtoPixelCoordinate(model_cloud);
	Eigen::MatrixXd object_image_matrix = generate2dMatrixFromPixelCoordinate(backprojected_object_cloud);
	
	std::size_t num_point_obstructed = 0;

	for (std::size_t idx = 0; idx < backprojected_object_cloud.cols(); ++idx)
	{
		int x = backprojected_object_cloud(0,idx);
		int y = backprojected_object_cloud(1,idx);
		const double &z = object_image_matrix(x,y);
		
		if (x >= 640 || y >= 480) continue;
		
		// tolerance 1e-4 for obstruction
		if (this->scene_image_pixel_(x,y) > 0 && this->scene_image_pixel_(x,y) < z)
		{
			++num_point_obstructed;
		}
	}

	std::cerr << " number of obstructed points: " << num_point_obstructed << "/" << model_cloud->size() << std::endl;
	return num_point_obstructed > 0.6 * model_cloud->size();
}

bool SequentialSceneHypothesis::checkObjectReplaced(const std::string &object_name,
	const std::string &model_name, const btTransform &object_pose)
{
	// use AABB to check if the original object volume has been occupied by something else
	// TODO: IMPLEMENT THIS!!
	// use bullet contactTest;
	if (!obj_database_)
	{
		std::cerr << "Error! Cannot check for replaced objects since no object database has been set.\n";
		return true;
	}
	else if (!physics_engine_)
	{
		std::cerr << "Error! Cannot check for replaced objects since Physics engine has not been set yet.\n";
		return true;
	}

	std::cerr << "Checking replaced objects.\n";
	if (!obj_database_->objectExistInDatabase(model_name))
	{
		std::cerr << "Object " << model_name << " does not exist in database.\n";
		return true;
	}

	// check if the object already exist in the world
	btCollisionObject* col_obj;
	col_obj = new btCollisionObject();
	Object target_model = obj_database_->getObjectProperty(model_name);
	col_obj->setCollisionShape(target_model.getCollisionShape());
	col_obj->setWorldTransform(object_pose);
	
	OverlappingObjectSensor result(*col_obj, object_name);
	this->physics_engine_->contactTest(col_obj,result);

	delete col_obj;
	std::cerr << "Overlaps: " << result.total_penetration_depth_ << "; " << result.total_intersecting_volume_ << std::endl;

	return false;
}


std::map<std::string, AdditionalHypotheses> SequentialSceneHypothesis::generateObjectHypothesesWithPreviousKnowledge( 
	std::vector<map_string_transform> &object_pose_by_dist,
	std::map<std::string, map_string_transform> &object_childs_map,
	const std::map<std::string, ObjectHypothesesData > &object_hypotheses_map)
{
	std::map<std::string, AdditionalHypotheses> result;
	SceneChanges change_in_scene = analyzeChanges();
	std::cerr << "Added objects: " << printVecString(change_in_scene.added_objects_);
	std::cerr << "Removed objects: " << printSetString(change_in_scene.removed_objects_);
	std::cerr << "Perturbed objects: " << printVecString(change_in_scene.perturbed_objects_);
	std::cerr << "Static objects: " << printVecString(change_in_scene.static_objects_);
	std::cerr << "Support retained objects: " << printVecString(change_in_scene.support_retained_object_);

	std::map<std::string, AdditionalHypotheses>  additional_perturbed = generateAdditionalHypothesesForObjectList(
		change_in_scene.perturbed_objects_,PERTURB_OBJECT,
		num_of_hypotheses_to_add_each_action_[PERTURB_OBJECT]);

	std::map<std::string, AdditionalHypotheses>  additional_static = generateAdditionalHypothesesForObjectList(
		change_in_scene.static_objects_,STATIC_OBJECT,
		num_of_hypotheses_to_add_each_action_[STATIC_OBJECT]);

	std::map<std::string, AdditionalHypotheses> additional_support_retained = generateAdditionalHypothesesForObjectList(
		change_in_scene.support_retained_object_,SUPPORT_RETAINED_OBJECT,
		num_of_hypotheses_to_add_each_action_[SUPPORT_RETAINED_OBJECT]);

	SceneHypothesis &previous_best_scene_hypothesis = this->previous_scene_observation_.best_scene_hypothesis_;
	std::map<std::string, vertex_t> prev_vertex_map = previous_best_scene_hypothesis.vertex_map_;
	// last element of object_pose_by_dist
	std::size_t disconnected_object_dist = object_pose_by_dist.size() - 1;
	std::map<std::string, vertex_t> cur_vertex_map = this->current_best_data_scene_structure_.vertex_map_;
	
	std::map<std::string, std::string> &object_label_class_map = this->previous_scene_observation_.object_label_class_map_;

	for (std::vector<std::string>::const_iterator it = change_in_scene.perturbed_objects_.begin();
		it != change_in_scene.perturbed_objects_.end(); ++it)
	{
		const std::string &object_id = *it;
		const vertex_t &prev_obj_vertex = prev_vertex_map[object_id];
		const btTransform &prev_transform = previous_best_scene_hypothesis.scene_support_graph_[prev_obj_vertex].object_pose_;

		const vertex_t &cur_obj_vertex = cur_vertex_map[object_id];
		const std::size_t cur_dist = current_best_data_scene_structure_.scene_support_graph_[cur_obj_vertex].distance_to_ground_;
		const btTransform &cur_transform = current_best_data_scene_structure_.scene_support_graph_[cur_obj_vertex].object_pose_;

		const std::size_t dist = cur_dist < object_pose_by_dist.size() ? cur_dist :  disconnected_object_dist;
		
		const std::string &model_name = object_label_class_map[object_id];

		std::cerr << object_id << ": ";

		double current_data_confidence = this->data_probability_check_->getIcpConfidenceResult(model_name, cur_transform);
		// use previous best pose if the confidence difference between current pose and previous pose is small
		if (current_data_confidence < 0.05 || this->judgeHypothesis(model_name,prev_transform,0.75*current_data_confidence))
		{
			std::cerr << "retained previous pose\n";
			object_pose_by_dist[dist][object_id] = prev_transform;
		}
		else
		{
			std::cerr << "use current pose\n";	
			object_pose_by_dist[dist][object_id] = cur_transform;
		}
	}

	for (std::vector<std::string>::const_iterator it = change_in_scene.support_retained_object_.begin();
		it != change_in_scene.support_retained_object_.end(); ++it)
	{
		// add the object pose by distance information for support retained objects
		const std::string &object_id = *it;
		const vertex_t &obj_vertex = prev_vertex_map[object_id];

		// the object_pose_by_dist starts at 0 (already excludes background)
		const std::size_t prev_dist = previous_best_scene_hypothesis.scene_support_graph_[obj_vertex].distance_to_ground_;
		const std::size_t dist = prev_dist > 1 && prev_dist <= object_pose_by_dist.size() 
			? prev_dist - 1 : disconnected_object_dist;

		const btTransform &prev_transform = previous_best_scene_hypothesis.scene_support_graph_[obj_vertex].object_pose_;

		const std::string &model_name = object_label_class_map[object_id];

		const vertex_t &cur_obj_vertex = cur_vertex_map[object_id];
		const btTransform &cur_transform = current_best_data_scene_structure_.scene_support_graph_[cur_obj_vertex].object_pose_;
		
		std::cerr << object_id << std::endl;

		double current_data_confidence = this->data_probability_check_->getIcpConfidenceResult(model_name, cur_transform);
		// use previous best pose if the confidence difference between current pose and previous pose is small
		if (current_data_confidence < 0.05 || this->judgeHypothesis(model_name,prev_transform,0.7*current_data_confidence))
		{
			std::cerr << "Retained previous pose\n";
			object_pose_by_dist[dist][object_id] = prev_transform;
		}
		else
		{
			std::cerr << "Use current pose\n";	
			object_pose_by_dist[dist][object_id] = cur_transform;
		}

		// add the child information of this object and move the optimization order of the child to its proper distance
		// if properly supported
		const std::vector<std::string> &list_object_supported = flying_object_support_retained_[*it];
		map_string_transform obj_child_tf_supported;
		for (std::vector<std::string>::const_iterator obj_it = list_object_supported.begin();
			obj_it != list_object_supported.end(); ++obj_it) 
		{
			std::size_t distance_to_ground = cur_vertex_map[*obj_it];
			if (distance_to_ground == 0) distance_to_ground = disconnected_object_dist;
			if (keyExistInConstantMap(*obj_it, object_pose_by_dist[disconnected_object_dist]))
			{
				// move the pose information to its proper distance
				const btTransform supp_obj_pose = object_pose_by_dist[disconnected_object_dist][*obj_it];
				obj_child_tf_supported[*obj_it] = supp_obj_pose;
				if (disconnected_object_dist == dist + 1)
				{
					std::cerr << *obj_it << " is already in proper distance map\n";
					continue;
				}
				else if (object_pose_by_dist.size() - 1 < dist + 1)
				{
					std::cerr << *obj_it << " needs to add new distance map vector\n";
					map_string_transform new_map_tf;
					new_map_tf[*obj_it] = supp_obj_pose;
					object_pose_by_dist.push_back(new_map_tf);
				}
				else
				{
					std::cerr << "Adding " << *obj_it << " in proper distance map\n";
					object_pose_by_dist[dist + 1][*obj_it] = supp_obj_pose;
				}
				object_pose_by_dist[disconnected_object_dist].erase(*obj_it);
			}
			else
			{
				std::cerr << "Warning, found inconsistency of object " << *obj_it << " in current scene graph\n";
				continue;
			}
		}
		object_childs_map[object_id] = obj_child_tf_supported;
	}

	if (additional_perturbed.size() > 0) result.insert(additional_perturbed.begin(),additional_perturbed.end());
	if (additional_static.size() > 0) result.insert(additional_static.begin(),additional_static.end());
	if (additional_support_retained.size() > 0) result.insert(additional_support_retained.begin(),additional_support_retained.end());

	for (std::map<std::string, ObjectHypothesesData >::const_iterator it = object_hypotheses_map.begin();
		it != object_hypotheses_map.end(); ++it)
	{
		const std::vector<btTransform> &object_pose_hypotheses = it->second.second;
		if (keyExistInConstantMap(it->first, result))
		{
			result[it->first].poses_.insert(result[it->first].poses_.end(),
				object_pose_hypotheses.begin(),object_pose_hypotheses.end());
		}
		else
		{
			result[it->first] = AdditionalHypotheses(it->second.first, object_pose_hypotheses, ADD_OBJECT);
		}
	}

	return result;
}

void SequentialSceneHypothesis::setMinimalDataConfidence(const double &min_confidence)
{
	this->minimum_data_probability_threshold_ = min_confidence;
}

bool SequentialSceneHypothesis::judgeHypothesis(const std::string &model_name, const btTransform &transform)
{
	return this->judgeHypothesis(model_name, transform, this->minimum_data_probability_threshold_);
}


bool SequentialSceneHypothesis::judgeHypothesis(
	const std::string &model_name, const btTransform &transform, const double &min_confidence)
{
	if (data_probability_check_ == NULL) return false;

	// double data_confidence = data_probability_check_->getConfidence(model_name, transform);
	double data_confidence = this->data_probability_check_->getIcpConfidenceResult(model_name, transform);
	return (data_confidence > min_confidence);
}

SceneChanges SequentialSceneHypothesis::findChanges()
{
	// Try to add last frame's scene points to the current frame
	// Assume ID stays the same
	// If not, find mapping
	SceneChanges result;
	flying_object_support_retained_.clear();
	std::map<std::string, vertex_t> cur_vertex_map = this->current_best_data_scene_structure_.vertex_map_;
	
	// ignore background in findChanges
	cur_vertex_map.erase("background");
	
	if (this->previous_scene_observation_.is_empty)
	{
		std::cerr << "Previous scene is empty.\n";
		// all objects are added object
		result.added_objects_.reserve(cur_vertex_map.size());
		for (std::map<std::string, vertex_t>::const_iterator it = cur_vertex_map.begin(); 
			it != cur_vertex_map.end(); ++it)
		{
			result.added_objects_.push_back(it->first);
		}
		return result;
	}

	SceneHypothesis &previous_best_scene_hypothesis = this->previous_scene_observation_.best_scene_hypothesis_;
	std::map<std::string, vertex_t> prev_vertex_map = previous_best_scene_hypothesis.vertex_map_;
	// ignore background in findChanges
	prev_vertex_map.erase("background");

	for (std::map<std::string, vertex_t>::iterator it = cur_vertex_map.begin(); it != cur_vertex_map.end();)
	{
		if (keyExistInConstantMap(it->first, prev_vertex_map))
		{
			// object is retained, check if it is stable/perturbed
			const btTransform &cur_pose = current_best_data_scene_structure_.scene_support_graph_[it->second].object_pose_;
			vertex_t &prev_vertex = prev_vertex_map[it->first];
			const btTransform &previous_pose = previous_best_scene_hypothesis.scene_support_graph_[prev_vertex].object_pose_;
			btScalar orientation_change = cur_pose.getRotation().angleShortestPath(previous_pose.getRotation());
			btScalar translation_change = cur_pose.getOrigin().distance(previous_pose.getOrigin())/SCALING;
			if (orientation_change > max_static_object_rotation_ || translation_change > max_static_object_translation_)
			{
				std::cerr << it->first << " is perturbed with " << translation_change*100 << "cm and " 
						  << orientation_change * 180. / boost::math::constants::pi<double>() << " degrees.\n";
				result.perturbed_objects_.push_back(it->first);
			}
			else
			{
				result.static_objects_.push_back(it->first);
			}
			prev_vertex_map.erase(it->first);
		}
		else
		{
			result.added_objects_.push_back(it->first);
		}
		cur_vertex_map.erase(it++);
	}

	if (prev_vertex_map.size() > 0)
	{
		// result.removed_objects_.reserve(prev_vertex_map.size());

		// some objects are removed from the scene
		for (std::map<std::string, vertex_t>::iterator it = prev_vertex_map.begin(); 
			it != prev_vertex_map.end(); ++it)
		{
			result.removed_objects_.insert(it->first);
		}
	}

	return result;
}

SceneChanges SequentialSceneHypothesis::analyzeChanges()
{
	// For all objects that are removed, make sure that these 2 conditions is true:
	// 1. There are no point cloud that support this object's existance.
	// 2. There are no object that are supported by this object in the previous scene

	// If there are some objects that are supported by removed object, however there are no point cloud
	// that support this object's existance, the status of this object (stable/perturbed) depends on the directly
	// support object

	SceneChanges compare_scene_result = this->findChanges();
	std::set<std::string> &removed_objects = compare_scene_result.removed_objects_;
	if (removed_objects.size() > 0)
	{
		SceneHypothesis &previous_best_scene_hypothesis = this->previous_scene_observation_.best_scene_hypothesis_;
		SceneSupportGraph &previous_best_scene_graph = previous_best_scene_hypothesis.scene_support_graph_;
		std::map<std::string, vertex_t> &prev_vertex_map = previous_best_scene_hypothesis.vertex_map_;
		std::map<std::string, vertex_t> &cur_vertex_map = this->current_best_data_scene_structure_.vertex_map_;
		SceneSupportGraph &cur_best_graph = this->current_best_data_scene_structure_.scene_support_graph_;

		std::map<std::string, std::string> &object_label_class_map = this->previous_scene_observation_.object_label_class_map_;

		// check if the object need to be retained because of its supported object
		for (std::set<std::string>::iterator it = removed_objects.begin(); it != removed_objects.end();)
		{
			// check if there is any object supported by this object in previous scene
			// that still exists in current scene
			vertex_t &observed_removed_vertex = prev_vertex_map[*it];
			const btTransform &transform = previous_best_scene_graph[observed_removed_vertex].object_pose_;

			int removed_status = updateRemovedObjectStatus(*it,object_label_class_map[*it],transform);
			switch(removed_status)
			{
				case 0: // object is removed
					++it;
					continue;
					break;
				case 1: // object is not removed
					compare_scene_result.support_retained_object_.push_back(*it);
					// removed_objects.erase(it++);
					// continue;
					break;
				default: // object is ephemeral
					// just assume that object should be added
					compare_scene_result.support_retained_object_.push_back(*it);
					// removed_objects.erase(it++);
					// continue;
					break;
			}

// TODO: Fix implementation for ephemeral object...

			// Fix distance map for previously supported object
			std::vector<vertex_t> supported_objects = getAllChildVertices(previous_best_scene_graph, observed_removed_vertex);
			std::cerr << *it << " previously support: " << supported_objects.size() << std::endl;
			if (supported_objects.size() > 0)
			{
				bool exclude_removed_object = false;
				// if any directly supported object is not in the list of removed object, do not remove this object
				std::vector<std::string> flying_object_had_support;
				for (std::vector<vertex_t>::iterator sup_it = supported_objects.begin(); 
					sup_it != supported_objects.end(); ++sup_it)
				{
					std::string &supported_object_id = previous_best_scene_graph[*sup_it].object_id_;

					// if the object became not ground supported without the removed object
					if ((removed_objects.find(supported_object_id) == removed_objects.end()) && 
						(!cur_best_graph[cur_vertex_map[supported_object_id]].ground_supported_))
					{
						exclude_removed_object = true;
						flying_object_had_support.push_back(supported_object_id);
						std::cerr << supported_object_id << " used to be supported" << std::endl;
						// break;
					}
				}

				if (exclude_removed_object)
				{
					// compare_scene_result.support_retained_object_.push_back(*it);
					flying_object_support_retained_[*it] = flying_object_had_support;
					// removed_objects.erase(it++);
				}
// TODO: Fix this later
#if 0
				else
				{
					++it;
				}
			}
			else
			{
				++it;
			}
#else
			}
			removed_objects.erase(it++);
#endif

		}
	}
	return compare_scene_result;
}

AdditionalHypotheses SequentialSceneHypothesis::generateAdditionalObjectHypothesesFromPreviousKnowledge(
	const std::string &object_id, const int &scene_change_mode,
	const int &max_good_hypotheses_to_add)
{
	int added_unique_hypotheses = 0;
	std::set<int> added_hypotheses_id;
	std::vector<btTransform> hypotheses_to_be_inserted_;

	if (this->previous_scene_observation_.is_empty)
	{
		// No additional hypothesis can be added, since there is no previous scene
		return AdditionalHypotheses();
	}
	SceneHypothesis &previous_best_scene_hypothesis = this->previous_scene_observation_.best_scene_hypothesis_;
	std::map<std::string, vertex_t> &previous_vertex_map = previous_best_scene_hypothesis.vertex_map_;
	if (!keyExistInConstantMap(object_id,previous_vertex_map))
	{
		// No additional hypothesis can be added, since this object does not exist in previous scene
		return AdditionalHypotheses();
	}

	std::map<std::string, std::string> &object_label_class_map = this->previous_scene_observation_.object_label_class_map_;
	hypotheses_to_be_inserted_.reserve(max_good_hypotheses_to_add);

	for (OneFrameSceneHypotheses::iterator it = this->previous_scene_observation_.scene_hypotheses_list_.begin();
		it != this->previous_scene_observation_.scene_hypotheses_list_.end(); ++it)
	{
		std::map<std::string, vertex_t> &vertex_map = it->vertex_map_;
		std::map<std::string, int> &scene_object_hypothesis_id = it->scene_object_hypothesis_id_;


		// object has not been added yet in this scene or not in the scene graph.
		if (!(keyExistInConstantMap(object_id,scene_object_hypothesis_id) && 
			keyExistInConstantMap(object_id,vertex_map) )) continue;

		SceneSupportGraph &scene_support_graph = it->scene_support_graph_;
		double &scene_probability = it->scene_probability_;
		if (added_unique_hypotheses < max_good_hypotheses_to_add)
		{
			int &hypothesis_id = scene_object_hypothesis_id[object_id];
			if (added_hypotheses_id.find(hypothesis_id) == added_hypotheses_id.end())
			{
				// This object hypothesis has not been added before
				vertex_t &vertex_id = vertex_map[object_id];
				hypotheses_to_be_inserted_.push_back(scene_support_graph[vertex_id].object_pose_);
				added_hypotheses_id.insert(scene_object_hypothesis_id[object_id]);
				++added_unique_hypotheses;
			}
		}
		else
		{
			// stop the loop if all needed amount of hypothesis had been added
			break;
		}
	}

	if (hypotheses_to_be_inserted_.size() == 0)
	{
		// return invalid additional hypotheses if it contains no additional hypotheses
		return AdditionalHypotheses();
	}

	std::vector<btTransform> valid_hypotheses_to_add;
	valid_hypotheses_to_add.reserve(hypotheses_to_be_inserted_.size());
	const std::string &model_name = object_label_class_map[object_id];

	for (std::vector<btTransform>::iterator it = hypotheses_to_be_inserted_.begin();
		 it != hypotheses_to_be_inserted_.end(); ++it)
	{
		// do not judge hypothesis for support retained object, as there is not enough data to support it
		if (scene_change_mode == SUPPORT_RETAINED_OBJECT || this->judgeHypothesis(model_name, *it))
		{
			valid_hypotheses_to_add.push_back(*it);
		}
	}

	return AdditionalHypotheses(model_name, valid_hypotheses_to_add, scene_change_mode);
}


std::map<std::string, AdditionalHypotheses> SequentialSceneHypothesis::generateAdditionalHypothesesForObjectList(
	const std::vector<std::string> &object_id_list,const int &scene_change_mode,
	const int &max_good_hypotheses_to_add)
{
	std::map<std::string, AdditionalHypotheses> result;
	for (std::vector<std::string>::const_iterator it = object_id_list.begin();
		 it != object_id_list.end(); ++it)
	{
		AdditionalHypotheses additional_obj_hypotheses = generateAdditionalObjectHypothesesFromPreviousKnowledge(
			*it, scene_change_mode, max_good_hypotheses_to_add);
		if (additional_obj_hypotheses.object_action_ != INVALID_ACTION)
		{
			result[*it]=additional_obj_hypotheses;
		}
	}
	return result;
}

