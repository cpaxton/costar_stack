#include "sequential_scene_hypothesis.h"

SceneObservation::SceneObservation(const OneFrameSceneHypotheses &input, 
	const std::map<std::string, std::string> &object_label_class_map) : 
		scene_hypotheses_list_(input),
		object_label_class_map_(object_label_class_map), is_empty(false)
{
	std::sort(this->scene_hypotheses_list_.begin(),this->scene_hypotheses_list_.end(),compare_greater);
	this->best_scene_hypothesis_ = this->scene_hypotheses_list_[0];
}

SequentialSceneHypothesis::SequentialSceneHypothesis() : minimum_data_probability_threshold_(0.1) 
{
	// These action implies these objects are not present in previous scene
	num_of_hypotheses_to_add_each_action_[INVALID_ACTION] = 0;
	num_of_hypotheses_to_add_each_action_[ADD_OBJECT] = 0;
	num_of_hypotheses_to_add_each_action_[REMOVE_OBJECT] = 0;

	num_of_hypotheses_to_add_each_action_[PERTURB_OBJECT] = 10;
	num_of_hypotheses_to_add_each_action_[STEADY_OBJECT] = 5;
	num_of_hypotheses_to_add_each_action_[SUPPORT_RETAINED_OBJECT] = 10;
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

std::map<std::string, AdditionalHypotheses> SequentialSceneHypothesis::generateObjectHypothesesWithPreviousKnowledge( 
	const std::map<std::string, ObjectHypothesesData > &object_hypotheses_map)
{
	std::map<std::string, AdditionalHypotheses> result;
	SceneChanges change_in_scene = analyzeChanges();

	std::map<std::string, AdditionalHypotheses>  additional_perturbed = generateAdditionalHypothesesForObjectList(
		change_in_scene.perturbed_objects_,PERTURB_OBJECT,
		num_of_hypotheses_to_add_each_action_[PERTURB_OBJECT]);

	std::map<std::string, AdditionalHypotheses>  additional_steady = generateAdditionalHypothesesForObjectList(
		change_in_scene.steady_objects_,STEADY_OBJECT,
		num_of_hypotheses_to_add_each_action_[STEADY_OBJECT]);

	std::map<std::string, AdditionalHypotheses>  additional_support_retained = generateAdditionalHypothesesForObjectList(
		change_in_scene.support_retained_object_,SUPPORT_RETAINED_OBJECT,
		num_of_hypotheses_to_add_each_action_[SUPPORT_RETAINED_OBJECT]);
	
	result.insert(additional_perturbed.begin(),additional_perturbed.end());
	result.insert(additional_steady.begin(),additional_steady.end());
	result.insert(additional_support_retained.begin(),additional_support_retained.end());

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
	if (data_probability_check_ == NULL) return false;

	double data_confidence = data_probability_check_->getConfidence(model_name, transform);
	return (data_confidence < minimum_data_probability_threshold_);
}

SceneChanges SequentialSceneHypothesis::findChanges()
{
	// Try to add last frame's scene points to the current frame
	// Assume ID stays the same
	// If not, find mapping
	SceneChanges result;
	std::map<std::string, vertex_t> cur_vertex_map = this->current_best_data_scene_structure_.vertex_map_;
	if (this->previous_scene_observation_.is_empty)
	{
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
	for (std::map<std::string, vertex_t>::iterator it = cur_vertex_map.begin(); it != cur_vertex_map.end();)
	{
		if (keyExistInConstantMap(it->first, prev_vertex_map))
		{
			// object is retained, check if it is stable/perturbed
			const btTransform &cur_pose = current_best_data_scene_structure_.scene_support_graph_[it->second].object_pose_;
			vertex_t &prev_vertex = prev_vertex_map[it->first];
			const btTransform &previous_pose = previous_best_scene_hypothesis.scene_support_graph_[prev_vertex].object_pose_;
			btScalar orientation_change = cur_pose.getRotation().angleShortestPath(previous_pose.getRotation());
			btScalar translation_change = cur_pose.getOrigin().distance(previous_pose.getOrigin());
			if (orientation_change > max_steady_object_rotation_ || translation_change > max_steady_object_translation_)
			{
				result.perturbed_objects_.push_back(it->first);
			}
			else
			{
				result.steady_objects_.push_back(it->first);
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

		// check if the object need to be retained because of its supported object
		for (std::set<std::string>::iterator it = removed_objects.begin(); it != removed_objects.end();)
		{
			// check if there is any object supported by this object in previous scene
			// that still exists in current scene
			vertex_t &observed_removed_vertex = prev_vertex_map[*it];
			OrderedVertexVisitor vis  = getOrderedVertexList(previous_best_scene_graph, observed_removed_vertex);
			std::map<std::size_t, std::vector<vertex_t> > vertex_visit_by_dist = vis.getVertexVisitOrderByDistances();
			if (keyExistInConstantMap(1ul,vertex_visit_by_dist))
			{
				std::vector<vertex_t> &supported_objects = vertex_visit_by_dist[1];
				bool exclude_removed_object = false;
				// if any directly supported object is not in the list of removed object, do not remove this object

				for (std::vector<vertex_t>::iterator sup_it = supported_objects.begin(); 
					sup_it != supported_objects.end(); ++sup_it)
				{
					std::string &supported_object_id = previous_best_scene_graph[*sup_it].object_id_;

					// if the object became not ground supported without the removed object
					if ((removed_objects.find(supported_object_id) == removed_objects.end()) && 
						(!cur_best_graph[cur_vertex_map[supported_object_id]].ground_supported_))
					{
						exclude_removed_object = true;
						break;
					}
				}

				if (exclude_removed_object)
				{
					compare_scene_result.support_retained_object_.push_back(*it);
					removed_objects.erase(it++);
				}
			}
			else
			{
				++it;
			}
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

	bool done = false;
	for (OneFrameSceneHypotheses::iterator it = this->previous_scene_observation_.scene_hypotheses_list_.begin();
		it != this->previous_scene_observation_.scene_hypotheses_list_.end(); ++it)
	{
		std::map<std::string, vertex_t> &vertex_map = it->vertex_map_;
		SceneSupportGraph &scene_support_graph = it->scene_support_graph_;
		double &scene_probability = it->scene_probability_;
		std::map<std::string, int> &scene_object_hypothesis_id = it->scene_object_hypothesis_id_;

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
		if (this->judgeHypothesis(model_name, *it))
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


