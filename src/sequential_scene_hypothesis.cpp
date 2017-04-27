#include "sequential_scene_hypothesis.h"

SceneObservation::SceneObservation(const OneFrameSceneHypotheses &input, 
	const std::map<std::string, std::string> &object_label_class_map) : 
		scene_hypotheses_list_(input),
		object_label_class_map_(object_label_class_map), is_empty(false)
{
	std::sort(this->scene_hypotheses_list_.begin(),this->scene_hypotheses_list_.end(),compare_greater);
	this->best_scene_hypothesis_ = this->scene_hypotheses_list_[0];
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

void SequentialSceneHypothesis::generateAdditionalObjectHypothesesFromPreviousKnowledge(
		std::map<std::string, ObjectHypothesesData > &object_hypotheses_map, 
		const int &max_good_hypotheses_to_add)
{
	// For adding hypothesis to perturbed objects

	std::map<std::string, int> added_unique_hypotheses;
	std::map<std::string, std::set<int> > added_hypotheses_id;
	std::map<std::string, std::vector<btTransform> > hypotheses_to_be_inserted_;

	if (this->previous_scene_observation_.is_empty)
	{
		// No additional hypothesis can be added, since there is no previous scene
		return;
	}
	
	std::map<std::string, std::string> &object_label_class_map = this->previous_scene_observation_.object_label_class_map_;
	SceneHypothesis &previous_best_scene_hypothesis = this->previous_scene_observation_.best_scene_hypothesis_;

	std::map<std::string, vertex_t> &previous_vertex_map = previous_best_scene_hypothesis.vertex_map_;
	// Initialize added hypotheses to contain all objects that are present in the previous scene hypothesis
	for (std::map<std::string, ObjectHypothesesData >::const_iterator it = object_hypotheses_map.begin();
		it != object_hypotheses_map.end(); ++it)
	{
		if (keyExistInConstantMap(it->first,previous_vertex_map))
		{
			added_unique_hypotheses[it->first] = 0;
			hypotheses_to_be_inserted_[it->first].reserve(max_good_hypotheses_to_add);
		}
	}

	bool done = false;
	for (OneFrameSceneHypotheses::iterator it = this->previous_scene_observation_.scene_hypotheses_list_.begin();
		it != this->previous_scene_observation_.scene_hypotheses_list_.end(); ++it)
	{
		std::map<std::string, vertex_t> &vertex_map = it->vertex_map_;
		SceneSupportGraph &scene_support_graph = it->scene_support_graph_;
		double &scene_probability = it->scene_probability_;
		std::map<std::string, int> &scene_object_hypothesis_id = it->scene_object_hypothesis_id_;

		for (std::map<std::string, int >::iterator it2 = added_unique_hypotheses.begin();
			it2 != added_unique_hypotheses.end();)
		{
			const std::string &object_tf_id = it2->first;
			int &add_hypothesis_count = it2->second;
			if (add_hypothesis_count < max_good_hypotheses_to_add)
			{
				int &hypothesis_id = scene_object_hypothesis_id[object_tf_id];
				if (added_hypotheses_id[object_tf_id].find(hypothesis_id) != added_hypotheses_id[object_tf_id].end())
				{
					// This stable object hypothesis is already added.
					continue;
				}
				else
				{
					// Add this object hypothesis.
					vertex_t &vertex_id = vertex_map[object_tf_id];
					hypotheses_to_be_inserted_[object_tf_id].push_back(scene_support_graph[vertex_id].object_pose_);
					added_hypotheses_id[object_tf_id].insert(scene_object_hypothesis_id[object_tf_id]);
					++add_hypothesis_count;
				}
				++it2;
			}
			else
			{
				// the amount needed for this object_id has reached, we can savely remove this hypothesis.
				added_unique_hypotheses.erase(it2++);
			}
		}

		// stop the loop if all needed amount of hypothesis had been added
		if (added_unique_hypotheses.size() == 0) break;
	}

	for (std::map<std::string, std::vector<btTransform> >::iterator it = hypotheses_to_be_inserted_.begin();
		 it != hypotheses_to_be_inserted_.end(); ++it)
	{
		const std::string &model_name = object_label_class_map[it->first];
		
		std::vector<btTransform> valid_hypotheses_to_add;
		valid_hypotheses_to_add.reserve(it->second.size());
		for (std::vector<btTransform>::iterator it2 = it->second.begin(); it2 != it->second.end(); ++it2)
		{
			if (this->judgeHypothesis(model_name, *it2))
			{
				valid_hypotheses_to_add.push_back(*it2);
			}
		}
		if (keyExistInConstantMap(it->first, object_hypotheses_map))
		{
			std::vector<btTransform> &hypothesis_list_to_be_modified = object_hypotheses_map[it->first].second;
			// add all valid hypotheses to the original hypothesis
			hypothesis_list_to_be_modified.insert(hypothesis_list_to_be_modified.end(), 
				valid_hypotheses_to_add.begin(), valid_hypotheses_to_add.end());
		}
		else
		{
			object_hypotheses_map[it->first] = std::make_pair(model_name, valid_hypotheses_to_add);
		}
	}
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


