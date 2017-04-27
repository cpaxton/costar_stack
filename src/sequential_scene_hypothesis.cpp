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
	const SceneHypothesis &current_best_data_scene_structure_)
{
	this->current_best_data_scene_structure_ = current_best_data_scene_structure_;
}

void SequentialSceneHypothesis::setPreviousSceneObservation(const SceneObservation &previous_scene)
{
	this->previous_scene_observation_ = previous_scene;	
}

void SequentialSceneHypothesis::generateAdditionalObjectHypothesesFromPreviousKnowledge(
		std::map<std::string, ObjectHypothesesData > &object_hypotheses_map, 
		const int &max_good_hypotheses_to_add)
{
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
