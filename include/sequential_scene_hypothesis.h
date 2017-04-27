#ifndef SEQUENTIAL_SCENE_HYPOTHESIS_H
#define SEQUENTIAL_SCENE_HYPOTHESIS_H

#include <iostream>
#include <utility>
#include <vector>
#include <map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>

#include "scene_physics_support.h"
#include "ObjRecRANSACTool/ObjRecRANSACTool.h"
#include "utility.h"
#include "typedef.h"

struct SceneHypothesis
{
	std::map<std::string, vertex_t> vertex_map_;
	SceneSupportGraph scene_support_graph_;
	double scene_probability_;
	std::map<std::string, int> scene_object_hypothesis_id_;

	SceneHypothesis(const std::map<std::string, vertex_t> &vertex_map, 
		const SceneSupportGraph &scene_support_graph,
		const double &scene_probability,
		const std::map<std::string, int> &obj_hypothesis_id,
		const std::map<std::string, std::string> &object_label_class_map) : 
			vertex_map_(vertex_map),
			scene_support_graph_(scene_support_graph), scene_probability_(scene_probability),
			scene_object_hypothesis_id_(obj_hypothesis_id)
	{}

	SceneHypothesis() : scene_probability_(-1.)
	{}

	bool operator<(const SceneHypothesis & other) //(1)
    {
        return scene_probability_ < other.scene_probability_;
    }
};

static bool compare_greater(const SceneHypothesis &lhs, const SceneHypothesis &rhs)
{
	return lhs.scene_probability_ > rhs.scene_probability_;
}

typedef std::vector<SceneHypothesis> OneFrameSceneHypotheses;

struct SceneObservation
{
	SceneHypothesis best_scene_hypothesis_;
	OneFrameSceneHypotheses scene_hypotheses_list_;
	std::map<std::string, std::string> object_label_class_map_;
	bool is_empty;

	SceneObservation(const OneFrameSceneHypotheses &input, 
		const std::map<std::string, std::string> &object_label_class_map);
	SceneObservation() : is_empty(true) { };
};

struct SceneChanges
{
	// Contains tf ID of the objects that are changed

	// added objects will only contains new object hypotheses
	std::vector<std::string> added_objects_;

	// removed object has no futher processing in scene_assessor
	std::set<std::string> removed_objects_;

	// objects that are retained, but has movement > threshold
	std::vector<std::string> perturbed_objects_;

	// objects that are retained, but has movement < threshold
	std::vector<std::string> steady_objects_;

	// objects that are retained (excluded from removal) because there is support/data as
	// the evidence of its existance
	std::vector<std::string> support_retained_object_;
	// std::vector<std::string> data_retained_object;
};

class SequentialSceneHypothesis
{
public:
	SequentialSceneHypothesis() : minimum_data_probability_threshold_(0.1) {};
	
	// set the best data scene structure to guess the resulting structure of the hypothesis
	void setCurrentDataSceneStructure(const SceneHypothesis &current_best_data_scene_structure);

	// accumulate the scene observation from the previous scene
	void setPreviousSceneObservation(const SceneObservation &previous_scene);
	
	// add good previous stable object hypotheses to the current hypothesis
	void generateAdditionalObjectHypothesesFromPreviousKnowledge(
		std::map<std::string, ObjectHypothesesData > &object_hypotheses_map, 
		const int &max_good_hypotheses_to_add = 10);

	// calculate the effect of scene change on the scene confidence
	void estimateSequentialSceneConfidence();

	void setMinimalDataConfidence(const double &min_confidence);

private:
	SceneChanges findChanges();
	SceneChanges analyzeChanges();
	bool judgeHypothesis(const std::string &model_name, const btTransform &transform);

	// minimum objRecRANSAC confidence for the old hypothesis
	double minimum_data_probability_threshold_;

	// threshold for retained object movement
	double max_steady_object_translation_;
	double max_steady_object_rotation_;

	ObjRecRANSACTool* data_probability_check_;
	SceneHypothesis current_best_data_scene_structure_;
	SceneObservation previous_scene_observation_;
};

#endif