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
#include <Eigen/Dense>

#include "scene_physics_support.h"
#include "scene_data_forces.h"
// #include "ObjRecRANSACTool/ObjRecRANSACTool.h"
#include "utility.h"
#include "typedef.h"
#include <boost/math/constants/constants.hpp>

struct SceneHypothesis
{
	std::map<std::string, vertex_t> vertex_map_;
	SceneSupportGraph scene_support_graph_;
	double scene_probability_;
	std::map<std::string, int> scene_object_hypothesis_id_;

	SceneHypothesis(const std::map<std::string, vertex_t> &vertex_map, 
		const SceneSupportGraph &scene_support_graph) :
			vertex_map_(vertex_map),
			scene_support_graph_(scene_support_graph),
			scene_probability_(-0.5)
	{}

	SceneHypothesis(const std::map<std::string, vertex_t> &vertex_map, 
		const SceneSupportGraph &scene_support_graph,
		const double &scene_probability,
		const std::map<std::string, int> &obj_hypothesis_id) : 
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

	SceneObservation(const SceneHypothesis &final_scene_hypothesis,
		const OneFrameSceneHypotheses &input, 
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
	std::vector<std::string> static_objects_;

	// objects that are retained (excluded from removal) because there is support/data as
	// the evidence of its existance
	std::vector<std::string> support_retained_object_;
	// std::vector<std::string> data_retained_object;
};

enum SceneChangeMode
{
	INVALID_ACTION,
	ADD_OBJECT,
	REMOVE_OBJECT,
	PERTURB_OBJECT,
	STATIC_OBJECT,
	SUPPORT_RETAINED_OBJECT,
	EPHEMERAL_OBJECT
};

struct AdditionalHypotheses
{
	std::string model_name_;
	std::vector<btTransform> poses_;
	int object_action_;
	AdditionalHypotheses(const std::string &model_name,
		const std::vector<btTransform> &poses, const int &object_action) : 
			model_name_(model_name),
			poses_(poses),
			object_action_(object_action)
	{}

	AdditionalHypotheses(): object_action_(INVALID_ACTION) {}
};

class SequentialSceneHypothesis
{
public:
	SequentialSceneHypothesis();
	
	// set the best data scene structure to guess the resulting structure of the hypothesis
	void setCurrentDataSceneStructure(const SceneHypothesis &current_best_data_scene_structure);

	// accumulate the scene observation from the previous scene
	void setPreviousSceneObservation(const SceneObservation &previous_scene);
	
	// add good previous stable object hypotheses to the current hypothesis
	std::map<std::string, AdditionalHypotheses> generateObjectHypothesesWithPreviousKnowledge(
		std::vector<map_string_transform> &object_pose_by_dist,
		std::map<std::string, map_string_transform> &object_childs_map,
		const std::map<std::string, ObjectHypothesesData > &object_hypotheses_map);

	// calculate the effect of scene change on the scene confidence
	void estimateSequentialSceneConfidence();

	void setMinimalDataConfidence(const double &min_confidence);

	// void setObjRansacTool(ObjRecRANSACTool &data_probability_check);
	void setConfidenceCheckTool(FeedbackDataForcesGenerator &data_probability_check);

	void setStaticObjectThreshold(const double &translation_meter, const double &rotation_degree);

	void setCameraMatrix(const double &fx, const double &fy, const double &cx, const double &cy);

	void setSceneData(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud);

private:
	SceneChanges findChanges();
	SceneChanges analyzeChanges();
	AdditionalHypotheses generateAdditionalObjectHypothesesFromPreviousKnowledge(const std::string &object_id,
		const int &scene_change_mode,
		const int &max_good_hypotheses_to_add);

	std::map<std::string, AdditionalHypotheses>  generateAdditionalHypothesesForObjectList(
		const std::vector<std::string> &object_id_list,const int &scene_change_mode,
		const int &max_good_hypotheses_to_add);

	bool judgeHypothesis(const std::string &model_name, const btTransform &transform);

	Eigen::MatrixXd backprojectCloudtoPixelCoordinate(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	Eigen::MatrixXd generate2dMatrixFromPixelCoordinate(const Eigen::MatrixXd &pixel_matrix);

	// deciding object removal status as Support Retained/Ephemeral/Removed Object
	int updateRemovedObjectStatus(const std::string &model_name, const btTransform &object_pose);
	bool checkObjectVisible(const std::string &model_name, const btTransform &object_pose);
	bool checkObjectObstruction(const std::string &model_name, const btTransform &object_pose);
	bool checkObjectReplaced(const std::string &model_name, const btTransform &object_pose);

	// minimum objRecRANSAC confidence for the old hypothesis
	double minimum_data_probability_threshold_;

	// threshold for retained object movement
	double max_static_object_translation_;
	double max_static_object_rotation_;

	// ObjRecRANSACTool* data_probability_check_;
	FeedbackDataForcesGenerator* data_probability_check_;

	SceneHypothesis current_best_data_scene_structure_;
	SceneObservation previous_scene_observation_;
	std::map<int, int> num_of_hypotheses_to_add_each_action_;
	std::map<std::string, std::vector<std::string> > flying_object_support_retained_;

	// for backprojecting point cloud to image
	Eigen::Matrix<double,3,4> backprojection_matrix_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_point_cloud_;
	Eigen::MatrixXd scene_image_pixel_;
};

#endif