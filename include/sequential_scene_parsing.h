#ifndef SEQUENTIAL_SCENE_PARSING_H
#define SEQUENTIAL_SCENE_PARSING_H

#include <iostream>
#include <utility>

// For plane segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>

// For creating mesh from point cloud input
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/conversions.h>

#include <pcl/io/pcd_io.h>

#include <boost/filesystem.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <Eigen/Geometry>

#include "ObjRecRANSACTool/ObjRecRANSACTool.h"
#include "symmetric_orientation_realignment.h"
#include "utility.h"
#include "typedef.h"

#include "scene_physics_engine.h"
#include "scene_data_forces.h"
#include "sequential_scene_hypothesis.h"

class SceneHypothesisAssessor
{
public:
	SceneHypothesisAssessor() : physics_engine_ready_(false) {};
	SceneHypothesisAssessor(ImagePtr input, ImagePtr background_image);
	
	// set physics engine environment to be used.
	void setPhysicsEngine(PhysicsEngine* physics_engine);
	// void setPhysicsEngine(PhysicsEngine* physics_engine);

	void addBackground(ImagePtr background_image, int mode = 0);
	void addScenePointCloud(ImagePtr scene_image);

	void addNewObjectTransforms(const std::vector<ObjectWithID> &objects);
	std::map<std::string, ObjectParameter> getCorrectedObjectTransform();
	std::map<std::string, ObjectParameter> getCorrectedObjectTransformFromSceneGraph();
	void setDebugMode(bool debug);
	
	void setObjectHypothesesMap(std::map<std::string, ObjectHypothesesData > &object_hypotheses_map);
	void evaluateAllObjectHypothesisProbability();

	bool loadObjectModels(const std::string &input_model_directory_path, const std::vector<std::string> &object_names);
	void setObjectSymmetryMap(const std::map<std::string, ObjectSymmetry> &object_symmetry_map);

	template <typename numericStandard>
	void setDataFeedbackForcesParameters(const numericStandard &forces_magnitude_per_point, 
		const numericStandard &max_point_distance_threshold)
	{
		data_forces_generator_.setForcesParameter(
			btScalar(forces_magnitude_per_point),btScalar(max_point_distance_threshold));
	}

private:
	void getCurrentSceneSupportGraph();
	void getUpdatedSceneSupportGraph();
	double evaluateObjectProbability(const std::string &object_label, 
		const std::string &object_model_name, const int &object_action,
		const bool &verbose = false);
	double evaluateSceneOnObjectHypothesis(std::map<std::string, btTransform> &object_pose_from_graph, 
		const std::string &object_label, const std::string &object_model_name, bool &background_support_status,
		const btTransform &object_pose_hypothesis, const int &object_action,const bool &reset_position);
	double evaluateSceneProbabilityFromGraph(const std::map<std::string, int> &object_action_map);
	void getSceneSupportGraphFromBestData(
		std::map<std::string, bool> &object_background_support_status,
		std::vector< std::map<std::string, btTransform> > &object_test_pose_map_by_dist,
		std::map<std::string, map_string_transform> &object_childs_map);
	std::map<std::string, map_string_transform> getAllChildTransformsOfVertices(
		const std::map<vertex_t, std::size_t> &vertex_distance_map);
	// void processHypothesis();

	bool debug_messages_;
	bool physics_engine_ready_;

	PhysicsEngine * physics_engine_;
	SceneSupportGraph scene_support_graph_;
	FeedbackDataForcesGenerator data_forces_generator_;

	// std::vector<SceneSupportGraph> scene_support_graph_;
	std::map<std::string, vertex_t> vertex_map_;

	ObjRecRANSACTool data_probability_check_;

	std::map<std::string, std::string> object_label_class_map;
	std::string background_label_;
	std::map<std::string, ImagePtr> object_point_cloud_;
	std::map<std::string, ObjectParameter> object_instance_parameter_;
	std::map<std::string, ObjectHypothesesData > object_hypotheses_map_;
	std::map<std::string, ObjectSymmetry> object_symmetry_map_;

	SequentialSceneHypothesis sequential_scene_hypothesis_;

	boost::mutex seq_mtx_;
	// std::map<std::string, std::vector<std::string> > support_pairs_; 
	// one label may support multiple objects. Maybe make this into graph instead of list for easier support check
	;
};


#endif
