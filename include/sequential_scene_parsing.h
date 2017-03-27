#ifndef SEQUENTIAL_SCENE_PARSING_H
#define SEQUENTIAL_SCENE_PARSING_H

#include <iostream>
#include <utility>
#include <vector>
#include <map>

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

#include <boost/filesystem.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>

#include "scene_physics_engine.h"
#include "ObjRecRANSACTool/ObjRecRANSACTool.h"
#include "utility.h"

 // ObjectParameter == Object Pose
typedef btTransform ObjectParameter;
// typedef Eigen::Transform< double,3,Eigen::Affine > ObjectParameter;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ImagePtr;
typedef pcl::PointXYZRGBA ImagePoint;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Image;
typedef std::vector<bool> DecisionVector;

// this contains model name and pose hypothesis of a single object
typedef std::pair<std::string, std::vector<ObjectParameter> > ObjectHypothesesData;

class SceneGraph
{
public:
	SceneGraph() : physics_engine_ready_(false) {};
	SceneGraph(ImagePtr input, ImagePtr background_image);
	
	// set physics engine environment to be used.
	void setPhysicsEngine(PhysicsEngine* physics_engine);
	// void setPhysicsEngine(PhysicsEngine* physics_engine);

	void addBackground(ImagePtr background_image, int mode = 0);
	void addScenePointCloud(ImagePtr scene_image);

	void addNewObjectTransforms(const std::vector<ObjectWithID> &objects);
	std::map<std::string, ObjectParameter> getCorrectedObjectTransform();
	void setDebugMode(bool debug);
	
	void setObjectHypothesesMap(std::map<std::string, ObjectHypothesesData > &object_hypotheses_map);
	void evaluateAllObjectHypothesisProbability();

	bool loadObjectModels(const std::string &input_model_directory_path, const std::vector<std::string> &object_names);

private:
	void getUpdatedSceneSupportGraph();
	double evaluateObjectProbability(const std::string &object_label, const std::string &object_model_name, const bool &verbose = false);
	double evaluateSceneOnObjectHypothesis(std::map<std::string, btTransform> &object_pose_from_graph, const std::string &object_label, const std::string &object_model_name,
		const btTransform &object_pose_hypothesis, const bool &reset_position);


	bool debug_messages_;
	bool physics_engine_ready_;

	PhysicsEngine * physics_engine_;
	SceneSupportGraph scene_support_graph_;
	// std::vector<SceneSupportGraph> scene_support_graph_;
	std::map<std::string, vertex_t> vertex_map_;

	ObjRecRANSACTool data_probability_check_;

	std::map<std::string, std::string> object_label_class_map;
	std::string background_label_;
	std::map<std::string, ImagePtr> object_point_cloud_;
	std::map<std::string, ObjectParameter> object_instance_parameter_;
	std::map<std::string, ObjectHypothesesData > object_hypotheses_map_;

	boost::mutex seq_mtx_;
	// std::map<std::string, std::vector<std::string> > support_pairs_; 
	// one label may support multiple objects. Maybe make this into graph instead of list for easier support check
	;
};


#endif
