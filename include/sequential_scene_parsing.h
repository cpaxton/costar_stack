#ifndef SEQUENTIAL_SCENE_PARSING_H
#define SEQUENTIAL_SCENE_PARSING_H

#include <vector>
#include <map>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Geometry>

#include "scene_physics_engine.h"
#include "ObjRecRANSACTool/ObjRecRANSACTool.h"

 // ObjectParameter == Object Pose
typedef btTransform ObjectParameter;
// typedef Eigen::Transform< double,3,Eigen::Affine > ObjectParameter;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ImagePtr;
typedef pcl::PointXYZRGBA ImagePoint;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Image;
typedef std::vector<bool> DecisionVector;

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
	
	void setObjectHypothesesMap(std::map<std::string, std::vector<ObjectParameter> > &object_hypotheses_map);
	void evaluateAllObjectHypothesisProbability();

private:
	void getUpdatedSceneSupportGraph();
	double evaluateObjectProbability(const std::string &object_label);
	bool evaluateObjectHypothesis(const std::string &object_label, const btTransform &object_pose_hypothesis);


	bool debug_messages_;
	bool physics_engine_ready_;

	PhysicsEngine * physics_engine_;
	SceneSupportGraph scene_support_graph_;
	// std::vector<SceneSupportGraph> scene_support_graph_;
	std::map<std::string, vertex_t> vertex_map_;

	ObjRecRANSACTool data_probability_check_;

	std::vector<std::string> object_label_;
	std::string background_label_;
	std::map<std::string, ImagePtr> object_point_cloud_;
	std::map<std::string, ObjectParameter> object_instance_parameter_;
	std::map<std::string, std::vector<ObjectParameter> > object_hypotheses_map_;

	// std::map<std::string, std::vector<std::string> > support_pairs_; 
	// one label may support multiple objects. Maybe make this into graph instead of list for easier support check
	;
};


#endif
