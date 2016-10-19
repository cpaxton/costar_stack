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
 // ObjectParameter == Object Pose
typedef btTransform ObjectParameter;
// typedef Eigen::Transform< double,3,Eigen::Affine > ObjectParameter;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr Image;
typedef std::vector<bool> DecisionVector;

class SceneGraph
{
public:
	SceneGraph() : physics_engine_ready_(false) {};
	SceneGraph(Image input, Image background_image);
	
	// set physics engine environment to be used.
	void setPhysicsEngine(PhysicsEngine* physics_engine);

	void addBackground(Image background_image);
	void addNewObjectTransforms(const std::vector<ObjectWithID> &objects);
	std::map<std::string, ObjectParameter> getCorrectedObjectTransform();
	void setDebugMode(bool debug);
	
private:
	bool debug_messages_;
	bool physics_engine_ready_;
	PhysicsEngine * physics_engine_;

	// TODO
	// void addObject();
	// void removeObject();
	// void moveObject();
	// void perturbObject();

	// // do everything (detecting objects etc)
	// void processImage(Image input);

	std::vector<std::string> object_label_;
	std::string background_label_;
	std::map<std::string, Image> object_point_cloud_;
	std::map<std::string, ObjectParameter> object_instance_parameter_;

	// std::map<std::string, std::vector<std::string> > support_pairs_; 
	// one label may support multiple objects. Maybe make this into graph instead of list for easier support check
	;
};

class SequentialSceneGraph
{
public:
	SequentialSceneGraph(Image background_image);
	void addScene(Image input_image);
	// generateEstimate();
;
private:
	DecisionVector BDLinear(Image I, SceneGraph G_minus, SceneGraph G_bar, DecisionVector D, std::vector<std::string> O);
	void binaryDecision();
	void graphSweep();
	void phase1();
	void phase2();
	void phase3();
	void phase4();
	void phase5();
	void phase6();
	std::vector <SceneGraph> sequential_scene_graph_;
;
};

#endif
