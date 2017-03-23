#ifndef ROS_SEQUENTIAL_SCENE_PARSING_H
#define ROS_SEQUENTIAL_SCENE_PARSING_H

// ROS stuffs
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>


// use detected object list instead of tf name convention
#include <costar_objrec_msgs/DetectedObject.h>
#include <costar_objrec_msgs/DetectedObjectList.h>

// object hypothesis associated with the detectedObjectList
#include <objrec_hypothesis_msgs/AllModelHypothesis.h>
#include <objrec_hypothesis_msgs/ModelHypothesis.h>
#include <objrec_hypothesis_msgs/Hypothesis.h>

#include "sequential_scene_parsing.h"

// Ros bundling for scene parsing
class RosSceneGraph : public SceneGraph
{
public:
	RosSceneGraph();
	RosSceneGraph(const ros::NodeHandle &nh);
	void setNodeHandle(const ros::NodeHandle &nh);
	void addBackground(const sensor_msgs::PointCloud2 &pc);
	
	void publishTf();

	void setDebugMode(bool debug);
	void callGlutMain(int argc, char* argv[]);

	PhysicsEngine physics_engine_;
	
private:
	bool fillObjectPropertyDatabase();
	void updateSceneFromDetectedObjectMsgs(const costar_objrec_msgs::DetectedObjectList &detected_objects);
	void fillObjectHypotheses(const objrec_hypothesis_msgs::AllModelHypothesis &detected_object_hypotheses);
	void processHypothesis();

	// void initialize();
	bool debug_messages_;
	bool class_ready_;
	bool physics_gravity_direction_set_;
	bool background_normal_as_gravity_;
	bool has_tf_;
	bool has_background_;

	// Background mode: 0 = BACKGROUND_PLANE, 1 = BACKGROUND_HULL, 2 = BACKGROUND_MESH
	int background_mode_;
	ros::Subscriber detected_object_sub;
	ros::Subscriber object_hypothesis_sub;
	ros::Subscriber background_pcl_sub;
	ros::Subscriber scene_pcl_sub;
	
	ros::NodeHandle nh_;
	tf::TransformListener listener_;
	tf::TransformBroadcaster tf_broadcaster_;
	SceneGraph ros_scene_;
	// PhysicsEngine physics_engine_;
	std::map<std::string, ObjectParameter> object_transforms_;
	std::map<std::string, tf::Transform> object_transforms_tf_;
	std::string parent_frame_;
	std::string tf_z_is_inverse_gravity_direction_;
	std::string tf_publisher_initial;
	ObjectDatabase obj_database_;
	std::map<std::string, PhysicalProperties> physical_properties_database_;

	boost::mutex mtx_;
};


#endif
