#ifndef ROS_PLANE_SEGMENTER
#define ROS_PLANE_SEGMENTER

#include <ros/ros.h>
// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// for using tf surface
#include <pcl_ros/transforms.h>

// for creating directory automatically
#include <boost/filesystem.hpp>

#include "object_on_table_segmenter/plane_segmenter.h"

// This class extend PlaneSegmenter to automatically configures and updates the plane segmenter parameters using ros param
class RosPlaneSegmenter : public PlaneSegmenter
{
public:
	RosPlaneSegmenter() : initialized_(false) {};
	~RosPlaneSegmenter();

	// Load all rosparam for the plane segmentation
	void initialize(const ros::NodeHandle &nh);

	// Update the plane if the plane data does not exist yet
	void segmentPlaneIfNotExist(const sensor_msgs::PointCloud2 &input_cloud);
	// Update the plane data no matter what. User of this function must ensure this class was initialized.
	void segmentPlane(const sensor_msgs::PointCloud2 &input_cloud);

	void enableSavingTableFile(){
		update_table_ = true;
	}

	void disableSavingTableFile(){
		update_table_ = false;
	}

private:
	bool initialized_;
	bool update_table_;
	bool use_rosbag_;
	std::string table_tf_name_;
	std::string load_table_path_;
	pcl::PCDWriter writer_;
	pcl::PCDReader reader_;
	tf::TransformListener * listener_;
	bool use_tf_surface_;
	Eigen::Vector3f crop_box_size_;
	double tf_table_hull_size_;
};

#endif