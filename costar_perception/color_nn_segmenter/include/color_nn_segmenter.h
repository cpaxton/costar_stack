#ifndef COLOR_NN_SEGMENTER
#define COLOR_NN_SEGMENTER

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <set>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
// #include <pcl/ml/kmeans.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/algorithm/string.hpp>

#include "utility.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZL> PointCloudXYZL;

class ColorNnSegmenter
{
public:
	ColorNnSegmenter() : ready(false), remap_color_label(false), unlabelled_model_cloud(new PointCloudXYZ()) {};
	
	// train a k means clustering color segmenter
	bool trainModel(const std::string &training_data_directory, const  int kmeans_point_per_model = 5);
	// save the model data to a file
	bool saveModel(const std::string &target_directory, const std::string &model_name);
	// load the model data from a file
	bool loadModel(const std::string &model_dat_file_path);

    // remap color label numbers to a newly defined set of numbers using a comma separated list of labels.
    // ex: setColorLabels("wood_block,white_bin", "red_block, green_block, blue_block, yellow_block")
    // will map to #s:     0,                 0    1,         2,           3,          4     
    // any items not in the foreground labels list will automatically be put in the background labels list
    void setColorLabels(const std::string& foreground_labels, const std::string ignored_labels = std::string());

    // perform segmentation
	PointCloudXYZL::Ptr segment(const PointCloudXYZRGB &input_cloud);

private:
	bool checkDirectoryValid(const boost::filesystem::path dir_path);
	PointCloudXYZ::Ptr stripCloudCoordinate(const PointCloudXYZRGB &input_cloud);
	void rgbToLabCloud(PointCloudXYZ::Ptr input_cloud);
	PointCloudXYZ::Ptr openCvKMeans(const PointCloudXYZ &input_cloud, const int &cluster_size);

	bool ready;
	bool remap_color_label;
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	PointCloudXYZ::Ptr unlabelled_model_cloud;
	PointCloudXYZL::Ptr model_cloud;
	std::map<unsigned int, std::string> label_index_map;
	std::set<unsigned int> background_label_index_map;
	// helps make setColorLabels work
	std::map<unsigned int, unsigned int> remapped_label_index_map;
};

#endif