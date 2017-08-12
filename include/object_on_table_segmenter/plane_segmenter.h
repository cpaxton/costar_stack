#ifndef PLANE_SEGMENTER
#define PLANE_SEGMENTER

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>

// plane segmentation
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

// segment object above plane
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/project_inliers.h>

// for eucledian clustering
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

#include <sstream>
#include <vector>

typedef pcl::PointCloud<pcl::PointXYZRGBA> CloudXYZRGBA;

class PlaneSegmenter
{
public:
	PlaneSegmenter() : ready(false),
		plane_convex_hull_(new CloudXYZRGBA()), 
		distance_threshold_(0.02), angular_threshold_(2.0), minimal_inliers_(5000), 
		min_above_plane_(0.01), max_above_plane_(0.5),
		visualize_(false) {};

	// Returns all points inside a box with dimension `box_size` located at `box_pose`
	CloudXYZRGBA::Ptr cropBox(const CloudXYZRGBA &input_cloud, const Eigen::Affine3f &box_pose, const Eigen::Vector3f &box_size) const;
	// Generate plane convex hull from biggest planar area in the input cloud
	CloudXYZRGBA::Ptr generatePlaneConvexHull(const CloudXYZRGBA &input_cloud) const;
	// Generate 4 corner points based on XY plane of the `pose` with dimension `size` x `size`
	CloudXYZRGBA::Ptr generatePlaneConvexHullFromPoseXYplane(const Eigen::Affine3f &pose, const double &size = 0.5) const;

	// Perform eucledian clusterization from input cloud
	std::vector<CloudXYZRGBA::Ptr> clusterPointCloud(const CloudXYZRGBA &input_cloud, const unsigned int &minimal_cluster_size = 500);

	// Returns all points above the input plane convex hull
	CloudXYZRGBA::Ptr segmentAbovePlane(const CloudXYZRGBA &input_cloud, const CloudXYZRGBA &plane_convex_hull,
		const double &min_above_plane, const double &max_above_plane) const;
	// Returns all points above the input plane convex hull with parameters from setPlaneSegmentationHeight
	CloudXYZRGBA::Ptr segmentAbovePlane(const CloudXYZRGBA &input_cloud, const CloudXYZRGBA &plane_convex_hull) const;

	// Returns all points above the saved plane convex hull from setPlaneConvexHull function
	CloudXYZRGBA::Ptr segmentAbovePlane(const CloudXYZRGBA &input_cloud, 
		const double &min_above_plane, const double &max_above_plane) const;
	// Returns all points above the saved plane convex hull from setPlaneConvexHull function with parameters from setPlaneSegmentationHeight
	CloudXYZRGBA::Ptr segmentAbovePlane(const CloudXYZRGBA &input_cloud) const;

	void setPlaneSegmentationParameters(const double &distance_threshold, const double &angular_threshold, const unsigned int &minimal_inliers);
	void setPlaneConvexHull(const CloudXYZRGBA &plane_convex_hull);
	void setPlaneSegmentationHeight(const double &min_above_plane, const double &max_above_plane);
	void setVisualizationFlag(const bool visualize = true);

	CloudXYZRGBA getPlaneConvexHull() const { return *plane_convex_hull_; };
	// Flag that shows whether the plane_convex_hull has been set or not
	bool ready;
	;
private:
	void organizedPlaneSegmentation(
		const CloudXYZRGBA &input_cloud,
		std::vector< pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > &regions,
		pcl::PointCloud<pcl::Label>::Ptr &labels,
		std::vector<pcl::PointIndices> &label_indices) const;
	CloudXYZRGBA::Ptr plane_convex_hull_;
	pcl::visualization::PCLVisualizer::Ptr viewer_;
	double distance_threshold_;
	double angular_threshold_;
	unsigned int minimal_inliers_;
	double min_above_plane_;
	double max_above_plane_;

	bool visualize_;
	;
};

#endif
