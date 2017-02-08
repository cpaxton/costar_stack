#ifndef TABLE_SEGMENTER
#define TABLE_SEGMENTER

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

// for pcl segmentation
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>

// for segment object above table
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/filters/project_inliers.h>

#include "sp_segmenter/utility/typedef.h"

void segmentCloudAboveTable(pcl::PointCloud<PointT>::Ptr &cloud_input, const pcl::PointCloud<PointT>::Ptr &convexHull, const double aboveTableMin = 0.01, const double aboveTableMax = 0.25);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getTableConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr negative, 
	pcl::visualization::PCLVisualizer::Ptr viewer,
	double distanceThreshold = 0.02, double angularThreshold = 2.0, int minimalInliers = 5000);

#endif
