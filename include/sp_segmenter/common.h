#ifndef SP_SEGMENTER_TRAIN_H
#define SP_SEGMENTER_TRAIN_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"


pcl::PointCloud<PointLT>::Ptr genSeg(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map);

pcl::PointCloud<PointT>::Ptr AveragePointCloud(const std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > &cloud_vec);

void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3]);

pcl::PointCloud<PointLT>::Ptr densifyLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, const pcl::PointCloud<PointT>::Ptr ori_cloud);

std::vector<int> spGt(const pcl::PointCloud<PointT>::Ptr gt_cloud, const std::vector<pcl::PointCloud<PointT>::Ptr> segs);

std::vector<std::string> readMesh(std::string mesh_path, std::vector<ModelT> &model_set);

std::vector<poseT> readGT(std::string pose_path, std::string file_id);

#endif