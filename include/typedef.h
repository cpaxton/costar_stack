#ifndef TYPEDEF_H
#define TYPEDEF_H

#include <vector>
#include <map>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <btBulletDynamicsCommon.h>

 // ObjectParameter == Object Pose
typedef btTransform ObjectParameter;
// typedef Eigen::Transform< double,3,Eigen::Affine > ObjectParameter;
typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ImagePtr;
typedef pcl::PointXYZRGBA ImagePoint;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Image;

// this contains model name and pose hypothesis of a single object
typedef std::pair<std::string, std::vector<ObjectParameter> > ObjectHypothesesData;


#endif