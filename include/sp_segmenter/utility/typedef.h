#pragma once

#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#define OPENCV_SIFT

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/usc.h>
#include <pcl/features/vfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <boost/iterator/filter_iterator.hpp>

#include <pcl/surface/mls.h>
#include <pcl/correspondence.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/common/norms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ply/ply.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/impl/vtk_lib_io.hpp>
#include <sys/time.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/supervoxel_clustering.h>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/core/eigen.hpp>

#ifdef USE_OPENMP
#include <omp.h>
#endif // USE_OPENMP

#define EPS 0.0000000001
#define PI 3.14159265358979

#define HNUM 50
#define SNUM 20
#define INUM 10
#define THREADNUM 8
#define FOCAL_LEN 525.0

#define INF_ 10000000

#define BB_INST_MAX 125
#define BB_INST_PER_NUM 1392
#define UW_CLASS_MAX 51
#define UW_INST_MAX 300
#define JHU_INST_MAX 50
#define OBJ_INST_MAX 500

/* Define some custom types to make the rest of our code easier to read */
typedef pcl::PointXYZ myPointXYZ;
typedef pcl::PointXYZRGBA PointT;
typedef pcl::Normal NormalT;
typedef pcl::PointXYZL PointLT;

typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 SHOTdescr;
typedef pcl::SHOT1344 Descriptor3DType;

struct PointXYZRGBIM
{
  union
  {
    struct
    {
      float x;
      float y;
      float z;
      float rgb;
      float imX;
      float imY;
    };
    float data[6];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBIM,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, rgb, rgb)
                                    (float, imX, imX)
                                    (float, imY, imY)
)
typedef PointXYZRGBIM myPointT;

struct Cloud_Image
{
    pcl::PointCloud<myPointT>::Ptr cloud;
    pcl::PointCloud<myPointXYZ>::Ptr surface;
    pcl::PointCloud<NormalT>::Ptr normals;
    cv::Mat rgb;
    cv::Mat depth;
    cv::Mat map2D3D;
    cv::Mat gray;
};

struct keyT
{
    myPointXYZ xyz;
    NormalT normal;
    std::vector<myPointXYZ> shift_vec;	//shift from the object center, in its own local reference frame
    std::vector<float> w_shift;		//use after normalization
    pcl::ReferenceFrame ref_frame;	//local reference frame
};

struct keyDescrT
{
    int fea_type;	// 0: sift, 1: shot, 2: fpfh
    uint32_t idx;       // index for keyT vector
    cv::Mat feaDescr;	// descriptor
};

struct keys_pool
{
    cv::Mat fea_pool;
    cv::flann::Index fea_tree;
    std::vector<int> map_idx;	//map keys to keyT vector
};

struct MulInfoT{
    cv::Mat xyz;
    cv::Mat uv;
    cv::Mat rgb;
    cv::Mat normal;
    pcl::PointCloud<PointT>::Ptr cloud;
    pcl::PointCloud<NormalT>::Ptr cloud_normals;
    
    pcl::PointCloud<PointT>::Ptr down_cloud;
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr down_lrf;
    int w,h;
    cv::Mat map2d;
    cv::Mat img;
    cv::Mat _3d2d;
    
    pcl::search::KdTree<PointT>::Ptr xyz_tree;
    pcl::search::KdTree<myPointXYZ>::Ptr lab_tree;
    pcl::search::KdTree<NormalT>::Ptr normal_tree;   
};

typedef std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poseVec;
typedef std::vector< pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > CloudSet;
typedef std::vector< pcl::PointCloud<NormalT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<NormalT>::Ptr> > NormalSet;
typedef std::vector< std::vector<MulInfoT> > ObjectSet;

//*
// previous stuff
struct PointObj
{
    union
    {
        float data_xyz[4];
        struct
        {
            float x;
            float y;
            float z;
        };
    };
    union
    {
        float data_n[4];
        float normal[3];
        struct
        {
            float normal_x;
            float normal_y;
            float normal_z;
        };
    };
    union
    {
        struct
        {
            float curvature;
        };
        float data_c[4];
    };
    union
    {
        float data_s[4];
        float shift[3];
        struct
        {
            float shift_x;
            float shift_y;
            float shift_z;
        };
    };
    union
    {
        struct
        {
            float x_axis[3];
            float y_axis[3];
            float z_axis[3];
        };
        float rf[12];
    };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointObj,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        //(uint32_t, rgb, rgb)
        (float, curvature, curvature)
        (float, normal_x, normal_x)
        (float, normal_y, normal_y)
        (float, normal_z, normal_z)
        (float, shift_x, shift_x)
        (float, shift_y, shift_y)
        (float, shift_z, shift_z)
        (float[3], x_axis, x_axis)
        (float[3], y_axis, y_axis)
        (float[3], z_axis, z_axis)
)

struct SIFTDescr{
    float siftDescr[128];
    
EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (SIFTDescr,
                    (float[128], siftDescr, siftDescr)
)

static bool comp1( const std::pair<int, int> &p1, const std::pair<int, int> &p2 )
{
    return p1.second > p2.second;
}

static bool comp2( const std::pair<int, float> &p1, const std::pair<int, float> &p2 )
{
    return p1.second < p2.second;
}

#endif
