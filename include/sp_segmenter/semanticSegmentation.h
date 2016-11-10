#ifndef SEMANTIC_SEGMENTATION_H
#define SEMANTIC_SEGMENTATION_H

#include <time.h> 
#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>

#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/plane.h"
#include "sp_segmenter/refinePoses.h"
#include "sp_segmenter/common.h"
#include "sp_segmenter/stringVectorArgsReader.h"
#include "sp_segmenter/seg.h"
#include "sp_segmenter/spatial_pose.h"
#include "sp_segmenter/table_segmenter.h"

#ifdef USE_OBJRECRANSAC
#include "sp_segmenter/greedyObjRansac.h"

enum ObjRecRansacMode {STANDARD_BEST, STANDARD_RECOGNIZE, GREEDY_RECOGNIZE};

struct ModelObjRecRANSACParameter
{
    ModelObjRecRANSACParameter() : pair_width_(0.1), voxel_size_(0.003), object_visibility_(0.1), scene_visibility_(0.1)
    {};

    ModelObjRecRANSACParameter(const double &pair_width, const double &voxel_size) : pair_width_(pair_width), voxel_size_(voxel_size), object_visibility_(0.1), scene_visibility_(0.1)
    {};

    void setPairWidth(const double &pair_width);
    void setVoxelSize(const double &voxel_size);
    void setObjectVisibility(const double &object_visibility);
    void setSceneVisibility(const double &scene_visibility);

    double pair_width_;
    double voxel_size_;
    double object_visibility_;
    double scene_visibility_;

};
#endif

#define OBJECT_MAX 100
struct objectTransformInformation
{
    // extended from poseT. All transform is with reference to the camera frame
    std::string transform_name_;
    std::string model_name_;
    Eigen::Vector3f origin_;
    Eigen::Quaternion<float> rotation_;
    double confidence_;

    objectTransformInformation() {};
    objectTransformInformation(const std::string &transform_name, const poseT &ObjRecRANSAC_result) : transform_name_(transform_name), model_name_(ObjRecRANSAC_result.model_name),
        origin_(ObjRecRANSAC_result.shift), rotation_(ObjRecRANSAC_result.rotation), confidence_(ObjRecRANSAC_result.confidence)
    {};
};

class semanticSegmentation
{
public:
    semanticSegmentation();
    ~semanticSegmentation();

    bool getTableSurfaceFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, const bool &save_table_pcd = false, const std::string &save_directory_path = ".");
    bool segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, pcl::PointCloud<PointLT>::Ptr &result);

#ifdef USE_OBJRECRANSAC
    std::vector<objectTransformInformation> calculateObjTransform(const pcl::PointCloud<PointLT>::Ptr &labelled_point_cloud);
    std::vector<objectTransformInformation> getUpdateOnOneObjTransform(const pcl::PointCloud<PointLT>::Ptr &labelled_point_cloud, const std::string &transform_name, const std::string &object_type);
    bool segmentAndCalculateObjTransform(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, 
        pcl::PointCloud<PointLT>::Ptr &labelled_point_cloud_result, std::vector<objectTransformInformation> &object_transform_result);
#endif

    void setDirectorySHOT(const std::string &path_to_shot_directory);
    void setDirectorySVM(const std::string &path_to_svm_directory, bool multi_model_svm_);
    void setUseMultiClassSVM(bool use_multi_class_svm);
    void setUseBinarySVM(bool use_binary_svm);
    void setPointCloudDownsampleValue(float down_ss);
    void setHierFeaRatio(float ratio);

    void setUseVisualization(bool visualization_flag);

    // Table Segmentation Functions
    void setCropBoxSize(const double &x, const double &y, const double &z);
    void setCropBoxPose(const Eigen::Affine3f &target_pose_relative_to_camera_frame);
    void setUseTableSegmentation(bool use_table_segmentation);
    void setCropAboveTableBoundary(const double &min, const double &max);
    void loadTableFromFile(const std::string &table_pcd_path);
    void setTableSegmentationParameters(double table_distance_threshold, bool table_angular_threshold, unsigned int table_minimal_inliers);

#ifdef USE_OBJRECRANSAC
    void setUseComputePose(bool compute_pose);
    void addModel(const std::string &path_to_model_directory, const std::string &model_name, const ModelObjRecRANSACParameter &parameter);
    void addModelSymmetricProperty(const std::string &model_name, const double &roll, const double &pitch, const double &yaw, const double &step, const std::string &preferred_axis);
    void setModeObjRecRANSAC(const int &mode);
    void setMinConfidenceObjRecRANSAC(const double &min_confidence);
    void setUseObjectPersistence(const bool use_object_persistence);

    template <typename numericalData>
    void setPreferredOrientation(Eigen::Quaternion<numericalData> base_rotation);
#endif

protected:
    void initializeSemanticSegmentation();
    void cropPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_input, 
      const Eigen::Affine3f& camera_transform_in_target, 
      const Eigen::Vector3f& box_size);

    bool class_ready_;
    bool visualizer_flag_;

    // Point Cloud Modifier Parameter
    Eigen::Vector3f crop_box_size_;
    bool use_crop_box_;
    Eigen::Affine3f crop_box_target_pose_;

    // Training File Informations
    bool svm_loaded_, shot_loaded_;
    bool use_binary_svm_, use_multi_class_svm_;
    std::vector<model*> binary_models_, multi_models_;
    std::vector<ModelT> mesh_set_;
    std::map<std::string, std::size_t> model_name_map_;
    std::size_t number_of_added_models_;

    // Table Parameters
    bool have_table_, use_table_segmentation_;
    double table_distance_threshold_, table_angular_threshold_;
    unsigned int table_minimal_inliers_;
    double above_table_min, above_table_max;
    pcl::PointCloud<PointT>::Ptr table_corner_points_;

    // Point Cloud Segmentation Parameters
    float pcl_downsample_;
    float hier_radius_, hier_ratio_;
    bool compute_pose_;
    bool use_cuda_;

    // ObjRecRANSAC Parameters
    int objRecRANSAC_mode_;
    double min_objrecransac_confidence;

    // Object Transformation Parameter
    bool use_preferred_orientation_;
    Eigen::Quaternion<double> base_rotation_;
    bool use_object_persistence_;

#ifdef USE_OBJRECRANSAC
    boost::shared_ptr<greedyObjRansac> combined_ObjRecRANSAC_;
    std::vector<boost::shared_ptr<greedyObjRansac> > individual_ObjRecRANSAC_;
    // keep information about TF index
    std::map<std::string, unsigned int> object_class_transform_index_;

    // map of symmetries for orientation normalization
    std::map<std::string, objectSymmetry> object_dict_;
    objectRtree segmented_object_tree_;
#endif

    pcl::visualization::PCLVisualizer::Ptr viewer;
    Hier_Pooler hie_producer;
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set;
    
    uchar color_label[11][3];
};
#endif
