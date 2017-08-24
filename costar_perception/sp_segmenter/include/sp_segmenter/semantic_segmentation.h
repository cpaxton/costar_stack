#ifndef SEMANTIC_SEGMENTATION_H
#define SEMANTIC_SEGMENTATION_H

#include <time.h> 
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>
#include <boost/filesystem.hpp>

#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/plane.h"
// #include "sp_segmenter/refinePoses.h"
#include "sp_segmenter/common.h"
#include "sp_segmenter/seg.h"
#include "sp_segmenter/spatial_pose.h"
#include "sp_segmenter/table_segmenter.h"

enum ObjRecRansacMode {STANDARD_BEST, STANDARD_RECOGNIZE, GREEDY_RECOGNIZE};
enum RgbToLabelChannel {RED, GREEN, BLUE, ALL};

#ifdef USE_OBJRECRANSAC
#include "sp_segmenter/greedyObjRansac.h"
#endif


struct ModelObjRecRANSACParameter
{
    ModelObjRecRANSACParameter() : pair_width_(0.1), voxel_size_(0.003), object_visibility_(0.1), scene_visibility_(0.1)
    {};

    ModelObjRecRANSACParameter(const double &pair_width, const double &voxel_size) : pair_width_(pair_width), voxel_size_(voxel_size), object_visibility_(0.1), scene_visibility_(0.1)
    {};

    ModelObjRecRANSACParameter(const double &pair_width, const double &voxel_size, const double &object_visibility, const double &scene_visibility) : 
        pair_width_(pair_width), voxel_size_(voxel_size), object_visibility_(object_visibility), scene_visibility_(scene_visibility)
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

#define OBJECT_MAX 100
struct ObjectTransformInformation
{
    // extended from poseT. All transform is with reference to the camera frame
    std::string transform_name_;
    std::string model_name_;
    unsigned int model_index_;
    Eigen::Vector3f origin_;
    Eigen::Quaternion<float> rotation_;
    double confidence_;

    ObjectTransformInformation() {};
    ObjectTransformInformation(const std::string &transform_name, const poseT &ObjRecRANSAC_result, unsigned const int &model_index) : transform_name_(transform_name), 
        model_name_(ObjRecRANSAC_result.model_name), model_index_(model_index),
        origin_(ObjRecRANSAC_result.shift), rotation_(ObjRecRANSAC_result.rotation), confidence_(ObjRecRANSAC_result.confidence)
    {};

    poseT asPoseT() const
    {
        poseT result;
        result.model_name = this->model_name_;
        result.shift = this->origin_;
        result.rotation = this->rotation_;
        result.confidence = this->confidence_;
        return result;
    }

    friend ostream& operator<<(ostream& os, const ObjectTransformInformation &tf_info);
    
    bool operator==(const ObjectTransformInformation& other) const;

    void print() const
    {
        std::cout << *this;
    }
};

class SemanticSegmentation
{
public:
    SemanticSegmentation();
    ~SemanticSegmentation();
// ---------------------------------------------------------- MAIN OPERATIONAL FUNCTIONS --------------------------------------------------------------------------------------------

    // initializeSemanticSegmentation needs to be called after all main parameter has been set. It will check whether all parameters has been set properly or not
    void initializeSemanticSegmentation();

    // segment input point cloud pcl::PointXYZRGBA to labelled point cloud pcl::PointXYZL
    bool segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &result);
#ifdef USE_OBJRECRANSAC
    // calculate all object poses based on the input labelled point cloud generated from segmentPointCloud function.
    std::vector<ObjectTransformInformation> calculateObjTransform(const pcl::PointCloud<pcl::PointXYZL>::Ptr &labelled_point_cloud);

    // segment and calculate all object poses based on input cloud. It returns true if the segmentation successful and the detected object poses > 0
    bool segmentAndCalculateObjTransform(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, 
        pcl::PointCloud<pcl::PointXYZL>::Ptr &labelled_point_cloud_result, std::vector<ObjectTransformInformation> &object_transform_result);

    // Update one object pose that has matching transform name and object type, then returns that updated pose with other poses(from previous detection). 
    std::vector<ObjectTransformInformation> getUpdateOnOneObjTransform(const pcl::PointCloud<pcl::PointXYZL>::Ptr &labelled_point_cloud, const std::string &transform_name, const std::string &object_type);

    pcl::PointCloud<pcl::PointXYZL>::Ptr convertRgbChannelToLabelCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, const int &channel_to_convert);
    
#endif

// ---------------------------------------------------------- ADDITIONAL OPERATIONAL FUNCTIONS --------------------------------------------------------------------------------------
    bool getTableSurfaceFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, const bool &save_table_pcd = false, const std::string &save_directory_path = ".");
    void convertPointCloudLabelToRGBA(const pcl::PointCloud<pcl::PointXYZL>::Ptr &input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &output) const;
// --------------------------------- MAIN PARAMETERS for point cloud segmentation that needs to be set before initializeSemanticSegmentation ----------------------------------------
    void setDirectorySHOT(const std::string &path_to_shot_directory);
    void setDirectoryFPFH(const std::string &path_to_fpfh_directory);
    void setDirectorySIFT(const std::string &path_to_sift_directory);

    void setUseSHOT(const bool &use_shot) { use_shot_ = use_shot; }
    void setUseFPFH(const bool &use_fpfh) { use_fpfh_ = use_fpfh; }
    void setUseSIFT(const bool &use_sift) { use_sift_ = use_sift; }
    
    void setDirectorySVM(const std::string &path_to_svm_directory);
    void setDirectorySVM(const std::string &path_to_svm_directory, const bool &use_binary_svm, const bool &use_multi_class_svm);
    void setUseMultiClassSVM(const bool &use_multi_class_svm);
    void setUseBinarySVM(const bool &use_binary_svm);
    template <typename NumericType>
        void setPointCloudDownsampleValue(const NumericType &down_ss);
    template <typename NumericType>
        void setHierFeaRatio(const NumericType &ratio);
    template <typename NumericType>
        void setPoseConsistencyMaximumDistance(const NumericType &distance);

// --------------------------------- MAIN PARAMETERS for ObjRecRANSAC that needs to be set before initializeSemanticSegmentation if compute pose is used-------------------------------

#ifdef USE_OBJRECRANSAC
    void setUseComputePose(const bool &compute_pose);
    void setUseCuda(const bool &use_cuda);
    void setUseCombinedObjRecRANSAC(const bool &use_combined_objRecRANSAC);
    void setModeObjRecRANSAC(const int &mode);
    template <typename NumericType>
        void setMinConfidenceObjRecRANSAC(const NumericType &min_confidence);

    // IMPORTANT: Add model need to follow a particular order if we are using multiple objects.
    // This order should follow the name of SVM directory.
    // For example, if the svm directory name is "link_node_sander_svm", then the order of adding model is: link model, node model, and sander model. 
    void addModel(const std::string &path_to_model_directory, const std::string &model_name, const ModelObjRecRANSACParameter &parameter);

#ifdef SCENE_PARSING
    std::vector<GreedyHypothesis> getHypothesisList() const;
#endif

#endif

// --------------------------------- OPTIONAL PARAMETERS that does not need to be set before initializeSemanticSegmentation, but may help in some cases -------------------------------
    // Visualize the point cloud segmentation with pcl visualizer
    void setUseVisualization(const bool &visualization_flag);

    // Crop box will crop the point cloud by making a box with crop_box_size_ around the crop_box_pose. It can be used to remove background data without using binarySVM
    void setUseCropBox(const bool &use_crop_box);
    template <typename NumericType>
        void setCropBoxSize(const NumericType &x, const NumericType &y, const NumericType &z);
    template <typename NumericType>
        void setCropBoxSize(const Eigen::Matrix<NumericType, 3, 1> &crop_box_size);
    template <typename NumericType>
        void setCropBoxPose(const Eigen::Transform< NumericType, 3, Eigen::Affine> &target_pose_relative_to_camera_frame);

    // Table Segmentation will extract the points above the table region. It can be used to remove background data without using binarySVM
    void setUseTableSegmentation(const bool &use_table_segmentation);
    template <typename NumericType>
        void setCropAboveTableBoundary(const NumericType &min, const NumericType &max);
    void loadTableFromFile(const std::string &table_pcd_path);
    template <typename NumericType1, typename NumericType2, typename NumericType3>
        void setTableSegmentationParameters(const NumericType1 &table_distance_threshold,const NumericType2 &table_angular_threshold,const NumericType3 &table_minimal_inliers);

    // Symmetric parameters will post-process ObjRecRANSAC pose to achieve more consistent pose for all objects that has symmetric properties (e.g. boxes)
    // by returning a pose that is closest to a preferred orientation or identity (if the preferred orientation is not set). If the symmetric parameter is not set, the object assumed to have no symmetric property
#ifdef USE_OBJRECRANSAC
    template <typename NumericType>
        void addModelSymmetricProperty(const std::string &model_name, const NumericType &roll, const NumericType &pitch, const NumericType &yaw, const NumericType &step, const std::string &preferred_axis);
    void addModelSymmetricProperty(const std::map<std::string, ObjectSymmetry> &object_dict);
    void setUsePreferredOrientation(const bool &use_preferred_orientation);
    
    template <typename NumericType>
        void setPreferredOrientation(const Eigen::Quaternion<NumericType> &base_rotation);

    // Object persistence will post-process ObjRecRANSAC pose to get an object orientation that is closest to the previous detection, 
    // if the detected object position is within 2.5 cm compared to previous object position
    void setUseObjectPersistence(const bool &use_object_persistence);
    void setUseExternalSegmentation(const bool &use_external_segmentation);
#endif

#ifdef USE_TRACKING
    std::vector<ModelT> getMeshSet() const
    {
        return this->mesh_set_;
    }
#endif

protected:
    void cropPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_input, 
        const Eigen::Affine3f &camera_transform_in_target, 
        const Eigen::Vector3f &box_size) const;
#ifdef USE_OBJRECRANSAC
    std::vector<ObjectTransformInformation> getTransformInformationFromTree(const double &current_time) const;
#endif
    bool checkFolderExist(const std::string &directory_path) const;
    bool class_ready_;
    bool visualizer_flag_;

    // Point Cloud Modifier Parameter
    Eigen::Vector3f crop_box_size_;
    bool use_crop_box_;
    Eigen::Affine3f crop_box_target_pose_;

    // Training File Informations
    bool svm_loaded_, shot_loaded_, fpfh_loaded_, sift_loaded_;
    bool use_binary_svm_, use_multi_class_svm_;
    std::vector<model*> binary_models_, multi_models_;
    std::vector<ModelT> mesh_set_;
    std::map<std::string, std::size_t> model_name_map_;
    std::size_t number_of_cloud_models_;

    // Table Parameters
    bool have_table_, use_table_segmentation_;
    double table_distance_threshold_, table_angular_threshold_;
    unsigned int table_minimal_inliers_;
    double above_table_min, above_table_max;
    pcl::PointCloud<PointT>::Ptr table_corner_points_;

    // Point Cloud Segmentation Parameters
    float pcl_downsample_;
    float hier_radius_, hier_ratio_;

    // ObjRecRANSAC Parameters
    bool compute_pose_;
    bool use_combined_objRecRANSAC_;
    bool use_cuda_;
    int objRecRANSAC_mode_;
    double min_objrecransac_confidence;

    // Object Transformation Parameter
    bool use_preferred_orientation_;
    Eigen::Quaternion<double> base_rotation_;
    bool use_object_persistence_;

    std::map<std::string, ObjectSymmetry> object_dict_;
    // keep information about TF index
    std::map<std::string, unsigned int> object_class_transform_index_;

    // map of symmetries for orientation normalization
    SpatialPose segmented_object_tree_;
    
#ifdef USE_OBJRECRANSAC
    boost::shared_ptr<greedyObjRansac> combined_ObjRecRANSAC_;
    std::map<std::size_t, std::string> cloud_idx_map; 
    std::map<std::string, boost::shared_ptr<greedyObjRansac> > individual_ObjRecRANSAC_;
    bool use_external_segmentation_;


#ifdef SCENE_PARSING
    std::vector<GreedyHypothesis> hypothesis_list_;
#endif

#endif
    pcl::visualization::PCLVisualizer::Ptr viewer;
    boost::shared_ptr<Hier_Pooler>  hie_producer;
    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set;
    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set;
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set;
    
    std::vector<cv::SiftFeatureDetector*> sift_det_vec;

    bool use_shot_, use_fpfh_, use_sift_;

    uchar color_label[11][3];
};

// template function implementation
#include "sp_segmenter/semantic_segmentation.tcc"
#endif
