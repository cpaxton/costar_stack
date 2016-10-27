#ifndef SEMANTIC_SEGMENTATION_H
#define SEMANTIC_SEGMENTATION_H

#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/greedyObjRansac.h"
#include "sp_segmenter/plane.h"
#include "sp_segmenter/refinePoses.h"
#include "sp_segmenter/common.h"
#include "sp_segmenter/stringVectorArgsReader.h"
#include "sp_segmenter/seg.h"
#include "sp_segmenter/tracker.h"

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// for TF services
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <costar_objrec_msgs/DetectedObject.h>
#include <costar_objrec_msgs/DetectedObjectList.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// chi objrec ransac utils
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
// contains function to normalize the orientation of symmetric object
#include "sp_segmenter/spatial_pose.h"
//#include "sp_segmenter/symmetricOrientationRealignment.h"
#include "sp_segmenter/table_segmenter.h"

// ros service messages for segmenting gripper
#include "sp_segmenter/segmentInGripper.h"
#include "sp_segmenter/segmenterTFObject.h"

#define OBJECT_MAX 100
class semanticSegmentation
{
private:
    ros::NodeHandle nh;
    
    bool classReady, useTFinsteadOfPoses;
    // TF related
    bool hasTF, doingGripperSegmentation;
    bool useObjectPersistence;
    objectRtree segmentedObjectTree;

    // std::vector<value> sp_segmenter_detectedPoses; // value: std::pair<point3d,objectPose>. see spatial_pose.h
    std::string targetNormalObjectTF;
    std::string targetTFtoUpdate;
    bool setObjectOrientationTarget;
    std::vector<segmentedObjectTF> segmentedObjectTFV;

    // TODO(ahundt): re-enable this to use previous positions of each object
    // the object poses result in TF. depreciate, used TF instead
    // std::map<std::string, segmentedObjectTF> segmentedObjectTFMap;

    // keep information about TF index
    std::map<std::string, unsigned int> objectTFIndex;
    tf::TransformListener * listener;

    // keep information about gripper TF frame for object in gripper segmentation
    std::string gripperTF;

    // map of symmetries for orientation normalization
    std::map<std::string, objectSymmetry> objectDict;

    bool loadTable, haveTable, useTableSegmentation;
    double tableDistanceThreshold, tableAngularThreshold;
    int tableMinimalInliers;
    tf::TransformBroadcaster br;
    ros::ServiceServer spSegmenter;
    ros::ServiceServer segmentGripper;
    bool useBinarySVM, useMultiClassSVM;
    
    // Point cloud related
    sensor_msgs::PointCloud2 inputCloud; // cache the point cloud
    pcl::PointCloud<PointT>::Ptr tableConvexHull; // for object in table segmentation
    double aboveTableMin, aboveTableMax; // point cloud need to be this value above the table in meters

    // Publisher related
    unsigned int table_corner_published;
    std::string POINTS_IN, POINTS_OUT, POSES_OUT;
    ros::Publisher pc_pub, pose_pub, detected_object_pub, table_corner_pub;
    ros::Subscriber pc_sub;
    unsigned int number_of_segmentation_done;
    
    // Segmentation related
    bool compute_pose;
    bool view_flag;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    Hier_Pooler hie_producer;
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set;
    std::vector<model*> binary_models;
    std::vector<model*> multi_models;
    float radius, ratio;
    float down_ss;
    double pairWidth;
    double voxelSize;
    std::string objRecRANSACdetector;
    double minConfidence;
    
    std::vector<boost::shared_ptr<greedyObjRansac> > objrec;
    boost::shared_ptr<greedyObjRansac> combinedObjRec;
    std::vector<std::string> model_name;
    std::vector<ModelT> mesh_set;
    
    std::map<std::string, int> model_name_map;
    uchar color_label[11][3];

    std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > cloud_vec;
    int maxframes;
    int cur_frame_idx;
    bool cloud_ready, use_median_filter;

    boost::shared_ptr<Tracker> tracker;
    bool enableTracking;
    
    bool useCropBox;
    tf::StampedTransform table_transform;
    Eigen::Vector3f crop_box_size;
   
protected:
//    void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3]);
    std::vector<poseT> spSegmenterCallback(const pcl::PointCloud<PointT>::Ptr full_cloud, pcl::PointCloud<PointLT> & final_cloud);
    bool getAndSaveTable (const sensor_msgs::PointCloud2 &pc);
    void updateCloudData (const sensor_msgs::PointCloud2 &pc);
    void initializeSemanticSegmentation();
    void populateTFMapFromTree();
    void cropPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_input, 
      const Eigen::Affine3f& camera_tf_in_table, 
      const Eigen::Vector3f& box_size);
public:
    semanticSegmentation(int argc, char** argv);
    semanticSegmentation(int argc, char** argv, const ros::NodeHandle &nh);
    ~semanticSegmentation();
    void setNodeHandle(const ros::NodeHandle &nh);
    void publishTF();
    void callbackPoses(const sensor_msgs::PointCloud2 &inputCloud);
    bool serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool serviceCallbackGripper (sp_segmenter::segmentInGripper::Request & request, sp_segmenter::segmentInGripper::Response& response);
};
#endif
