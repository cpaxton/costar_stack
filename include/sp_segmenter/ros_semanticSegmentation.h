#ifndef SEMANTIC_SEGMENTATION_H
#define SEMANTIC_SEGMENTATION_H

#ifdef USE_OBJRECRANSAC
#endif

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

#include "sp_segmenter/semanticSegmentation.h"

class RosSemanticSegmentation : public semanticSegmentation
{

protected:
    ros::NodeHandle nh;
    bool classReady, useTFinsteadOfPoses;

    // TF related
    bool hasTF;

    std::string targetNormalObjectTF;
    std::string targetTFtoUpdate;
    std::vector<segmentedObjectTF> segmentedObjectTFV;

    tf::TransformListener * listener;
    tf::TransformBroadcaster br;

    // keep information about gripper TF frame for object in gripper segmentation
    std::string gripperTF, objectClassInGripper;

    ros::ServiceServer spSegmenter;
    ros::ServiceServer segmentGripper;
    
    // Point cloud related
    sensor_msgs::PointCloud2 inputCloud; // cache the point cloud

    // Publisher related
    unsigned int table_corner_published;
    std::string POINTS_IN, POINTS_OUT, POSES_OUT;
    ros::Publisher pc_pub, pose_pub, detected_object_pub, table_corner_pub;
    ros::Subscriber pc_sub;
    unsigned int number_of_segmentation_done;
    
    // Segmentation related
    bool compute_pose_;
    
    std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > cloud_vec;
    int maxframes;
    int cur_frame_idx;
    bool cloud_ready, use_median_filter;

    tf::StampedTransform table_transform;
    Eigen::Vector3f crop_box_size, crop_box_gripper_size;
   
//    void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3]);
    std::vector<poseT> spSegmenterCallback(const pcl::PointCloud<PointT>::Ptr full_cloud, pcl::PointCloud<PointLT> & final_cloud);
    bool getAndSaveTable (const sensor_msgs::PointCloud2 &pc);
    void updateCloudData (const sensor_msgs::PointCloud2 &pc);
    void initializeSemanticSegmentationFromRosParam();
public:
    semanticSegmentation();
    semanticSegmentation(const ros::NodeHandle &nh);
    ~semanticSegmentation();
    void setNodeHandle(const ros::NodeHandle &nh);
    void publishTF();
    void callbackPoses(const sensor_msgs::PointCloud2 &inputCloud);
    bool serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool serviceCallbackGripper (sp_segmenter::segmentInGripper::Request & request, sp_segmenter::segmentInGripper::Response& response);
};
#endif
