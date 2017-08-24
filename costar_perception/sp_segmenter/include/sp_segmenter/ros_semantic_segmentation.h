#ifndef ROS_SEMANTIC_SEGMENTATION_H
#define ROS_SEMANTIC_SEGMENTATION_H

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// for TF services
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#ifdef COSTAR
#include <costar_objrec_msgs/DetectedObject.h>
#include <costar_objrec_msgs/DetectedObjectList.h>
#endif

#ifdef USE_TRACKING
#include "sp_segmenter/tracker.h"
#endif

#ifdef SCENE_PARSING
#include <objrec_hypothesis_msgs/Hypothesis.h>
#include <objrec_hypothesis_msgs/ModelHypothesis.h>
#include <objrec_hypothesis_msgs/AllModelHypothesis.h>
#endif

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

#include "sp_segmenter/semantic_segmentation.h"

// ros service messages for segmenting gripper
#include "sp_segmenter/SegmentInGripper.h"

struct segmentedObjectTF
{
    tf::Transform transform;
    std::string TFname;
    segmentedObjectTF(const ObjectTransformInformation &input);
    segmentedObjectTF();
    tf::StampedTransform generateStampedTransform(const std::string &parent) const;
};

class RosSemanticSegmentation : public SemanticSegmentation
{
public:
    RosSemanticSegmentation();
    RosSemanticSegmentation(const ros::NodeHandle &nh);
    void setNodeHandle(const ros::NodeHandle &nh);
    void publishTF();
    void callbackPoses(const sensor_msgs::PointCloud2 &inputCloud);
    bool serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
#ifdef COSTAR
    bool serviceCallbackGripper (sp_segmenter::SegmentInGripper::Request & request, sp_segmenter::SegmentInGripper::Response& response);
#endif

#ifdef USE_TRACKING
    void setEnableTracking(const bool &tracking_flag);
#endif

protected:
    void processExternalSegmentationResult(const std_msgs::Empty &input_msgs);
    bool getAndSaveTable (const sensor_msgs::PointCloud2 &pc);
    void updateCloudData (const sensor_msgs::PointCloud2 &pc);
    void initializeSemanticSegmentationFromRosParam();
    void populateTFMap(std::vector<ObjectTransformInformation> all_poses);

    costar_objrec_msgs::DetectedObjectList last_object_list_;
    sensor_msgs::PointCloud2 last_segmented_cloud_;
#ifdef SCENE_PARSING
    objrec_hypothesis_msgs::AllModelHypothesis generateAllModelHypothesis() const;
    bool getLastHypotheses (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
#endif

    ros::NodeHandle nh;
    bool useTFinsteadOfPoses;

    // TF related
    bool hasTF;

    std::string targetNormalObjectTF;
    std::vector<segmentedObjectTF> segmentedObjectTFV;

    tf::TransformListener * listener;
    tf::TransformBroadcaster br;

    // keep information about gripper TF frame for object in gripper segmentation
    std::string gripperTF;

    ros::ServiceServer spSegmenter;
    ros::ServiceServer segmentGripper;
    
    // Point cloud related
    sensor_msgs::PointCloud2 inputCloud; // cache the point cloud

    // Publisher related
    unsigned int table_corner_published;
    std::string POINTS_IN, POINTS_OUT, POSES_OUT;
    ros::Publisher pc_pub, pose_pub, detected_object_pub, table_corner_pub;
    ros::Subscriber pc_sub, external_segmentation_sub;
    unsigned int number_of_segmentation_done;
    
    std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > cloud_vec;
    int maxframes;
    int cur_frame_idx;
    bool cloud_ready, use_median_filter;

    tf::StampedTransform table_transform, preferred_transform;
    Eigen::Vector3f crop_box_size, crop_box_gripper_size;
    Eigen::Affine3d crop_box_pose_table_;
    bool has_crop_box_pose_table_, use_crop_box_, need_preferred_tf_ ;

#ifdef SCENE_PARSING
    ros::Publisher hypothesis_pub_;
    ros::ServiceServer last_hypotheses_server_;
#endif
    

#ifdef USE_TRACKING
    boost::shared_ptr<Tracker> tracker_;
    bool use_tracking_;
#endif
};
#endif
