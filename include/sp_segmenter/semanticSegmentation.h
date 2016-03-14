#ifndef SEMANTIC_SEGMENTATION_H
#define SEMANTIC_SEGMENTATION_H

#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/greedyObjRansac.h"
#include "sp_segmenter/plane.h"
#include "sp_segmenter/refinePoses.h"
#include "sp_segmenter/common.h"
#include "sp_segmenter/stringVectorArgsReader.h"

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// for TF services
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// chi objrec ransac utils
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
// contains function to normalize the orientation of symmetric object
#include "sp_segmenter/symmetricOrientationRealignment.h"
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
    bool hasTF;
    std::map<std::string, segmentedObjectTF> segmentedObjectTFMap; // the object poses result in TF
    std::map<std::string, unsigned int> objectTFIndex; // keep information about TF index
    tf::TransformListener * listener;
    std::string gripperTF; // keep information about gripper TF frame for object in gripper segmentation
    std::map<std::string, objectSymmetry> objectDict; // for orientation normalization
    bool loadTable, haveTable;
    tf::TransformBroadcaster br;
    ros::ServiceServer spSegmenter;
    ros::ServiceServer segmentGripper;
    
    // Point cloud related
    sensor_msgs::PointCloud2 inputCloud; // cache the point cloud
    pcl::PointCloud<PointT>::Ptr tableConvexHull; // for object in table segmentation
    double aboveTable; // point cloud need to be this value above the table in meters
    std::string POINTS_IN, POINTS_OUT, POSES_OUT;
    ros::Publisher pc_pub, pose_pub;
    ros::Subscriber pc_sub;
    
    // Segmentation related
    bool compute_pose;
    bool view_flag;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    Hier_Pooler hie_producer;
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set;
    std::vector<model*> binary_models;
    float radius, ratio;
    float down_ss;
    double pairWidth;
    double voxelSize;
    bool bestPoseOnly;
    double minConfidence;
    
    boost::shared_ptr<greedyObjRansac> objrec;
    std::vector<std::string> model_name;
    std::vector<ModelT> mesh_set;
    
    std::map<std::string, int> model_name_map;
    uchar color_label[11][3];
;
protected:
//    void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3]);
    std::vector<poseT> spSegmenterCallback(const pcl::PointCloud<PointT>::Ptr full_cloud, pcl::PointCloud<PointLT> & final_cloud);
    bool getAndSaveTable (const sensor_msgs::PointCloud2 &pc);
    void updateCloudData (const sensor_msgs::PointCloud2 &pc);
    void initializeSemanticSegmentation();
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
