#ifndef _SP_TRACKER_HPP_
#define _SP_TRACKER_HPP_

#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_broadcaster.h>

#include "sp_segmenter/klttracker.h"
#include "sp_segmenter/utility/utility.h"

class Tracker
{
  public:
  Tracker();
  bool addTracker(const ModelT& mesh);
  void generateTrackingPoints(ros::Time stamp,
    const std::vector<poseT>& poses);

  private:

  struct TrackingInfo
  {
    TrackingInfo(ModelT mesh, unsigned int max_kps): mesh(mesh), klt_tracker(max_kps)
    {
      current_pose.setIdentity();
    }
    KLTTracker klt_tracker;
    ModelT mesh;
    Eigen::Matrix4f current_pose;
    ros::Time last_track_time;
  };
  using TrackingMap = std::map<std::string, TrackingInfo>;

  cv::Mat meshPoseToMask(pcl::PolygonMesh::Ptr pmesh, const Eigen::Matrix4f& pose_trfm, cv::Mat&);
  void publishTf(const Eigen::Matrix4f& tf, std::string name, std::string base_frame, 
    ros::Time stamp);
  void monitorQueue();
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ci);
  void imageCallback(const sensor_msgs::ImageConstPtr &im);
  void depthImageCallback(const sensor_msgs::ImageConstPtr &dep);
  void createTfViz(cv::Mat& src, cv::Mat& dst, const Eigen::Matrix4f& tf,
    const Eigen::Matrix3f& K);

  ros::NodeHandle nh;

  ros::Subscriber cam_info_sub, image_sub, depth_image_sub;
  sensor_msgs::CameraInfo cam_info;

  boost::thread callback_thread;
  boost::mutex klt_mutex, history_mutex, track_time_mutex;
  ros::CallbackQueue callback_queue;

  TrackingMap trackers; ///> map from model names to trackers

  std::vector<std::pair<ros::Time, cv::Mat> > image_history;
  std::vector<std::pair<ros::Time, cv::Mat> > depth_history;

  Eigen::Matrix3f K_eig;
  cv::Mat Kcv;
  bool has_cam_info;
  tf::TransformBroadcaster br;
  
  double max_tracking_reproj_error;
  int min_tracking_inliers;
  bool show_tracking_debug;
  int max_kps;
  std::string CAMERA_INFO_IN, IMAGE_IN, DEPTH_IN;
};

#endif
