#ifndef _SP_TRACKER_HPP_
#define _SP_TRACKER_HPP_

#include <vector>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <boost/thread.hpp>

#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include "sp_segmenter/klttracker.h"
#include "sp_segmenter/utility/utility.h"

class Tracker
{
  public:
  Tracker(int num_trackers);
  void generateTrackingPoints(ros::Time stamp, const std::vector<ModelT>& meshs,
    const std::vector<poseT>& poses);

  private:
  cv::Mat meshPoseToMask(pcl::PolygonMesh::Ptr pmesh, const Eigen::Matrix4f& pose_trfm);
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
  ros::CallbackQueue callback_queue;

  std::vector<KLTTracker> klt_trackers;
  std::vector<Eigen::Matrix4f> current_poses;
  std::vector<ros::Time> last_track_times;
  std::vector<std::pair<ros::Time, cv::Mat> > image_history;
  std::vector<std::pair<ros::Time, cv::Mat> > depth_history;

  Eigen::Matrix3f K_eig;
  cv::Mat Kcv;
  
  double max_tracking_reproj_error;
  int min_tracking_inliers;
  std::string CAMERA_INFO_IN, IMAGE_IN, DEPTH_IN;
};

#endif
