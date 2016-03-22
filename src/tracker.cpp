#include "sp_segmenter/tracker.h"
#include "sp_segmenter/PnPUtil.h"
#include <cv_bridge/cv_bridge.h>

using namespace cv;

Tracker::Tracker(int num_trackers): nh("~"), klt_trackers(num_trackers),
  current_poses(num_trackers, Eigen::Matrix4f::Identity()),
  last_track_times(num_trackers)
{
  nh.param("CAMERA_INFO_IN", CAMERA_INFO_IN,std::string("/camera/camera_info"));
  nh.param("IMAGE_IN", IMAGE_IN,std::string("/camera/image_raw"));
  nh.param("min_tracking_inliers", min_tracking_inliers, 8);
  nh.param("max_tracking_reproj_error", max_tracking_reproj_error, 3.0);

  cam_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(CAMERA_INFO_IN, 1000,
    &Tracker::cameraInfoCallback,
    this, ros::TransportHints().tcpNoDelay());

  nh.setCallbackQueue(&callback_queue);

  callback_thread = boost::thread(&Tracker::monitorQueue, this);
}

void Tracker::monitorQueue()
{
  ros::Time lastTime = ros::Time::now();
  while (ros::ok())
  {
    callback_queue.callAvailable(ros::WallDuration(0.1));
  }
}

void Tracker::generateTrackingPoints(ros::Time stamp, const std::vector<ModelT>& meshs,
  const std::vector<poseT>& poses)
{
  assert(poses.size() == mesh.size());

  // Fine closest depth map
  unsigned int depth_match_idx, image_match_idx;
  for(depth_match_idx = 0; depth_match_idx < depth_history.size(); depth_match_idx++)
  {
    if(depth_history[depth_match_idx].first >= stamp)
      break;
  }
  for(image_match_idx = 0; image_match_idx < image_history.size(); image_match_idx++)
  {
    if(image_history[image_match_idx].first >= stamp)
      break;
  }
  if(image_match_idx == image_history.size() || depth_match_idx == depth_history.size()
    || (image_history[image_match_idx].first-stamp).toSec() > 0.5
    || (depth_history[depth_match_idx].first-stamp).toSec() > 0.5)
  {
    ROS_WARN("Tracker: Could not find image or depth matching pose stamp");
    return;
  }
  // TODO: check if closest time less than stamp is closer

  unsigned int largest_final_image_idx = 0;
  for(int i = 0; i < meshs.size(); i++)
  {
    pcl::PolygonMesh::Ptr pmesh = meshs.at(i).model_mesh;
    // Project mesh triangles to get mask
    Eigen::Matrix4f pose_trfm;
    pose_trfm.setIdentity();
    pose_trfm.topLeftCorner<3,3>() = poses.at(i).rotation.toRotationMatrix();
    pose_trfm.block<3,1>(0,3) = poses.at(i).shift;

    Mat mask = meshPoseToMask(pmesh, pose_trfm);

    // Extract KPs from mesh silhouette  
    // Backproject to 3D
    // Fast-forward tracking of new 2D points to current frame
    std::vector<Mat> ff_imgs;
    unsigned int final_image_idx = image_match_idx;
    for(; final_image_idx < image_history.size(); final_image_idx++)
    {
      if(image_history.at(final_image_idx).first < last_track_times.at(i))
      {
        ff_imgs.push_back(image_history.at(final_image_idx).second);
      }
      else
      {
        break;
      }
    }
    if(klt_trackers.at(i).hasTracking())
    {
      ff_imgs.push_back(klt_trackers.at(i).getLastImage());
    }
    klt_trackers.at(i).initPointsAndFastforward(ff_imgs, depth_history[depth_match_idx].second, K_eig,
      pose_trfm, mask);
    largest_final_image_idx = max(largest_final_image_idx, final_image_idx);
  } 
  // TODO: might need a better way of deleting history
  // Purge past frames up until most recently used one
  image_history.erase(image_history.begin(), image_history.begin()+largest_final_image_idx);
  // TODO: this assumes image and depth are synced...should search for depth idx instead
  depth_history.erase(depth_history.begin(), depth_history.begin()+largest_final_image_idx);
} 

// TODO: Maybe get depth map from this too?
Mat Tracker::meshPoseToMask(pcl::PolygonMesh::Ptr pmesh, const Eigen::Matrix4f& pose_trfm)
{
  Mat mask(cam_info.height, cam_info.width, CV_8UC1, cv::Scalar(0));
  pcl::PointCloud<myPointXYZ> cloud;
  pcl::fromPCLPointCloud2(pmesh->cloud, cloud);
  for(pcl::Vertices& verts : pmesh->polygons)
  {
    // Project triangle verts
    Eigen::Vector3f p_projs[3];
    for(int k = 0; k < 3; k++)
    {  
      myPointXYZ p = cloud.points.at(verts.vertices.at(k));  
      Eigen::Vector3f pe(p.x, p.y, p.z);
      Eigen::Vector3f p_proj = K_eig*(pose_trfm*pe.homogeneous()).head<3>();
      p_proj /= p_proj(2);
      p_projs[k] = p_proj;
    }
    // Fill triangle
    Eigen::Vector2f d1 = (p_projs[1] - p_projs[0]).head<2>();
    Eigen::Vector2f d2 = (p_projs[2] - p_projs[0]).head<2>();
    for(double d1_step = 0; d1_step <= 1; d1_step+=1./d1.norm())
    {
      for(double d2_step = 0; d2_step <= 1; d2_step+=1./d2.norm())
      {
        if(d1_step + d2_step <= 1)
        {
          Eigen::Vector2f tri_pt =  p_projs[0].head<2>() + d1_step*d1 + d2_step*d2;
          if(tri_pt(0) < cam_info.width && tri_pt(0) >= 0 && 
            tri_pt(1) < cam_info.height && tri_pt(1) >= 0)
          {
            mask.at<uchar>(int(tri_pt(1)), int(tri_pt(0))) = 255;
          }
        }
      }
    }
  }
  return mask;
}

void Tracker::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &ci)
{
  ROS_INFO("Received camera info"); 
  cam_info = *ci;
  cam_info_sub.shutdown();

  Kcv = (Mat_<double>(3,3) <<  ci->K[0], ci->K[1], ci->K[2],
                               ci->K[3], ci->K[4], ci->K[5],
                               ci->K[6], ci->K[7], ci->K[8]);
  K_eig << ci->K[0], ci->K[1], ci->K[2],
           ci->K[3], ci->K[4], ci->K[5],
           ci->K[6], ci->K[7], ci->K[8];

  image_sub = nh.subscribe<sensor_msgs::Image>(IMAGE_IN, 10,
    &Tracker::imageCallback,
    this, ros::TransportHints().tcpNoDelay());
  depth_image_sub = nh.subscribe<sensor_msgs::Image>(DEPTH_IN, 10,
    &Tracker::depthImageCallback,
    this, ros::TransportHints().tcpNoDelay());
}

void Tracker::depthImageCallback(const sensor_msgs::ImageConstPtr &dep)
{
  cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvCopy(dep);
  Mat image = cvImg->image;
  depth_history.push_back(std::pair<ros::Time, cv::Mat> (dep->header.stamp, image));
}

void Tracker::imageCallback(const sensor_msgs::ImageConstPtr &im)
{
  cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvCopy(im);
  Mat image = cvImg->image;
  // Loop on vector of KLTTrackers, one for each mesh
  for(int i = 0; i < klt_trackers.size(); i++)
  {
    // Process frame if tracking is valid
    if(klt_trackers[i].hasTracking())
    {
      Mat tracking_viz;
      std::vector<Point2f> pts2d;
      std::vector<Point3f> pts3d;
      std::vector<int> ptIds;
      klt_trackers[i].processFrame(image, tracking_viz, pts2d, pts3d, ptIds);
      last_track_times.at(i) = im->header.stamp;
      // SolvePnP
      Eigen::Matrix4f tfran;
      double pnpReprojError;
      std::vector<int> inlierIdx;
      PnPUtil::RansacPnP(pts3d, pts2d, Kcv, current_poses[i], tfran, inlierIdx,
        &pnpReprojError);
      if(inlierIdx.size() > min_tracking_inliers && pnpReprojError < max_tracking_reproj_error)
      {
        current_poses[i] = tfran;

        cv::Mat tf_viz;
        createTfViz(image, tf_viz, tfran, K_eig);
        namedWindow(std::string("Object Transform ") + std::to_string(i), WINDOW_NORMAL);
        imshow(std::string("Object Transform ") + std::to_string(i), tf_viz);
      }
    }
  }      
  // TODO: add locking everywhere!
  // Store frame
  image_history.push_back(std::pair<ros::Time, cv::Mat> (im->header.stamp, image));
  waitKey(1); 
}

void Tracker::createTfViz(Mat& src, Mat& dst, const Eigen::Matrix4f& tf,
  const Eigen::Matrix3f& K)
{
  cvtColor(src, dst, CV_GRAY2RGB);
  Eigen::Vector3f t = tf.block<3,1>(0,3);
  Eigen::Vector3f xr = tf.block<3,1>(0,0);
  Eigen::Vector3f yr = tf.block<3,1>(0,1);
  Eigen::Vector3f zr = tf.block<3,1>(0,2);
        
  Eigen::Vector3f x = t + xr/6*xr.norm();
  Eigen::Vector3f y = t + yr/6*yr.norm();
  Eigen::Vector3f z = t + zr/6*zr.norm();
          
  Eigen::Vector3f origin = K*t;
  Eigen::Vector3f xp = K*x;
  Eigen::Vector3f yp = K*y;
  Eigen::Vector3f zp = K*z;
  Point o2d(origin(0)/origin(2), origin(1)/origin(2));
  Point x2d(xp(0)/xp(2), xp(1)/xp(2));
  Point y2d(yp(0)/yp(2), yp(1)/yp(2));
  Point z2d(zp(0)/zp(2), zp(1)/zp(2));

  line(dst, o2d, x2d, CV_RGB(255, 0, 0), 3, CV_AA);
  line(dst, o2d, y2d, CV_RGB(0, 255, 0), 3, CV_AA);
  line(dst, o2d, z2d, CV_RGB(0, 0, 255), 3, CV_AA);
}
