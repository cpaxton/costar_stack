#include "sp_segmenter/tracker.h"
#include "sp_segmenter/PnPUtil.h"

#include <cv_bridge/cv_bridge.h>

using namespace cv;

Tracker::Tracker(): nh("~"), has_cam_info(false)
{
  nh.param("CAMERA_INFO_IN", CAMERA_INFO_IN,std::string("/camera/camera_info"));
  nh.param("IMAGE_IN", IMAGE_IN,std::string("/camera/image_raw"));
  nh.param("DEPTH_IN", DEPTH_IN,std::string("/camera/depth"));
  nh.param("min_tracking_inliers", min_tracking_inliers, 8);
  nh.param("max_tracking_reproj_error", max_tracking_reproj_error, 3.0);
  nh.param("show_tracking_debug", show_tracking_debug, false);

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

bool Tracker::addTracker(const ModelT& model)
{
  TrackingInfo ti(model);

  std::pair<TrackingMap::iterator, bool> res = trackers.insert(std::make_pair(model.model_label, ti));
  return res.second;
}

// poses should be pose of object in frame of camera
void Tracker::generateTrackingPoints(ros::Time stamp,  const std::vector<poseT>& poses)
{
  if(!has_cam_info)
    return;
  //ROS_INFO("Generating tracking points");
  // Find closest image to stamp
  //unsigned int depth_match_idx;
  unsigned int image_match_idx;
  unsigned int final_image_idx;
  //Mat depth_match;
  std::vector<Mat> ff_imgs;
  {
    boost::mutex::scoped_lock lock(history_mutex);
    /*
    for(depth_match_idx = 0; depth_match_idx < depth_history.size(); depth_match_idx++)
    {
      if(depth_history[depth_match_idx].first >= stamp)
        break;
    }
    */
    for(image_match_idx = 0; image_match_idx < image_history.size(); image_match_idx++)
    {
      if(image_history[image_match_idx].first >= stamp)
        break;
    }
    // Check if closest time less than stamp is closer
    if(image_match_idx > 0 &&
      abs(image_history[image_match_idx].first.toSec()-stamp.toSec()) > abs(image_history[image_match_idx-1].first.toSec()-stamp.toSec()))
    {
      image_match_idx = image_match_idx-1;
    }
    /*
    if(depth_match_idx > 0 &&
      abs(depth_history[depth_match_idx].first.toSec()-stamp.toSec()) > abs(depth_history[depth_match_idx-1].first.toSec()-stamp.toSec()))
    {
      depth_match_idx = depth_match_idx-1;
    }
    */
    if(image_match_idx != image_history.size())// || depth_match_idx != depth_history.size())
    {
      std::cout << "time diff=" << image_history[image_match_idx].first.toSec()-stamp.toSec() 
        << std::endl;
    }
    if(image_match_idx == image_history.size()// || depth_match_idx == depth_history.size()
      || (image_history[image_match_idx].first-stamp).toSec() > 0.5)
      //|| (depth_history[depth_match_idx].first-stamp).toSec() > 0.5)
    {
      ROS_WARN("Tracker: Could not find image or depth matching pose stamp");
      return;
    }
    //depth_match = depth_history[depth_match_idx].second;

    // Find last image to fastforward to
    final_image_idx = image_history.size()-1;
    for(unsigned int imidx = image_match_idx; imidx <= final_image_idx; imidx++)
    {
      ff_imgs.push_back(image_history.at(imidx).second);
    }
    /*
    ros::Time max_track_time;
    {
      boost::mutex::scoped_lock lock(track_time_mutex); 
      max_track_time = max_element(trackers.begin(), trackers.end(),
        [](const std::pair<std::string, TrackingInfo>& p1,
           const std::pair<std::string, TrackingInfo>& p2) {
          return p1.second.last_track_time.toSec() < p2.second.last_track_time.toSec(); })
          ->second.last_track_time; 
     
    }
    std::cout << "ff window length = " 
      << (max_track_time -image_history.at(image_match_idx).first).toSec() << std::endl;
    for(final_image_idx = image_match_idx; final_image_idx < image_history.size(); final_image_idx++)
    {
      if(image_history.at(final_image_idx).first < max_track_time)
      {
        ff_imgs.push_back(image_history.at(final_image_idx).second);
      }
      else
      {
        break;
      }
    }
    */
  }
  
  // Generate points for each pose
  for(int i = 0; i < poses.size(); i++)
  {
    auto search = trackers.find(poses.at(i).model_name);
    if(search == trackers.end())
    {
      ROS_WARN("Tracker not present for model \"%s\"", poses.at(i).model_name.c_str()); 
      continue;
    }

    KLTTracker& klt_tracker = search->second.klt_tracker;

    if(klt_tracker.getNumPointsTracked() >= 100)
      continue;

    ROS_INFO("Generating new tracking points for model \"%s\"", poses.at(i).model_name.c_str());
       
    pcl::PolygonMesh::Ptr pmesh = search->second.mesh.model_mesh;
    // Project mesh triangles to get mask
    Eigen::Matrix4f pose_trfm;
    pose_trfm.setIdentity();
    pose_trfm.topLeftCorner<3,3>() = poses.at(i).rotation.toRotationMatrix();
    pose_trfm.block<3,1>(0,3) = poses.at(i).shift;

    Mat model_depth;
    Mat mask = meshPoseToMask(pmesh, pose_trfm, model_depth);
    if(show_tracking_debug)
    {
      namedWindow(std::string("Object Mask ") + poses.at(i).model_name, WINDOW_NORMAL);
      imshow(std::string("Object Mask ") + poses.at(i).model_name, mask);

      double min, max;
      cv::minMaxIdx(model_depth, &min, &max);
      cv::Mat scaled_model_depth;
      cv::convertScaleAbs(model_depth, scaled_model_depth, 255 / max);

      namedWindow(std::string("Object Depth ") + poses.at(i).model_name, WINDOW_NORMAL);
      imshow(std::string("Object Depth ") + poses.at(i).model_name, scaled_model_depth);
      waitKey(1);
    }

    // Extract KPs from mesh silhouette  
    // Backproject to 3D
    // Fast-forward tracking of new 2D points to current frame
    std::vector<Mat> ff_imgs_i = ff_imgs;
    if(klt_tracker.hasTracking())
    {
      ff_imgs_i.push_back(klt_tracker.getLastImage());
    }
    {
      boost::mutex::scoped_lock lock(klt_mutex); 
      //klt_tracker.initPointsAndFastforward(ff_imgs_i, depth_match, K_eig,
      //  pose_trfm.inverse(), mask);
      if(!klt_tracker.hasTracking())
        search->second.current_pose = pose_trfm;
      klt_tracker.initPointsAndFastforward(ff_imgs_i, model_depth, K_eig,
        pose_trfm.inverse(), mask);
      // TODO: fastforward pose estimate too
    }
  } 
  {
    boost::mutex::scoped_lock lock(history_mutex);
    // Purge past frames up until most recently used one
    image_history.erase(image_history.begin(), image_history.begin()+final_image_idx);
    // TODO: this assumes image and depth are synced...should search for depth idx instead
    //unsigned int final_depth_idx = std::min(final_image_idx, uint(depth_history.size()-1));
    //depth_history.erase(depth_history.begin(), depth_history.begin()+final_depth_idx);
  }
} 

Mat Tracker::meshPoseToMask(pcl::PolygonMesh::Ptr pmesh, const Eigen::Matrix4f& pose_trfm,
  Mat& depth_out)
{
  Mat mask(cam_info.height, cam_info.width, CV_8UC1, cv::Scalar(0));
  Mat depth(cam_info.height, cam_info.width, CV_32FC1, cv::Scalar(0));
  pcl::PointCloud<myPointXYZ> cloud;
  pcl::fromPCLPointCloud2(pmesh->cloud, cloud);
  for(pcl::Vertices& verts : pmesh->polygons)
  {
    // Project triangle verts
    Eigen::Vector3f p_projs[3];
    double p_depths[3];
    for(int k = 0; k < 3; k++)
    {  
      myPointXYZ p = cloud.points.at(verts.vertices.at(k));  
      Eigen::Vector3f pe(p.x, p.y, p.z);
      Eigen::Vector3f p_proj = K_eig*(pose_trfm*pe.homogeneous()).head<3>();
      p_depths[k] = p_proj(2);
      p_proj /= p_proj(2);
      p_projs[k] = p_proj;
    }
    // Fill triangle
    Eigen::Vector2f d1 = (p_projs[1] - p_projs[0]).head<2>();
    Eigen::Vector2f d2 = (p_projs[2] - p_projs[0]).head<2>();
    double dep_slope1 = p_depths[1] - p_depths[0];
    double dep_slope2 = p_depths[2] - p_depths[0];
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
            float p_depth = p_depths[0] + d1_step*dep_slope1 + d2_step*dep_slope2;
            
            if(depth.at<float>(int(tri_pt(1)), int(tri_pt(0))) == 0)
              depth.at<float>(int(tri_pt(1)), int(tri_pt(0))) = p_depth;
            else
              depth.at<float>(int(tri_pt(1)), int(tri_pt(0))) =
                min(depth.at<float>(int(tri_pt(1)), int(tri_pt(0))), p_depth);
          }
        }
      }
    }
  }
  medianBlur(depth, depth_out, 3);
  medianBlur(mask, mask, 3);
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

  image_sub = nh.subscribe<sensor_msgs::Image>(IMAGE_IN, 1,
    &Tracker::imageCallback,
    this, ros::TransportHints().tcpNoDelay());
  //depth_image_sub = nh.subscribe<sensor_msgs::Image>(DEPTH_IN, 1,
  //  &Tracker::depthImageCallback,
  //  this, ros::TransportHints().tcpNoDelay());
  has_cam_info = true;
}

void Tracker::depthImageCallback(const sensor_msgs::ImageConstPtr &dep)
{
  cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvCopy(dep);
  Mat image = cvImg->image;
  {
    boost::mutex::scoped_lock lock(history_mutex);
    depth_history.push_back(std::pair<ros::Time, cv::Mat> (dep->header.stamp, image));
  }
}

void Tracker::imageCallback(const sensor_msgs::ImageConstPtr &im)
{
  cv_bridge::CvImageConstPtr cvImg = cv_bridge::toCvCopy(im);
  Mat image = cvImg->image;
  // Loop on vector of KLTTrackers, one for each mesh
  for(TrackingMap::iterator titr = trackers.begin(); titr != trackers.end(); titr++)
  //for(int i = 0; i < klt_trackers.size(); i++)
  {
    // Process frame if tracking is valid
    bool has_tracking;
    {
      boost::mutex::scoped_lock lock(klt_mutex);
      has_tracking = titr->second.klt_tracker.hasTracking();
    }
    if(has_tracking)
    {
      //ROS_INFO("Tracking model \"%s\"", titr->first.c_str());
      Mat tracking_viz;
      std::vector<Point2f> pts2d;
      std::vector<Point3f> pts3d;
      std::vector<int> ptIds;
      {
        boost::mutex::scoped_lock lock(klt_mutex);
        //ROS_INFO("Processing frame...");
        titr->second.klt_tracker.processFrame(image, tracking_viz, pts2d, pts3d, ptIds);
      }
      //ROS_INFO("Processed frame");
      {
        boost::mutex::scoped_lock lock(track_time_mutex); 
        titr->second.last_track_time = im->header.stamp;
      }
      if(pts2d.size() == 0)
        continue;

      if(show_tracking_debug)
      {
        namedWindow(std::string("Tracking ") + titr->first, WINDOW_NORMAL);
        imshow(std::string("Tracking ") + titr->first, tracking_viz);
      }
      // SolvePnP
      Eigen::Matrix4f tfran;
      double pnpReprojError;
      std::vector<int> inlierIdx;
      PnPUtil::RansacPnP(pts3d, pts2d, Kcv, titr->second.current_pose, tfran, inlierIdx,
        &pnpReprojError);
      if(inlierIdx.size() > min_tracking_inliers && pnpReprojError < max_tracking_reproj_error)
      {
        titr->second.current_pose = tfran;

        // Publish pose
        publishTf(tfran, titr->first + std::string("_tracking"), im->header.frame_id, im->header.stamp);
        if(show_tracking_debug)
        {
          cv::Mat tf_viz;
          createTfViz(image, tf_viz, tfran, K_eig);
          namedWindow(std::string("Object Transform ") + titr->first, WINDOW_NORMAL);
          imshow(std::string("Object Transform ") + titr->first, tf_viz);
        }
      }
      else
      {
        std::cout << "Tracker: Bad tracking: #inliers=" << inlierIdx.size() << " reproj error="
          << pnpReprojError
          << std::endl;
      }
    }
  }      
  // Store frame
  {
    boost::mutex::scoped_lock lock(history_mutex);
    image_history.push_back(std::pair<ros::Time, cv::Mat> (im->header.stamp, image));
  }
  if(show_tracking_debug)
    waitKey(1); 
}

void Tracker::publishTf(const Eigen::Matrix4f& tf, std::string name, std::string base_frame, 
  ros::Time stamp)
{
  tf::Transform tf_transform;
  tf_transform.setOrigin(tf::Vector3(tf(0,3), tf(1,3), tf(2,3)));
  tf_transform.setBasis(tf::Matrix3x3(tf(0,0), tf(0,1), tf(0,2),
                                      tf(1,0), tf(1,1), tf(1,2),
                                      tf(2,0), tf(2,1), tf(2,2)));
  br.sendTransform(tf::StampedTransform(tf_transform, stamp, base_frame, name));
} 

void Tracker::createTfViz(Mat& src, Mat& dst, const Eigen::Matrix4f& tf,
  const Eigen::Matrix3f& K)
{
  //cvtColor(src, dst, CV_GRAY2RGB);
  src.copyTo(dst);
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
