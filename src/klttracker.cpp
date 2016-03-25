#include "sp_segmenter/klttracker.h"
#include <iostream>
#include <algorithm>

using namespace Eigen;
using namespace cv;

KLTTracker::KLTTracker(unsigned int max_kps)
{
  m_nextID = 0;
  m_maxNumberOfPoints = max_kps;
  m_fastDetector = cv::FastFeatureDetector::create(std::string("FAST"));
}

unsigned int KLTTracker::getNumPointsTracked()
{
  return m_ptIDs.size();
}

bool KLTTracker::hasTracking()
{
 return m_ptIDs.size() > 0 && m_prevPts.size() > 0;
}

cv::Mat KLTTracker::getLastImage()
{
  return m_prevImg;
}

std::vector<unsigned char> KLTTracker::filterMatchesEpipolarContraint(
  const std::vector<cv::Point2f>& pts1, 
  const std::vector<cv::Point2f>& pts2)
{
  std::vector<unsigned char> status; 
  findFundamentalMat(pts1, pts2, CV_FM_RANSAC, 3, .99, status);
  return status;
}

void KLTTracker::initPointsAndFastforward(const std::vector<cv::Mat>& inputFrames, const cv::Mat& depth,
  const Eigen::Matrix3f& K, 
  const Eigen::Matrix4f& inputTf, const cv::Mat& mask)
{
  if(inputFrames.size() == 0)
  {
    std::cout << "KLTTracker: number of input frames = 0" << std::endl;
    return;
  }
  if(m_ptIDs.size() == m_maxNumberOfPoints)
  {
    std::cout << "KLTTracker: Max keypoints reached" << std::endl;
    return;
  }

  std::vector<cv::KeyPoint> detected_pts;
  std::vector<cv::Point2f> new_pts;
  std::vector<cv::Point3f> new_3d_pts;
  // Detect new points
  m_fastDetector->detect(inputFrames.at(0), detected_pts, mask);
  std::sort(detected_pts.begin(), detected_pts.end(),
    [](const cv::KeyPoint &a, const cv::KeyPoint& b) -> bool
    { 
      return a.response > b.response; 
    }
  );
  //std::random_shuffle(detected_pts.begin(), detected_pts.end());
  unsigned int num_new_pts = min(detected_pts.size(), m_maxNumberOfPoints - m_ptIDs.size());

  std::cout << "detected " << num_new_pts << " new pts" << std::endl;
  // Backproject points to 3D using depth data
  // TODO: try with model instead???
  for (size_t i=0; i<num_new_pts; i++)
  {
    Eigen::Vector3f hkp(detected_pts[i].pt.x, detected_pts[i].pt.y, 1);

    // TODO: DOUBLE CHECK DEPTH IS IN METERS
    double pt_depth = depth.at<float>(int(hkp(1)), int(hkp(0)));
    if(pt_depth == 0 || pt_depth == -1 || std::isnan(pt_depth))
      continue;

    Eigen::Vector3f backproj = K.inverse()*hkp;
    backproj /= backproj(2);    
    backproj *= pt_depth;
    Eigen::Vector4f backproj_h(backproj(0), backproj(1), backproj(2), 1);
    backproj_h = inputTf*backproj_h;
    new_3d_pts.push_back(Point3f(backproj_h(0), backproj_h(1), backproj_h(2)));
    new_pts.push_back(detected_pts.at(i).pt);
  }
  std::cout << "backprojected " << new_pts.size() << " pts"  << std::endl;

  // Fastforward tracked points to current frame
  std::vector<cv::Point2f> prev_pts = new_pts;
  std::vector<cv::Point2f> next_pts;
  std::vector<cv::Point3f> valid_3dpts;
  std::vector<unsigned char> status;
  for(unsigned int i = 1; i < inputFrames.size(); i++)
  {
    if(prev_pts.size() == 0)
    {
      std::cout << "KLTTracker: Fastforward failed" << std::endl;
      break;
    }
    processFrameInternal(inputFrames.at(i-1), inputFrames.at(i), prev_pts, next_pts, status);
    prev_pts.clear();
    valid_3dpts.clear();
    for(unsigned int j = 0; j < status.size(); j++)
    {
      if(status[j])
      {
        prev_pts.push_back(next_pts.at(j));
        valid_3dpts.push_back(new_3d_pts.at(j));
      }
    }
    //std::cout << "frame " << i << "/" << inputFrames.size() << " " << next_pts.size() << " pts" 
    //  << std::endl;
    new_3d_pts = valid_3dpts;
  }
  
  assert(new_3dpts.size() == prev_pts.size());
  // Add prev image if initializing
  if(m_prevImg.cols == 0)
    m_prevImg = inputFrames.back();
  // Add new points to tracker
  std::cout <<"adding " << prev_pts.size() << " new pts" << std::endl;
  m_tracked3dPts.insert(m_tracked3dPts.end(), new_3d_pts.begin(), new_3d_pts.end());
  m_prevPts.insert(m_prevPts.end(), prev_pts.begin(), prev_pts.end());
  std::cout <<"pts_size=" << m_prevPts.size() << std::endl;
  for(unsigned int i = 0; i < new_3d_pts.size(); i++)
  {
    m_ptIDs.push_back(m_nextID++);
  }
}


bool KLTTracker::processFrameInternal(const cv::Mat& prev_image, const cv::Mat& next_image,
  const std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& next_pts,
  std::vector<unsigned char>& status)
{
  next_pts.clear();
  status.clear();
  std::vector<float> error;
  if (prev_pts.size() > 0)
  {
    cv::calcOpticalFlowPyrLK(prev_image, next_image, prev_pts, next_pts, status, error);
  }
  else
  { 
    std::cout << "prev_points.size()=0" << std::endl;
    return false;
  }

  std::vector<cv::Point2f> lkPrevPts, lkNextPts;
  std::vector<int> lkIds;
  for (size_t i=0; i<status.size(); i++)
  {
    if (status[i])
    {
      lkPrevPts.push_back(prev_pts[i]);
      lkNextPts.push_back(next_pts[i]);
      lkIds.push_back(i);
    }
  }
  
  std::vector<unsigned char> epStatus;
  std::vector<int> epIds;
  if(lkPrevPts.size() > 0)
    epStatus = filterMatchesEpipolarContraint(lkPrevPts, lkNextPts);
  std::vector<cv::Point2f> trackedPts;

  for (size_t i=0; i<epStatus.size(); i++)
  {
    if (epStatus[i])
    {
      trackedPts.push_back(lkNextPts.at(i));
      epIds.push_back(lkIds.at(i));
    }
  }

  status = std::vector<unsigned char>(next_pts.size(), false);
  for(unsigned int i = 0; i < epIds.size(); i++)
  {
    status.at(epIds.at(i)) = true;
    next_pts.at(epIds.at(i)) = trackedPts.at(i);
  }

  return true;
}

//!// Processes a frame and returns output image
bool KLTTracker::processFrame(const cv::Mat& inputFrame, cv::Mat& outputFrame, 
  std::vector<cv::Point2f>& pts2d, std::vector<cv::Point3f>& pts3d, std::vector<int>& ptIDs)
{
  pts2d.clear();
  pts3d.clear();
  inputFrame.copyTo(m_nextImg);
  inputFrame.copyTo(outputFrame);
  //cv::cvtColor(inputFrame, outputFrame, CV_GRAY2BGR);
  
  std::vector<unsigned char> status;
  //std::cout << "processing internal..." << std::endl;
  //std::cout <<"next dim=" << m_nextImg.rows << "x" << m_nextImg.cols << std::endl;
  //std::cout <<"prev dim=" << m_prevImg.rows << "x" << m_prevImg.cols << std::endl;
  //imshow("next_img", m_nextImg);
  //imshow("prev_img", m_prevImg);
  //waitKey(1);
  processFrameInternal(m_prevImg, m_nextImg, m_prevPts, m_nextPts, status);
  //std::cout << "processed internal..." << std::endl;

  std::vector<cv::Point2f> trackedPts;
  std::vector<cv::Point3f> tracked3dPts;
  std::vector<int> trackedPtIDs;
  for (size_t i=0; i<status.size(); i++)
  {
    if (status[i])
    {
      tracked3dPts.push_back(m_tracked3dPts[i]);
      trackedPts.push_back(m_nextPts[i]);
      trackedPtIDs.push_back(m_ptIDs[i]);
      cv::line(outputFrame, m_prevPts[i], m_nextPts[i], cv::Scalar(0,250,0));
      cv::circle(outputFrame, m_nextPts[i], 3, cv::Scalar(0,250,0), -1);
      cv::putText(outputFrame, std::to_string(m_ptIDs[i]), m_nextPts[i], 
        cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar::all(255));
    }
  }
  pts2d = trackedPts;
  pts3d = tracked3dPts;
  ptIDs = trackedPtIDs;
  m_tracked3dPts = tracked3dPts;
  m_prevPts = trackedPts;
  m_ptIDs = trackedPtIDs;

  m_nextImg.copyTo(m_prevImg);
  return true;
}
