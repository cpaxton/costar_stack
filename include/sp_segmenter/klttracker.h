#ifndef _KLTTracker_hpp
#define _KLTTracker_hpp

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class KLTTracker
{

public:
  KLTTracker(unsigned int max_kps);
  void initPointsAndFastforward(const std::vector<cv::Mat>& inputFrames, const cv::Mat& depth,
    const Eigen::Matrix3f& K, const Eigen::Matrix4f& inputTf, const cv::Mat& mask);
  virtual bool processFrame(const cv::Mat& inputFrame, cv::Mat& outputFrame,
    std::vector<cv::Point2f>& pts2d, std::vector<cv::Point3f>& pts3d, std::vector<int>& ptIDs);
  bool hasTracking();
  unsigned int getNumPointsTracked();
  cv::Mat getLastImage();
  void clear();

private:
  static bool processFrameInternal(const cv::Mat& prev_image, const cv::Mat& next_image,
    const std::vector<cv::Point2f>& prev_pts, std::vector<cv::Point2f>& next_pts,
    std::vector<unsigned char>& status);
  static std::vector<unsigned char> filterMatchesEpipolarContraint(const std::vector<cv::Point2f>& pts1,
    const std::vector<cv::Point2f>& pts2);

  unsigned int m_maxNumberOfPoints;

  cv::Mat m_prevImg;
  cv::Mat m_nextImg;

  std::vector<cv::Point2f> m_prevPts;
  std::vector<cv::Point2f> m_nextPts;
  std::vector<cv::Point3f> m_tracked3dPts;

  std::vector<int> m_ptIDs;
  int m_nextID;

  cv::Ptr<cv::FeatureDetector> m_fastDetector;
};
#endif

