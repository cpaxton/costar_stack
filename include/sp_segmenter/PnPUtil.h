#ifndef _PNP_UTIL_H_
#define _PNP_UTIL_H_


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

class PnPUtil
{
public:
  static std::vector<cv::Point3f> BackprojectPts(const std::vector<cv::Point2f>& pts, 
    const Eigen::Matrix4f& camTf, const Eigen::Matrix3f& K, const cv::Mat& depth);
  static bool RansacPnP(const std::vector<cv::Point3f>& matchPts3d, 
    const std::vector<cv::Point2f>& matchPts, cv::Mat Kcv, Eigen::Matrix4f tfguess, 
    Eigen::Matrix4f& tf, std::vector<int>& inlierIdx, double* avgReprojError = NULL);
};
#endif
