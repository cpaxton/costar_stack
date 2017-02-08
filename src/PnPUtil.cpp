#include "sp_segmenter/PnPUtil.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <iostream>

using namespace cv;

std::vector<Point3f> PnPUtil::BackprojectPts(const std::vector<Point2f>& pts, const Eigen::Matrix4f& camTf, const Eigen::Matrix3f& K, const Mat& depth)
{
  std::vector<Point3f> pts3d;
  for(std::size_t i = 0; i < pts.size(); i++)
  {
    Point2f pt = pts[i];
    Eigen::Vector3f hkp(pt.x, pt.y, 1);
    Eigen::Vector3f backproj = K.inverse()*hkp;
    backproj /= backproj(2);    
    backproj *= depth.at<float>(pt.y, pt.x);
    if(depth.at<float>(pt.y, pt.x) == 0)
      std::cout << "Bad depth (0)" << std::endl;
    Eigen::Vector4f backproj_h(backproj(0), backproj(1), backproj(2), 1);
    backproj_h = camTf*backproj_h;
    pts3d.push_back(Point3f(backproj_h(0), backproj_h(1), backproj_h(2)));
  }
  return pts3d;
}

bool PnPUtil::RansacPnP(const std::vector<Point3f>& matchPts3d, const std::vector<Point2f>& matchPts, 
  Mat Kcv, Eigen::Matrix4f tfguess, Eigen::Matrix4f& tf, std::vector<int>& bestInliersIdx,
  double* avgReprojError)
{
  bestInliersIdx.clear();
  Mat distcoeffcvPnp = (Mat_<double>(4,1) << 0, 0, 0, 0);
  tf = Eigen::MatrixXf::Identity(4,4);
  Mat Rvec, t;
  Mat Rguess = (Mat_<double>(3,3) << tfguess(0,0), tfguess(0,1), tfguess(0,2),
                                     tfguess(1,0), tfguess(1,1), tfguess(1,2),
                                     tfguess(2,0), tfguess(2,1), tfguess(2,2));
  Rodrigues(Rguess, Rvec);
  t = (Mat_<double>(3,1) << tfguess(0,3), tfguess(1,3), tfguess(2,3));
  
  // RANSAC PnP
  const int niter = 50; // Assumes about 45% outliers
  const double reprojThresh = 2.0; // in pixels
  const int m = 4; // points per sample
  const int inlier_ratio_cutoff = 0.4; 
  std::vector<int> ind;
  for(std::size_t i = 0; i < matchPts.size(); i++)
  {
    ind.push_back(i);
  }

  bool abort = false;
  //ros::Time start = ros::Time::now();
  //#pragma omp parallel for
  for(int i = 0; i < niter; i++)
  {
    //#pragma omp flush (abort)
    //if(abort)
    //{
    //  continue;
    //}

    Eigen::Matrix4f rand_tf;
    // Get m random points
    std::random_shuffle(ind.begin(), ind.end());
    std::vector<int> randInd(ind.begin(), ind.begin()+m);

    std::vector<Point3f> rand_matchPts3d;
    std::vector<Point2f> rand_matchPts;
    for(int j = 0; j < m; j++)
    {
      rand_matchPts3d.push_back(matchPts3d[randInd[j]]);
      rand_matchPts.push_back(matchPts[randInd[j]]);
    }

    Mat ran_Rvec, ran_t;
    Rvec.copyTo(ran_Rvec);
    t.copyTo(ran_t);

    solvePnP(rand_matchPts3d, rand_matchPts, Kcv, distcoeffcvPnp, ran_Rvec, ran_t, true, CV_P3P);

    // Test for inliers
    std::vector<Point2f> reprojPts;
    projectPoints(matchPts3d, ran_Rvec, ran_t, Kcv, distcoeffcvPnp, reprojPts);
    std::vector<int> inliersIdx;
    for(std::size_t j = 0; j < reprojPts.size(); j++)
    {
      double reprojError = sqrt((reprojPts[j].x-matchPts[j].x)*(reprojPts[j].x-matchPts[j].x) + (reprojPts[j].y-matchPts[j].y)*(reprojPts[j].y-matchPts[j].y));

      if(reprojError < reprojThresh)
      {
        inliersIdx.push_back(j);
      }
    }

    //#pragma omp critical
    {
      if(inliersIdx.size() > bestInliersIdx.size())
      {
        bestInliersIdx = inliersIdx;
      } 
      if(bestInliersIdx.size() > inlier_ratio_cutoff*matchPts.size())
      {
        //std::cout << "Pnp abort n=" << i << std::endl;
        //abort = true;  
        //#pragma omp flush (abort)
      }
    }
    
  } 
  //std::cout << "Num inliers: " << bestInliersIdx.size() << "/" << matchPts.size() << std::endl;
  //if(bestInliersIdx.size() < 10)
  //{
    //std::cout << "RansacPnP: Could not find enough inliers (" << bestInliersIdx.size() << ")"
    //  << std::endl;
  //  return false;
  //}  
  //ROS_INFO("PnpRansac: Ransac time: %f", (ros::Time::now()-start).toSec());  

  std::vector<Point3f> inlierPts3d;
  std::vector<Point2f> inlierPts2d;
  for(std::size_t i = 0; i < bestInliersIdx.size(); i++)
  {
    inlierPts3d.push_back(matchPts3d[bestInliersIdx[i]]);
    inlierPts2d.push_back(matchPts[bestInliersIdx[i]]);
  }

  //start = ros::Time::now();
  solvePnP(inlierPts3d, inlierPts2d, Kcv, distcoeffcvPnp, Rvec, t, true);
  //ROS_INFO("PnpRansac: Pnp Inliers time: %f", (ros::Time::now()-start).toSec());  
  if(avgReprojError)
  {
    *avgReprojError = 0;
    std::vector<Point2f> reprojPts;
    Mat J;
    projectPoints(inlierPts3d, Rvec, t, Kcv, distcoeffcvPnp, reprojPts, J);
    for(std::size_t j = 0; j < reprojPts.size(); j++)
    {
      double reprojError = sqrt((reprojPts[j].x-inlierPts2d[j].x)*(reprojPts[j].x-inlierPts2d[j].x) + (reprojPts[j].y-inlierPts2d[j].y)*(reprojPts[j].y-inlierPts2d[j].y));
      *avgReprojError += reprojError/reprojPts.size();
    }
  }
    
  Mat R;
  Rodrigues(Rvec, R);

  tf << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2),
             0,      0,      0,    1;


  return true;

}
