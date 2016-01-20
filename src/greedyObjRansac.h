/* 
 * File:   seg.h
 * Author: chi
 *
 * Created on March 30, 2015, 4:56 PM
 */

#include "seg.h"

#include <pcl/features/normal_3d.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>


class greedyObjRansac{
public:
    greedyObjRansac(double pairWidth = 0.1, double voxelSize = 0.03);
    ~greedyObjRansac(){}
    
    void setParams(double vis_, double rel_);
    
    void ICP(std::vector<poseT> &poses, const pcl::PointCloud<myPointXYZ>::Ptr scene);
    
    void AddModel(std::string name, std::string label);
    void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, int color[3]);
    void visualize_m(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, std::map<std::string, int> &model_name_map, uchar model_color[11][3]);
    void clearMesh(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses);
    
    void GreedyRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses);
    void StandardRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses, double confidenceThreshold = 0);
    void StandardBest(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses);
    
    void genHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::list<AcceptedHypothesis> &acc_hypotheses);
    void mergeHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::list<AcceptedHypothesis> &acc_hypotheses, std::vector<poseT> &poses);
    pcl::PointCloud<myPointXYZ>::Ptr FillModelCloud(const std::vector<poseT> &poses);
    
private:
    std::vector<ModelT> models;
    ObjRecRANSAC objrec;
    
    poseT recognizeOne(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, pcl::PointCloud<myPointXYZ>::Ptr &rest_cloud);
    poseT getBestModel(std::list< boost::shared_ptr<PointSetShape> >& detectedShapes);
    
    void getPairFeas(const pcl::PointCloud<myPointXYZ>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, std::list<ObjRecRANSAC::OrientedPair> &PairFeas, float maxDist, int num);
    
    double successProbability;      //0.99
    double voxelSize;               //0.03
    double pairWidth;                       
    
    double visibility;              //0.1   
    double relativeObjSize;         //0.1
};
    

