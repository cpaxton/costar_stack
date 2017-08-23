/* 
 * File:   seg.h
 * Author: chi
 *
 * Created on March 30, 2015, 4:56 PM
 */

#include "sp_segmenter/seg.h"
#include "sp_segmenter/utility/utility.h"

#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <pcl/features/normal_3d.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>


#if SCENE_PARSING
#include <vector>
#include <map>
struct GreedyHypothesis
{
    std::map< std::size_t, std::vector<AcceptedHypothesisWithConfidence> > by_object_hypothesis;
    std::string model_id;
    // std::vector<std::vector<AcceptedHypothesis> > hypothesis;
};
#endif

class greedyObjRansac{
public:
    greedyObjRansac(double pairWidth = 0.1, double voxelSize = 0.03, double relNumOfPairsInHashTable_ = 0.5);
    ~greedyObjRansac(){}
    
    /// Set parameters for object detection
    /// 
    ///
    /// @param vis_ What visible proportion of an object is acceptable for a detection (0.0-1.0)
    /// @param rel_ relative size of the object with respect to the scene.
    ///             aka what portion of the object is expected to be visible in the scene. 
    ///             Each time we see an object we only see a partial match, 
    ///             and the partial match portion (0.0-1.0). Note that "scene" consists
    ///             of actual points sent to this object, not necessarily the full
    ///             frame of a sensor.
    void setParams(double vis_, double rel_);
    
    void ICP(std::vector<poseT> &poses, const pcl::PointCloud<myPointXYZ>::Ptr scene);
    
    void AddModel(std::string name, std::string label);
    void visualize(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, uchar color[3]);
    void visualize_m(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses, std::map<std::string, std::size_t> &model_name_map, uchar model_color[11][3]);
    void clearMesh(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses);
    
    void GreedyRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses);
    void StandardRecognize(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses, double minConfidence = 0);
    void StandardBest(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::vector<poseT> &poses);
    
    void genHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::list<AcceptedHypothesis> &acc_hypotheses);
    void mergeHypotheses(const pcl::PointCloud<myPointXYZ>::Ptr scene_xyz, std::list<AcceptedHypothesis> &acc_hypotheses, std::vector<poseT> &poses);
    pcl::PointCloud<myPointXYZ>::Ptr FillModelCloud(const std::vector<poseT> &poses);
    void setUseCUDA(bool useCUDA){objrec.setUseCUDA(useCUDA);}

#if SCENE_PARSING
    GreedyHypothesis getLatestAcceptedHypothesis(const bool &combined_ransac = false);
#endif

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
    

