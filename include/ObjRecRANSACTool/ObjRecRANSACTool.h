#ifndef OBJ_REC_RANSAC_TOOL_H
#define OBJ_REC_RANSAC_TOOL_H

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkSmartPointer.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <btBulletDynamicsCommon.h>

#include <ObjRecRANSAC/ObjRecRANSAC.h>

#include "physics_world_parameters.h"

class ObjRecRANSACTool : public ObjRecRANSAC
{
public:
    ObjRecRANSACTool(double pairWidth = 0.1, double voxelSize = 0.004);
    ~ObjRecRANSACTool(){}
    
    void addModelFromPath(const std::string &name, const std::string &label);
    // void clearMesh(pcl::visualization::PCLVisualizer::Ptr viewer, const std::vector<poseT> &poses);

    void setPointCloudData(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    double getConfidence(const std::string &model_name, const btTransform &transform);

private:
    // std::vector<ModelT> models;
    // ObjRecRANSAC objrec;

    double successProbability_;      //0.99
    double voxelSize_;               //0.03
    double pairWidth_;                       
    
    double visibility_;              //0.1   
    double relativeObjSize_;         //0.1

    bool have_scene_points_;
};



#endif