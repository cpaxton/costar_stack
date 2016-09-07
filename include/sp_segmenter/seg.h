/* 
 * File:   seg.h
 * Author: chi
 *
 * Created on March 30, 2015, 4:56 PM
 */
#ifndef SEG_H
#define SEG_H

#include "sp_segmenter/utility/utility.h"

//#include <BasicTools/Vtk/VtkTransform.h>
//#include <BasicTools/ComputationalGeometry/Algorithms/RANSACPlaneDetector.h>
#include <ObjRecRANSAC/ObjRecRANSAC.h>
#include <VtkBasics/VtkWindow.h>
#include <VtkBasics/VtkPolyData.h>
#include <VtkBasics/VtkPoints.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkTransformPolyDataFilter.h>
#include <list>
#include <vtkNew.h>

#ifndef FOCAL_X
#define FOCAL_X 539.4611119133837
#define FOCAL_Y 540.5136158944412
#define CENTER_X 313.183195464011
#define CENTER_Y 232.1443718048976
#endif

static bool mycomp(const PointT &p1, const PointT &p2)
{
    return p1.z <= p2.z;
}

struct segT{
    pcl::PointCloud<PointT>::Ptr cloud;
    std::vector<int> indices;   //index in original cloud
};   
       
static uchar model_color[11][3] = 
{ {0, 0, 0}, 
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
  {255, 128, 0},
  {255, 0, 128},
  {0, 128, 255},
  {128, 0, 255},
}; 

vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
//vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(const pcl::PointCloud<PointT>::Ptr cloud);

//void splitCloud(pcl::PointCloud<PointT>::Ptr cloud, std::vector< pcl::PointCloud<myPointXYZ>::Ptr > &cloud_set);
void splitCloud(pcl::PointCloud<PointLT>::Ptr cloud, std::vector< pcl::PointCloud<myPointXYZ>::Ptr > &cloud_set);
void splitCloud(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<myPointXYZ>::Ptr link_cloud, pcl::PointCloud<myPointXYZ>::Ptr node_cloud);

float sqrDistPt(const myPointXYZ &pt1, const myPointXYZ &pt2);
float sqrDistPtT(const PointT &pt1, const PointT &pt2);
pcl::PointCloud<myPointXYZ>::Ptr FilterCloud(const pcl::PointCloud<myPointXYZ>::Ptr scene, const pcl::PointCloud<myPointXYZ>::Ptr tran_model, float T = 0.015);

ModelT LoadMesh(std::string filename, std::string label);
pcl::PointCloud<PointT>::Ptr cropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, float elev);
  
void setObjID(std::map<std::string, int> &model_name_map);

#endif
