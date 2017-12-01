/**************
 * This is a python binding for SemanticSegmentation
 * Felix Jonathan
 * 11/18/2016
**************/
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "py_input_to_cpp_utils.h"
#include "sp_segmenter/semantic_segmentation.h"
using namespace boost::python;

void printAllObjectTransformInformation(std::vector<ObjectTransformInformation> & pose_estimation)
{
    for (std::vector<ObjectTransformInformation>::const_iterator it = pose_estimation.begin(); 
        it!=pose_estimation.end(); ++it)
    {
        std::cerr << *it;
    }
}

void (SemanticSegmentation::*setCropBoxSize_d)(const double &,const double &,const double &) = &SemanticSegmentation::setCropBoxSize;
void (SemanticSegmentation::*setCropBoxSize_f)(const float &,const float &,const float &) = &SemanticSegmentation::setCropBoxSize;
void (SemanticSegmentation::*setDirectorySVM_1)(const std::string &) =  &SemanticSegmentation::setDirectorySVM;
void (SemanticSegmentation::*setDirectorySVM_2)(const std::string &, const bool &, const bool &) =  &SemanticSegmentation::setDirectorySVM;

BOOST_PYTHON_MODULE(SemanticSegmentationPy)
{
    def("loadPointCloudFromFile",loadPointCloudFromFile);
    def("makeEigenPose",makeEigenPose);
    def("printAllPoses",printAllObjectTransformInformation);

    class_<Eigen::Quaternionf>("EigenQuaternion")
        .def(init<float,float,float,float>())
        .def(init<double,double,double,double>())
    ;

    class_<Eigen::Translation3f>("EigenTranslation")
        .def(init<float,float,float>())
        .def(init<double,double,double>())
    ;

    class_<Eigen::Affine3f>("EigenPose");
    class_<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>("PointCloudXYZRGBA");
    class_<pcl::PointCloud<pcl::PointXYZL>::Ptr>("PointCloudXYZL");

    class_<ModelObjRecRANSACParameter>("ModelObjRecRANSACParameter")
        .def(init<double, double>())
        .def(init<double, double, double, double>())
        .def("setPairWidth", &ModelObjRecRANSACParameter::setPairWidth)
        .def("setVoxelSize", &ModelObjRecRANSACParameter::setVoxelSize)
        .def("setObjectVisibility", &ModelObjRecRANSACParameter::setObjectVisibility)
        .def("setSceneVisibility", &ModelObjRecRANSACParameter::setSceneVisibility)
    ;

    class_<ObjectTransformInformation>("ObjectTransformInformation")
        .def_readwrite("transform_name", &ObjectTransformInformation::transform_name_)
        .def_readwrite("model_name", &ObjectTransformInformation::model_name_)
        .def_readwrite("model_index", &ObjectTransformInformation::model_index_)
        .def_readwrite("model_name", &ObjectTransformInformation::model_name_)
    ;

    class_<std::vector<ObjectTransformInformation> >("vectorOfObjectTransformInformation")
        .def(vector_indexing_suite< std::vector<ObjectTransformInformation>, true>())
    ;

    class_<SemanticSegmentation>("SemanticSegmentation")
        .def("initializeSemanticSegmentation", &SemanticSegmentation::initializeSemanticSegmentation)
        .def("segmentPointCloud", &SemanticSegmentation::segmentPointCloud)
        .def("getTableSurfaceFromPointCloud",&SemanticSegmentation::getTableSurfaceFromPointCloud)
        .def("convertPointCloudLabelToRGBA",&SemanticSegmentation::convertPointCloudLabelToRGBA)
        .def("setDirectorySHOT",&SemanticSegmentation::setDirectorySHOT)
        .def("setDirectoryFPFH",&SemanticSegmentation::setDirectoryFPFH)
        .def("setDirectorySIFT",&SemanticSegmentation::setDirectorySIFT)
        .def("setUseSHOT",&SemanticSegmentation::setUseSHOT)
        .def("setUseFPFH",&SemanticSegmentation::setUseFPFH)
        .def("setUseSIFT",&SemanticSegmentation::setUseSIFT)
        .def("setDirectorySVM",setDirectorySVM_1)
        .def("setDirectorySVM",setDirectorySVM_2)
        .def("setUseMultiClassSVM",&SemanticSegmentation::setUseMultiClassSVM)
        .def("setUseBinarySVM",&SemanticSegmentation::setUseBinarySVM)
        .def("setPointCloudDownsampleValue",&SemanticSegmentation::setPointCloudDownsampleValue<double>)
        .def("setPointCloudDownsampleValue",&SemanticSegmentation::setPointCloudDownsampleValue<float>)
        .def("setHierFeaRatio",&SemanticSegmentation::setHierFeaRatio<double>)
        .def("setHierFeaRatio",&SemanticSegmentation::setHierFeaRatio<float>)

        .def("setUseVisualization", &SemanticSegmentation::setUseVisualization)
        .def("setUseTableSegmentation", &SemanticSegmentation::setUseTableSegmentation)
        .def("setCropAboveTableBoundary", &SemanticSegmentation::setCropAboveTableBoundary<double>)
        .def("setCropAboveTableBoundary", &SemanticSegmentation::setCropAboveTableBoundary<float>)
        .def("loadTableFromFile", &SemanticSegmentation::loadTableFromFile)
        .def("setTableSegmentationParameters", &SemanticSegmentation::setTableSegmentationParameters<double,double,double>)
        .def("setTableSegmentationParameters", &SemanticSegmentation::setTableSegmentationParameters<float,float,float>)
        
        .def("setUseCropBox", &SemanticSegmentation::setUseCropBox)
        .def("setCropBoxSize", setCropBoxSize_d)
        .def("setCropBoxSize", setCropBoxSize_f)
        .def("setCropBoxPose", &SemanticSegmentation::setCropBoxPose<float>)
        .def("setCropBoxPose", &SemanticSegmentation::setCropBoxPose<double>)

#ifdef USE_OBJRECRANSAC
        .def("setUseComputePose", &SemanticSegmentation::setUseComputePose)
        .def("setUseCuda", &SemanticSegmentation::setUseCuda)
        .def("setUseCombinedObjRecRANSAC", &SemanticSegmentation::setUseCombinedObjRecRANSAC)
        .def("setModeObjRecRANSAC", &SemanticSegmentation::setModeObjRecRANSAC)
        .def("setMinConfidenceObjRecRANSAC", &SemanticSegmentation::setMinConfidenceObjRecRANSAC<double>)
        .def("setMinConfidenceObjRecRANSAC", &SemanticSegmentation::setMinConfidenceObjRecRANSAC<float>)
        .def("calculateObjTransform", &SemanticSegmentation::calculateObjTransform)
        .def("segmentAndCalculateObjTransform", &SemanticSegmentation::segmentAndCalculateObjTransform)
        .def("getUpdateOnOneObjTransform", &SemanticSegmentation::getUpdateOnOneObjTransform)  
        .def("addModel", &SemanticSegmentation::addModel)
        // .def("setUsePreferredOrientation", &SemanticSegmentation::setUsePreferredOrientation)
        .def("setPreferredOrientation", &SemanticSegmentation::setPreferredOrientation<float>)
        .def("setPreferredOrientation", &SemanticSegmentation::setPreferredOrientation<double>)
        .def("addModelSymmetricProperty", &SemanticSegmentation::addModelSymmetricProperty<double>)
        .def("addModelSymmetricProperty", &SemanticSegmentation::addModelSymmetricProperty<float>)
        .def("setUseObjectPersistence", &SemanticSegmentation::setUseObjectPersistence)
#endif
    ;
}
