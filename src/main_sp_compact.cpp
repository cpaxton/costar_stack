/**************
 * This is an example program for extracting features from point cloud and 
 * training SVM.
 * The point cloud data should be in organized format. It is also the default format
 * you get from openni2_launch.
 * For object training data, you should get rid of the background data by setting their 
 * (x,y,z) to (inf, inf, inf) while keeping their rgba values in the organized format.
 * This is used for computing complete sift feature.
 * When you specify a data path, the program will read all .pcd files as training data.
 * Since background data is the whole scene and object data is just a segment, please tune the 
 * sampling supervoxel numbers according to different sources.
 * 
 * Chi Li
 * 12/16/2015
**************/

#include <iostream>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

// ros header for roslaunch capability
#ifdef BUILD_ROS_BINDING
#include <ros/ros.h>
#endif

#include "sp_segmenter/stringVectorArgsReader.h"
#include "sp_segmenter/sp_compact.h"

int main(int argc, char** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
// -------------------------------------------------------------------------
    std::string root_path;
    double foregroundBackgroundCC;
    double multiclassCC;
    //NUM_OBJECT_TRAINING_DATA*OBJ_SAMPLES approximately = NUM_BACKGROUND_TRAINING_DATA*NUM_BG_SAMPLES
    int bgSampleNum, objSampleNum;

    std::string shot_path, sift_path, fpfh_path;
    std::vector<std::string> obj_names, bg_names;
    std::string out_fea_path, out_svm_path;

    bool train_bg_flag;
    bool train_multi_flag;
    bool skip_fea;

#ifdef BUILD_ROS_BINDING
// Getting the parameter from ros param.
    ros::init(argc,argv,"sp_compact_node");
    ros::NodeHandle nh("~");

    nh.param("root_path", root_path, std::string("data/training"));
    nh.param("foreground_cc", foregroundBackgroundCC, 0.001);
    nh.param("multiclass_cc", multiclassCC, 0.001);
    nh.param("bg_sample_num", bgSampleNum, 66);
    nh.param("obj_sample_num", objSampleNum, 100);
    
    obj_names = stringVectorArgsReader (nh, "obj_names", std::string("drill"));
    bg_names = stringVectorArgsReader (nh, "bg_names", std::string("UR5"));

    // paths for dictionaries
    nh.param("shot_path",shot_path,std::string("data/UW_shot_dict"));
    nh.param("sift_path",sift_path,std::string("data/UW_sift_dict"));
    nh.param("fpfh_path",fpfh_path,std::string("data/UW_fpfh_dict"));

    nh.param("out_fea_path",out_fea_path,std::string("fea_pool"));
    nh.param("out_svm_path",out_svm_path,std::string("svm_pool"));

    nh.param("skip_fea",skip_fea,false);
    nh.param("train_bg_flag",train_bg_flag, true);
    nh.param("train_multi_flag",train_multi_flag, true);
#else
// Set the parameter manually,
    root_path = "data/training";
    foregroundBackgroundCC =  0.001;
    multiclassCC = 0.001;
    bgSampleNum = 66;
    objSampleNum = 100;
    obj_names = stringVectorArgsReader ("drill");
    bg_names = stringVectorArgsReader ("UR5");

    // paths for dictionaries
    shot_path = "data/UW_shot_dict";
    sift_path = "data/UW_sift_dict";
    fpfh_path = "data/UW_fpfh_dict";

    out_fea_path = "fea_pool";
    out_svm_path = "svm_pool";

    skip_fea = false;
    train_bg_flag  = true;
    train_multi_flag = true;
#endif
// ---------------------------------------------------------------------
// Setting up the training parameters
    SpCompact training;
    training.setInputPathSIFT(sift_path);
    training.setInputPathSHOT(shot_path);
    training.setInputPathFPFH(fpfh_path);

    // Setting up training directory locations. Return false if directory is not exist
    training.setInputTrainingPath(root_path);

    // The class will load the pcd files from directory_path/object_name[1 .. n]/*.pcd
    // and directory_path/background_name[1 .. n]/*.pcd
    training.setObjectNames(obj_names);
    training.setBackgroundNames(bg_names);

    // Set file output directory. Will create the output directory if it does not exist
    training.setOutputDirectoryPathFEA(out_fea_path);
    training.setOutputDirectoryPathSVM(out_svm_path);
    
    training.setForegroundCC(foregroundBackgroundCC);
    training.setMultiCC(multiclassCC);

    training.setBackgroundSampleNumber(bgSampleNum);
    training.setObjectSampleNumber(objSampleNum);

    training.setCurOrderMax(3);

    training.setSkipFeaExtraction(skip_fea);
    training.setSkipBackgroundSVM(!train_bg_flag);
    training.setSkipMultiSVM(!train_multi_flag);

    training.startTrainingSVM();

#ifdef BUILD_ROS_BINDING
    ros::shutdown();
#endif

    return 0;
} 
