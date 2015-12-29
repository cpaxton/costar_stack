/* 
 * File:   features.h
 * Author: chi
 *
 * Created on July 25, 2014, 2:36 PM
 */
#pragma once

#include "../include/index.h"
#include "features.h"

class extCSHOT{
public:
    extCSHOT(float rad = 0.02);
    ~extCSHOT();
    
    std::vector<cv::Mat> getHierFea(MulInfoT &data, int layer);
    
    // For the usage of Dictionary Learning, SubSampling features
    std::vector<cv::Mat> getRawFea(MulInfoT &data, int layer, size_t max_num);
    
    void setRatio(float rr_) {ratio=rr_;}
    std::vector<int> LoadDict_L0(std::string path, std::string colorK, std::string depthK, std::string jointK="");
//    std::vector<int> LoadDict_L1(std::string dict_path, std::vector<std::string> dictK);
//    std::vector<int> LoadDict_L2(std::string dict_path, std::vector<std::string> dictK);
    
    std::vector<int> sampleRaw_L0(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normal,
                    cv::Mat &depth_fea, cv::Mat &color_fea, int max_num, float rad = 0.03);
    std::vector<cv::Mat> EncodeLayer_L0(const cv::Mat &depth_fea, const cv::Mat &color_fea);
    
private:
    void computeRaw_L0(MulInfoT &data, cv::Mat &depth_fea, cv::Mat &color_fea, float rad = 0.03);
    
//    std::vector<cv::Mat> PoolLayer_L1(const MulInfoT &data, const std::vector<cv::Mat> code_L0, std::vector<size_t> idxs, bool max_pool=true);
//    std::vector<cv::Mat> EncodeLayer_L1(const std::vector<cv::Mat> rawfea_L1);    
//    std::vector<cv::Mat> PoolLayer_L2(const MulInfoT &data, const std::vector<cv::Mat> code_L1, std::vector<size_t> idxs, bool max_pool=true);
//    std::vector<cv::Mat> EncodeLayer_L2(const std::vector<cv::Mat> rawfea_L2);
    
    cv::Mat dict_color_L0, dict_depth_L0, dict_joint_L0;
    cv::flann::Index tree_color_L0, tree_depth_L0, tree_joint_L0;
    
//    cv::Mat dict_colorInLAB_L1, dict_colorInXYZ_L1, dict_depthInLAB_L1, dict_depthInXYZ_L1;
//    cv::flann::Index tree_colorInLAB_L1, tree_colorInXYZ_L1, tree_depthInLAB_L1, tree_depthInXYZ_L1;
//    cv::Mat dict_colorInLAB_L2, dict_colorInXYZ_L2, dict_depthInLAB_L2, dict_depthInXYZ_L2;
//    cv::flann::Index tree_colorInLAB_L2, tree_colorInXYZ_L2, tree_depthInLAB_L2, tree_depthInXYZ_L2;
    
    //0: color-lab
    //1: depth-lab
    //2: color-xyz
    //3: depth-xyz
//    bool pool_flag[4];          
//    int pool_type_num;          //4
    float pool_radius_L0;       // 0.03
//    float pool_radius_L1[2];    // 0.05, 0.05
//    float pool_radius_L2[2];    // 0.05, 0.05
    
    float ratio;                //0.15
};

struct PoolingPair{
    std::vector<cv::Mat> pool_feas;
    cv::Mat responses;
};

struct OnePooling{
    cv::Mat pool_seed;
    cv::flann::Index pool_tree;
    
    int pool_len;
    float pool_radius;
    int poolK;
};

class layerPooler{
public:
    layerPooler();
    ~layerPooler(){};
    
    int LoadSeedsPool(std::string pool_seed_file);
    int LoadSeedsPool(cv::Mat pool_seeds_);
    void ConstructLAB(int level, int poolK_);
    void ContructFPFH(cv::Mat pool_seeds, int poolK_);
    void ContructSIFT(cv::Mat pool_seeds, int poolK_);
    void Construct2D(int level, int poolK_);
    
    PoolingPair Pooling(const PoolingPair &fea);
    
    void setMaxPool(bool max_flag){max_pool = max_flag;}
    
    PoolingPair highCoding(const PoolingPair &fea);
    void configureHigh(int K);
    
//private:
    
    std::vector<int> getMultiPoolIdx(const std::vector<cv::Mat> &pool_fea, int idx = 0);
    std::vector<int> calculateIdx(const std::vector< std::vector<int> > &index_set, const std::vector< int > &sum_vec, int level);
//    std::vector<int> getMultiPoolIdx(const cv::Mat &pool_fea, float radius);
    
    std::vector< boost::shared_ptr<OnePooling> > pool_spaces;
    std::vector< std::string > pool_names;
    std::vector< std::vector<int> > pool_group;
    bool max_pool;
    
    int lab_range;
    
    std::vector< std::vector<int> > high_vec;
    int highK;
};


cv::Mat LABSeeds(int level);
cv::Mat spreadFea(const PoolingPair &fea);


