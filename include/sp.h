/* 
 * File:   features.h
 * Author: chi
 *
 * Created on July 25, 2014, 2:36 PM
 */

#pragma once

#include "features.h"
#include "HierFea.h"

class spExt{
public:
    spExt(float ss_=0.005);
    ~spExt();
    
    void LoadPointCloud(const pcl::PointCloud<PointT>::Ptr cloud);
    pcl::PointCloud<PointT>::Ptr getCloud();
    pcl::PointCloud<PointLT>::Ptr getLabels();
    IDXSET getSegsToCloud();
    
    IDXSET getSPIdx(int level);
    std::vector<pcl::PointCloud<PointT>::Ptr> getSPCloud(int level);
//    void updateLevel(int level, const IDXSET &in_idx);
    void setSPFlags(const std::vector<int> &sp_flags_, bool constrained_flag);
    void clear();
    
    void setSS(float ss_){down_ss = ss_;}
    void setParams(float voxel_resol, float seed_resol, float color_w, float spatial_w, float normal_w);
private:
    pcl::PointCloud<PointT>::Ptr down_cloud;
    pcl::PointCloud<PointLT>::Ptr label_cloud;
    
    std::vector<pcl::PointCloud<PointT>::Ptr> low_segs;
    std::vector<int> sp_flags;
    IDXSET segs_to_cloud;
    std::vector<IDXSET> sp_level_idx;
    
    std::multimap<uint32_t, uint32_t> graph;
    
    int low_seg_num;
    float down_ss;
    
    float voxel_resol;
    float seed_resol;
    float color_w;
    float spatial_w;
    float normal_w;
    
    void buildOneSPLevel(int level);
};

class spPooler{
public:
    spPooler();
    ~spPooler();
    
    void build_SP_LAB(const std::vector< boost::shared_ptr<Pooler_L0> > &lab_pooler_set, bool max_pool_flag = false);
    void build_SP_FPFH(const std::vector< boost::shared_ptr<Pooler_L0> > &fpfh_pooler_set, float radius, bool max_pool_flag = false);
    void build_SP_SIFT(const std::vector< boost::shared_ptr<Pooler_L0> > &sift_pooler_set, extCSHOT &cshot_producer,  const std::vector<cv::SiftFeatureDetector*> &sift_det_vec, bool max_pool_flag = false);
    
//    pcl::PointCloud<PointT>::Ptr semanticSegment(const std::vector<model*> &model_set, int level);
    std::vector<cv::Mat> gethardNegtive(const model *model_set, int level, bool max_pool = false);
    void InputSemantics(const model *model_set, int level, bool reset = false, bool max_pool = false);
    pcl::PointCloud<PointLT>::Ptr getSemanticLabels();
    std::vector<cv::Mat> sampleSPFea(const int level, int sample_num = -1, bool max_pool_flag = false, bool normalized = true);
//    std::vector<cv::Mat> sampleSPRawFea(const int level, int sample_num = -1, bool max_pool_flag = false);
    
    void init(const pcl::PointCloud<PointT>::Ptr cloud, extCSHOT &cshot_producer, float radius, float ss_ = 0.005);
    void lightInit(const pcl::PointCloud<PointT>::Ptr cloud, extCSHOT& cshot_producer, float radius, float down_ss = 0.005);
    void reset();
    
    void extractForeground(bool constrained_flag);
    
private:
    
    pcl::PointCloud<PointT>::Ptr refineScene(const pcl::PointCloud<PointT>::Ptr scene);
//    std::vector<cv::Mat> combineRawALL(const IDXSET &idx_set, bool max_pool = false);
    std::vector<cv::Mat> combineRaw(const std::vector< std::vector<cv::Mat> > &raw_set, const IDXSET &idx_set, bool max_pool = false, bool normalized = true);
//    std::vector<cv::Mat> getSPRawFea(const IDXSET &idx_set, bool max_pool);
            
    std::vector<cv::Mat> getSPFea(const IDXSET &idx_set, bool max_pool = false, bool normalized = true);
    
    MulInfoT data;
    spExt ext_sp;
    
    std::vector< std::vector<cv::Mat> > raw_sp_lab;
    std::vector< std::vector<cv::Mat> > raw_sp_fpfh;
    std::vector< std::vector<cv::Mat> > raw_sp_sift;
    
    std::vector<cv::Mat> raw_color_fea;
    std::vector<cv::Mat> raw_depth_fea;
    
    std::vector<MulInfoT> raw_data_seg;
    IDXSET segs_to_cloud;
    
//    cv::Mat depth_local;
//    cv::Mat color_local;
//    pcl::PointCloud<PointT>::Ptr down_cloud;
    pcl::PointCloud<PointT>::Ptr full_cloud;
    std::vector<int> segs_label;
    std::vector<float> segs_max_score;
    std::vector< std::vector<float> > class_responses;
    
    bool max_pool_flag;
    
    size_t sp_num;
};

