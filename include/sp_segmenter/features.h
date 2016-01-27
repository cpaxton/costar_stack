/* 
 * File:   features.h
 * Author: chi
 *
 * Created on July 25, 2014, 2:36 PM
 */

#ifndef FEATURES_H
#define FEATURES_H

#include "sp_segmenter/utility/utility.h"
//#include "../omp/ompcore.h"

struct Hypo{
    cv::Rect box;
    float score;
    int label;
    float ap_ratio;
};

#define FEA_LAYER 5

#define FOCAL_X 539.22
#define FOCAL_Y 539.17
#define CENTER_X 317.06
#define CENTER_Y 232.76
//#define FOCAL_X 570.3
//#define FOCAL_Y 570.3
//#define BOX_WIDTH 0.05
//#define BOX_HEIGHT 0.05
#define IN_OVERLAP_RATIO 0.25

void setObjID(std::map<std::string, int> &model_name_map);

static bool Hypo_comp( const Hypo &hp1, const Hypo &hp2 ){  return hp1.score < hp2.score;  }

bool overlap(const Hypo &hp1, const Hypo &hp2, float ratio = IN_OVERLAP_RATIO);

void CvMatToFeatureNode(cv::Mat one_fea, sparseVec &fea_vec);

std::pair<float, float> readBoxFile(std::string filename);

int readGround(std::string filename, std::vector< std::vector<Hypo> > &hypo_set);

void saveGround(std::string filename, const std::vector< std::vector<Hypo> > &hypo_set);

std::vector< std::pair<float, float> > PRScore(const std::vector< std::vector<Hypo> > &hypo_set, const std::vector< std::vector<Hypo> > &gt_set, int class_num);

void SceneOn2D(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &uv, cv::Mat &map2d, 
        float ap_ratio = 1.0, float fx = FOCAL_X, float fy = FOCAL_Y, float center_x = CENTER_X, float center_y = CENTER_Y);

std::vector<cv::Rect> UniformCubing(const pcl::PointCloud<PointT>::Ptr cloud, const cv::Mat &map2d, const std::vector< std::pair<float, float> > &box_size, 
        int step, float ap_ratio = 1.0, float fx = FOCAL_X, float fy = FOCAL_Y, float center_x = CENTER_X, float center_y = CENTER_Y);

cv::Mat getImage(const pcl::PointCloud<PointT>::Ptr cloud, float ap_ratio = 1.0, float fx = FOCAL_X, float fy = FOCAL_Y, float center_x = CENTER_X, float center_y = CENTER_Y);

pcl::PointIndices::Ptr getSegIdx(const cv::Mat &map2d, const cv::Rect &reg);

std::vector<pcl::PointIndices::Ptr> CropSegs(const MulInfoT &data, const std::vector< std::pair<float, float> > &box_size, 
                                            int max_rand_sample, float in_box_ratio, float fx = FOCAL_X, float fy = FOCAL_Y, float center_x = CENTER_X, float center_y = CENTER_Y);

std::vector<cv::Mat> extFea(const std::vector<cv::Mat> &main_fea, const std::vector<int> &idx);

cv::Mat clOn2D(const pcl::PointCloud<PointT>::Ptr cloud, float fx = FOCAL_X, float fy = FOCAL_Y, float center_x_ = CENTER_X, float center_y_ = CENTER_Y);

std::vector<pcl::PointIndices::Ptr> CropSegs(const MulInfoT &data, int min_num, int max_num, 
        int max_rand_sample, float fx = FOCAL_X, float fy = FOCAL_Y, float center_x = CENTER_X, float center_y = CENTER_Y);

void LoadSeedsHigh(std::string high_seed_file, cv::flann::Index &fea_tree, cv::Mat &fea_seeds, int &feaK, float ratio = 0.15);

void LoadSeedsNormal(std::string normal_seed_file, cv::flann::Index &fea_tree, cv::Mat &fea_seeds, int &feaK, float ratio = 0.15);

cv::Mat getNormalCode(const MulInfoT &data, cv::flann::Index &fea_tree, int fea_len, int feaK);

void get_SHOT_PCSHOT_Code_ss(const MulInfoT &data, 
        const cv::Mat shot_dict, cv::flann::Index &shot_tree, int shot_len, int shotK, cv::Mat &shot_code, 
        const cv::Mat pcshot_dict, cv::flann::Index &pcshot_tree, int pcshot_len, int pcshotK, cv::Mat &pcshot_code, 
        MulInfoT &high_data, float rad, float ss = -1);

void getSHOTCode_ss(const MulInfoT &data, const cv::Mat dict, cv::flann::Index &fea_tree, cv::Mat &high_code, MulInfoT &high_data, int fea_len, int feaK, float rad, int type = 0, float ss = -1);

cv::Mat getColorDiff(const pcl::PointCloud<PointT>::Ptr cloud);

//std::vector<cv::KeyPoint> extSIFTKeys(const cv::Mat &cur_gray, const std::vector<cv::SiftFeatureDetector*> &sift_det_vec);

class Pooler_L0{
public:
    Pooler_L0(int hsi_len);
    Pooler_L0();
    ~Pooler_L0(){}
    

    cv::Mat PoolOneDomain(const cv::Mat &domain, const cv::Mat &fea_code, int pool_type, bool max_pool=true);
    std::vector<cv::Mat> PoolOneDomain_Raw(const cv::Mat &domain, const cv::Mat &fea_code, int pool_type, bool max_pool=true);
    cv::Mat PoolHybridDomain(const cv::Mat &mixed_domain, const cv::Mat &fea_code, bool max_pool=true);
    
    void setHSIPoolingParams(int hsi_len);
    int getHSIPoolLen(){return total_hs_len;}
    int getXYPoolLen(){return total_xy_len;}
    
    cv::Mat getGenericPoolMat(const cv::Mat &domain);
    int getGenericPoolLen(){return pool_len;}
    int LoadSeedsPool(std::string pool_seed_file);
    
   // void getHybridPoolMat(const cv::Mat &mixed_domain, std::vector< std::vector<int> > &pool_idxs, float scale = 1);
    int getHybridPoolLen(){return hybrid_len;}
    int LoadHybridPool(std::string hybrid_center_file, float radius);
    
private:
    //int getXYZPoolIdx(const cv::Mat &xyz);
    int getXYPoolIdx(const cv::Mat &xyz);
    int getHSIPoolIdx(const cv::Mat &hsi);
    
    int getGenericPoolIdx(const cv::Mat &pool_fea);
    void getHSPoolIdxs(const cv::Mat &hsi, std::vector<int> &idxs, std::vector<float> &w, int K);

    cv::Mat pool_seeds;
    cv::flann::Index pool_tree;
    int pool_len;
    
    cv::Mat hybrid_centers;
    cv::Mat hybrid_range;
    int hybrid_len;
   
    int len_h, len_s, len_i, total_hs_len, total_xy_len;
    float hs_cell_scale; 
};

void poolRGBLayer(Pooler_L0 &pooler, const MulInfoT &inst, const cv::Mat &local_fea, std::vector<cv::Mat> &pool_fea_vec);
void poolXYZLayer(Pooler_L0 &pooler, const MulInfoT &inst, const cv::Mat &local_fea, std::vector<cv::Mat> &pool_fea_vec);

cv::Mat multiPool(const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const MulInfoT &inst, const std::vector<cv::Mat> &local_fea);

cv::Mat multiFPFHPool(const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const MulInfoT &inst, const std::vector<cv::Mat> &local_fea, float radius);

std::vector<cv::Mat> multiPool_raw(const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const MulInfoT &inst, const std::vector<cv::Mat> &local_fea);

class Hier_Pooler{
public:
    Hier_Pooler(float rad = 0.03);
    ~Hier_Pooler();
    
    std::vector<cv::Mat> getHierFea(MulInfoT &data, int layer);
    // For the usage of Dictionary Learning, SubSampling features
    std::vector<cv::Mat> getRawFea(MulInfoT &data, int layer, size_t max_num);
    
    void setRatio(float rr_) {ratio=rr_;}
    std::vector<int> LoadDict_L0(std::string path, std::string colorK, std::string depthK, std::string jointK="");
    std::vector<int> LoadDict_L1(std::string dict_path, std::vector<std::string> dictK);
    std::vector<int> LoadDict_L2(std::string dict_path, std::vector<std::string> dictK);
    
    std::vector<int> sampleRaw_L0(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normal,
                    cv::Mat &depth_fea, cv::Mat &color_fea, int max_num, float rad = 0.03);
    std::vector<cv::Mat> EncodeLayer_L0(const cv::Mat &depth_fea, const cv::Mat &color_fea);
    
private:
    void computeRaw_L0(MulInfoT &data, cv::Mat &depth_fea, cv::Mat &color_fea, float rad = 0.03);
    
    
    std::vector<cv::Mat> PoolLayer_L1(const MulInfoT &data, const std::vector<cv::Mat> code_L0, std::vector<size_t> idxs, bool max_pool=true);
    std::vector<cv::Mat> EncodeLayer_L1(const std::vector<cv::Mat> rawfea_L1);
    
    std::vector<cv::Mat> PoolLayer_L2(const MulInfoT &data, const std::vector<cv::Mat> code_L1, std::vector<size_t> idxs, bool max_pool=true);
    std::vector<cv::Mat> EncodeLayer_L2(const std::vector<cv::Mat> rawfea_L2);
    
    cv::Mat dict_color_L0, dict_depth_L0, dict_joint_L0;
    cv::flann::Index tree_color_L0, tree_depth_L0, tree_joint_L0;
    
    cv::Mat dict_colorInLAB_L1, dict_colorInXYZ_L1, dict_depthInLAB_L1, dict_depthInXYZ_L1;
    cv::flann::Index tree_colorInLAB_L1, tree_colorInXYZ_L1, tree_depthInLAB_L1, tree_depthInXYZ_L1;
    
    cv::Mat dict_colorInLAB_L2, dict_colorInXYZ_L2, dict_depthInLAB_L2, dict_depthInXYZ_L2;
    cv::flann::Index tree_colorInLAB_L2, tree_colorInXYZ_L2, tree_depthInLAB_L2, tree_depthInXYZ_L2;
    
    //0: color-lab
    //1: depth-lab
    //2: color-xyz
    //3: depth-xyz
    bool pool_flag[4];          
    int pool_type_num;          //4
    float pool_radius_L0;       // 0.03
    float pool_radius_L1[2];    // 0.05, 0.05
    float pool_radius_L2[2];    // 0.05, 0.05
    
    float ratio;                //0.15
};

class IntImager{
public:
    IntImager(float ap=0.5);
    ~IntImager(){};
    
    void ComputeInt(const MulInfoT &data, const std::vector<cv::Mat> &local_fea);
    std::vector<Hypo> DetectObjects(const model* obj_model, const std::pair<float, float> &cur_box, int step);
    void setVoidFlags(std::vector<bool> &void_flags, std::vector<int> &valid_dims);
    void setCameraParams(float fx_, float fy_, float center_x_, float center_y_);
    void setTarget(size_t label);
    
private:
    //void CloudOn2D(const MulInfoT &data, cv::Mat &uv, cv::Mat &map2d);
    void Pooling(const cv::Rect &reg, sparseVec &pooled_fea);   
    std::vector<Hypo> nonmax_suppress(std::vector<Hypo> &hypo_set);
    
    void Reset();
    std::vector<int> getPoolIdx(cv::Mat lab);
    
    float fx, fy;
    float center_x, center_y;
    int img_w, img_h;
    float ap_ratio;                 //0.5 by default
    
    int lab_layer;                  //5
    int total_bin;                  //number of receptive fields
    std::vector<float> layer_scale;
    std::vector<int> bin_to_layer;
    
    std::vector<bool> *void_ptr;
    std::vector<int> *valid_idx;
    
    std::vector<bool> target_flags;
    
    int fea_dim;                    //set by void_ptr->size()
    int valid_dim;                  //set by valid_idx->size()
    std::vector<int> dim_per_cell;  //set by local codes
    
    //change per frame
    std::vector<cv::Mat> int_imgs;  //reset prior to each new coming frame
    cv::Mat int_map2d, int_uv;
    pcl::PointCloud<PointT>::Ptr cur_down_cloud;
    
};

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
    //void build_SP_SIFT(const std::vector< boost::shared_ptr<Pooler_L0> > &sift_pooler_set, Hier_Pooler &cshot_producer,  const std::vector<cv::SiftFeatureDetector*> &sift_det_vec, bool max_pool_flag = false);
    
//    pcl::PointCloud<PointT>::Ptr semanticSegment(const std::vector<model*> &model_set, int level);
    std::vector<cv::Mat> gethardNegtive(const model *model_set, int level, bool max_pool = false);
    void InputSemantics(const model *model_set, int level, bool reset = false, bool max_pool = false);
    pcl::PointCloud<PointLT>::Ptr getSemanticLabels();
    std::vector<cv::Mat> sampleSPFea(const int level, int sample_num = -1, bool max_pool_flag = false, bool normalized = true);
//    std::vector<cv::Mat> sampleSPRawFea(const int level, int sample_num = -1, bool max_pool_flag = false);
    
    void init(const pcl::PointCloud<PointT>::Ptr cloud, Hier_Pooler &cshot_producer, float radius, float ss_ = 0.005);
    void lightInit(const pcl::PointCloud<PointT>::Ptr cloud, Hier_Pooler& cshot_producer, float radius, float down_ss = 0.005);
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

#endif //features_h