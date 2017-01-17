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
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>

#include "sp_segmenter/features.h"

class feaExtractor{
public:
    feaExtractor(): radius_(0.02), down_ss_(0.003), ratio_(0.1), order_(2), use_shot_(true), use_fpfh_(false), use_sift_(false) {};
    feaExtractor(const std::string &shot_path, const std::string &sift_path, const std::string &fpfh_path);
    ~feaExtractor(){};

    void setPaths(const std::string &shot_path, const std::string &sift_path, const std::string &fpfh_path);
    template<typename NumericFloatType, typename NumericIntType>
        void setFeaParameter(const NumericFloatType &radius, const NumericFloatType &down_ss, const NumericFloatType &ratio, const NumericIntType &order)
    {
        radius_ = radius;
        down_ss_ = down_ss;
        ratio_ = ratio;
        order_ = order;
    }

    template<typename NumericIntType>
    void setFeaOrder(const NumericIntType &order)
    {
        order_ = order;
    }

    void setUseSHOT(const bool &use_shot) { use_shot_ = use_shot; }
    void setUseFPFH(const bool &use_fpfh) { use_fpfh_ = use_fpfh; }
    void setUseSIFT(const bool &use_sift) { use_sift_ = use_sift; }

    // the in_path stores the organized point cloud data for one object or background class
    // the vector final_fea stores the computed features for the in_path class at different orders
    // box_num: sample number extracted in one pcd files.
    int computeFeature(std::string in_path, std::vector< std::vector<sparseVec> > &final_fea, int box_num);
    
protected:
    void readData(std::string path, ObjectSet &scene_set);
    std::vector<std::string> readData(std::string path);
    boost::shared_ptr<Hier_Pooler> hie_producer;
    
    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set;
    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set;
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set;
    
    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
    
    // Can be fixed in different settings
    float radius_;   //0.02m,    radius for CSHOT feature
    float down_ss_;  //0.003m,   point cloud downsampled rate
    float ratio_;    //0.1,      soft encoder ratio
    int order_;      //2,        order degree for training
    // int box_num;    //10,       number of supervoxels extracted from each object data
    ///////////////////////////////////////////////////////////////////////////////

    bool use_shot_, use_fpfh_, use_sift_;
};

class SpCompact{
public:
    SpCompact(): trainining_directory_("data/training"), sift_directory_("data/UW_sift_dict"),
        shot_directory_("data/UW_shot_dict"), fpfh_directory_("data/UW_fpfh_dict"),
        binary_cc_(0.001), multi_cc_(0.001), background_sample_num_(66), foreground_sample_num_(100),
        skip_fea_(false), skip_background_(false), skip_multi_(false), cur_order_max_(3), 
        use_shot_(true), use_fpfh_(false), use_sift_(false) 
    {
        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    }

    // Do the training
    void startTrainingSVM();

    // Setting up file IO locations. Return false if directory is not exist
    bool setInputPathSIFT(const std::string &directory_path);
    bool setInputPathSHOT(const std::string &directory_path);
    bool setInputPathFPFH(const std::string &directory_path);

    void setUseSHOT(const bool &use_shot) { use_shot_ = use_shot; }
    void setUseFPFH(const bool &use_fpfh) { use_fpfh_ = use_fpfh; }
    void setUseSIFT(const bool &use_sift) { use_sift_ = use_sift; }

    // Setting up training directory locations. Return false if directory is not exist
    bool setInputTrainingPath(const std::string &directory_path);

    // The class will load the pcd files from directory_path/object_name[1 .. n]/*.pcd
    // and directory_path/background_name[1 .. n]/*.pcd
    void setObjectNames(const std::vector<std::string> &object_names);
    void setBackgroundNames(const std::vector<std::string> &background_names);

    // Set file output directory. Will create the output directory if it does not exist
    void setOutputDirectoryPathFEA(const std::string &directory_path);
    void setOutputDirectoryPathSVM(const std::string &directory_path);

    // weight of incorrectly classified cost
    template<typename NumericType>
        void setForegroundCC(const NumericType &cc)
    {
        binary_cc_ = float(cc);
    }

    template<typename NumericType>
        void setMultiCC(const NumericType &cc)
    {
        multi_cc_ = float(cc);
    }

    // NUM_OBJECT_TRAINING_DATA*OBJ_SAMPLES ~= NUM_BACKGROUND_TRAINING_DATA*NUM_BG_SAMPLES
    void setBackgroundSampleNumber(const unsigned int number_of_sample_per_file);
    void setObjectSampleNumber(const unsigned int number_of_sample_per_file);

    void setCurOrderMax(unsigned int cur_order);

    // Skip Feature Extraction. Use only for repeating svm training when feature extraction
    void setSkipFeaExtraction(const bool &flag);

    // Skip Binary SVM (Background/Foreground classification)
    void setSkipBackgroundSVM(const bool &flag);

    // Skip Multi Class SVM (Inter Object classification)
    void setSkipMultiSVM(const bool &flag);

protected:
    bool checkFolderExist(const std::string &directory_path) const;
    void extractFea();
    void doSVM(const bool &background_svm);

    std::string trainining_directory_, sift_directory_, shot_directory_, fpfh_directory_;
    std::string fea_out_directory_, svm_out_directory_;
    std::vector<std::string> object_names_, background_names_;

    float binary_cc_, multi_cc_;
    unsigned int background_sample_num_, foreground_sample_num_;
    bool skip_fea_, skip_background_, skip_multi_;
    unsigned int cur_order_max_;

    bool use_shot_, use_fpfh_, use_sift_;
};
