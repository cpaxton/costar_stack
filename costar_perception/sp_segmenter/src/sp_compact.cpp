#include "sp_segmenter/sp_compact.h"

feaExtractor::feaExtractor(const std::string &shot_path, const std::string &sift_path, const std::string &fpfh_path) : radius_(0.02), 
    down_ss_(0.003), ratio_(0.1),order_(2), use_shot_(true), use_fpfh_(false), use_sift_(false) 
{
    this->setPaths(shot_path,sift_path,fpfh_path);   
}

void feaExtractor::setPaths(const std::string &shot_path, const std::string &sift_path, const std::string &fpfh_path)
{
    // Initialize the CSHOT feature extractor
    hie_producer = boost::shared_ptr<Hier_Pooler> (new Hier_Pooler(radius_));
    hie_producer->LoadDict_L0(shot_path, "200", "200");
    hie_producer->setRatio(ratio_);
    
    // Initialize sift, fpfh and lab pooler
    sift_pooler_set.clear();
    sift_pooler_set.resize(2);
    sift_pooler_set[1] = boost::shared_ptr<Pooler_L0> (new Pooler_L0(-1));
    sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_400.cvmat"); 
    
    fpfh_pooler_set.clear();
    fpfh_pooler_set.resize(2);
    fpfh_pooler_set[1] = boost::shared_ptr<Pooler_L0> (new Pooler_L0(-1));
    fpfh_pooler_set[1]->LoadSeedsPool(fpfh_path+"dict_fpfh_L0_400.cvmat");

    lab_pooler_set.clear();
    lab_pooler_set.resize(6);
    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        lab_pooler_set[i] = boost::shared_ptr<Pooler_L0> (new Pooler_L0);
        lab_pooler_set[i]->setHSIPoolingParams(i);
    }
    
    // sift paramters can be fixed like this, it won't change too much, to 
    // speed up you can reduce the range like [0.7, 1.6] to [0.8, 0.9]
    for( float sigma = 0.7 ; sigma <= 1.61 ; sigma += 0.1 )
    {   
        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
            0, // nFeatures
            4, // nOctaveLayers
            -10000, // contrastThreshold 
            100000, //edgeThreshold
            sigma//sigma
            );
        sift_det_vec.push_back(sift_det);   
    }
}

void feaExtractor::readData(std::string path, ObjectSet& scene_set)
{
    // the input point cloud must be in the organized pcd format
    // for object segment, the non-object area has no valid (x,y,z) values
    
    std::string workspace(path);
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);
    
    size_t file_num = pcd_files.size();
    std::vector<MulInfoT> cur_train;
    for( size_t j = 0 ; j < file_num ; j++ )
    {
        std::string cloud_name(workspace + pcd_files[j]);

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(cloud_name, *cloud);

        if( cloud->size() >= 30 ) // if more than 30 points in the point cloud
        {
            // put a dummy normal object here which is used later on
            pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
            MulInfoT temp = convertPCD(cloud, cloud_normal);
            cur_train.push_back(temp);
        }
    }
    scene_set.push_back(cur_train);
}

std::vector<std::string> feaExtractor::readData(std::string path)
{
    // the input point cloud must be in the organized pcd format
    // for object segment, the non-object area has no valid (x,y,z) values
    
    std::string workspace(path);
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);

    for( size_t j = 0 ; j < pcd_files.size() ; j++ )
    {
        std::string cloud_name(workspace + pcd_files[j]);
        pcd_files[j] = cloud_name;
    }
    
    return pcd_files;
}



//box_num is the number of supervoxels extracted from each object data at each order
//usually for object data, box_num = 10; for background, box_num = 100~200; it is dependent on the actual memory size
int feaExtractor::computeFeature(std::string in_path, std::vector< std::vector<sparseVec> > &final_fea, int box_num)
{
    ObjectSet train_objects;
    //readData(in_path, train_objects);
    std::vector<std::string> file_names = readData(in_path);
    //std::cerr << "Read Data Done!" << std::endl;
    
    //int train_num = train_objects[0].size();
    int train_num = file_names.size();
    int train_dim = -1;
    
    if( final_fea.size() != order_ + 1 )
        final_fea.resize(order_+1);
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < train_num ; j++ )
    {
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(file_names[j], *full_cloud);
        // process cloud more than 30 pts
        if( full_cloud->size() > 30 )
        {
            // pcl::PointCloud<PointT>::Ptr full_cloud = train_objects[0][j].cloud;
            std::cerr << "Processing (" << j + 1 << "/" << train_num <<")\n";
            
            spPooler triple_pooler;
            if (use_sift_) triple_pooler.init(full_cloud, *hie_producer, radius_, down_ss_);
            else triple_pooler.lightInit(full_cloud, *hie_producer, radius_, down_ss_);
            
            if (use_shot_) triple_pooler.build_SP_LAB(lab_pooler_set, false);
            if (use_fpfh_) triple_pooler.build_SP_FPFH(fpfh_pooler_set, radius_, false);
            if (use_sift_) triple_pooler.build_SP_SIFT(sift_pooler_set, *hie_producer, sift_det_vec, false);
            
            for( int ll = 0 ; ll <= order_ ; ll++ )
            {
                std::vector<cv::Mat> sp_fea = triple_pooler.sampleSPFea(ll, box_num, false, true);
                for( std::vector<cv::Mat>::iterator it = sp_fea.begin(); it < sp_fea.end() ; it++ )
                {
                    if( train_dim > 0 && it->cols != train_dim )
                    {
                        std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << train_dim << " " << it->cols << std::endl;
                        exit(0);
                    }
                    else if( train_dim < 0 )
                    {
                        #pragma omp critical
                        {
                            train_dim = it->cols;
                            std::cerr << "Fea Dim: " << train_dim << std::endl;
                        }
                    }    
                    std::vector< sparseVec> this_sparse;
                    sparseCvMat(*it, this_sparse);
                    #pragma omp critical
                    {
                        final_fea[ll].push_back(this_sparse[0]);
                    }
                }
            }
        }
    }
    train_objects.clear();
    
    return train_dim;
}

bool SpCompact::setInputPathSIFT(const std::string &directory_path)
{
    bool success = checkFolderExist(directory_path);
    if (success)
    {
        sift_directory_ = directory_path;
    }
    return success;
}

bool SpCompact::setInputPathSHOT(const std::string &directory_path)
{
    bool success = checkFolderExist(directory_path);
    if (success)
    {
        shot_directory_ = directory_path;
    }
    return success;
}

bool SpCompact::setInputPathFPFH(const std::string &directory_path)
{
    bool success = checkFolderExist(directory_path);
    if (success)
    {
        fpfh_directory_ = directory_path;
    }
    return success;
}

bool SpCompact::setInputTrainingPath(const std::string &directory_path)
{
    bool success = checkFolderExist(directory_path);
    if (success)
    {
        trainining_directory_ = directory_path;
    }
    return success;
}

void SpCompact::setObjectNames(const std::vector<std::string> &object_names)
{
    object_names_ = object_names;
}

void SpCompact::setBackgroundNames(const std::vector<std::string> &background_names)
{
    background_names_ = background_names;
}

void SpCompact::setOutputDirectoryPathFEA(const std::string &directory_path)
{
    bool success = checkFolderExist(directory_path);
    if (!success)
    {
        boost::filesystem::create_directories(directory_path);
    }
    fea_out_directory_ = directory_path;
}

void SpCompact::setOutputDirectoryPathSVM(const std::string &directory_path)
{
    bool success = checkFolderExist(directory_path);
    if (!success)
    {
        boost::filesystem::create_directories(directory_path);
    }
    svm_out_directory_ = directory_path;
}

void SpCompact::setBackgroundSampleNumber(const unsigned int number_of_sample_per_file)
{
    background_sample_num_ = number_of_sample_per_file;
}

void SpCompact::setObjectSampleNumber(const unsigned int number_of_sample_per_file)
{
    foreground_sample_num_ = number_of_sample_per_file;
}

void SpCompact::setCurOrderMax(unsigned int cur_order)
{
    cur_order_max_ = cur_order;
}

void SpCompact::setSkipFeaExtraction(const bool &flag)
{
    skip_fea_ = flag;
}

void SpCompact::setSkipBackgroundSVM(const bool &flag)
{
    skip_background_ = flag;
}

void SpCompact::setSkipMultiSVM(const bool &flag)
{
    skip_multi_ = flag;
}

bool SpCompact::checkFolderExist(const std::string &directory_path) const
{
    return boost::filesystem::is_directory(directory_path);
}

void SpCompact::startTrainingSVM()
{
    if (!skip_fea_) extractFea();
    if (!skip_background_)
    {
        this->doSVM(true);
    }

    if (!skip_multi_ && object_names_.size() > 1)
    {
        this->doSVM(false);
    }

    std::cerr<<std::endl<<"Training complete"<<std::endl;
}


void SpCompact::extractFea()
{
    feaExtractor object_ext(shot_directory_, sift_directory_, fpfh_directory_);
    feaExtractor background_ext(shot_directory_, sift_directory_, fpfh_directory_);
    
    object_ext.setFeaOrder(cur_order_max_ - 1);
    background_ext.setFeaOrder(cur_order_max_ - 1);

    object_ext.setUseSHOT(use_shot_);
    object_ext.setUseFPFH(use_fpfh_);
    object_ext.setUseSIFT(use_sift_);

    background_ext.setUseSHOT(use_shot_);
    background_ext.setUseFPFH(use_fpfh_);
    background_ext.setUseSIFT(use_sift_);

    for( size_t i = 0 ; i < object_names_.size() ; i++ )
    {
        std::stringstream ss;
        ss << i+1;
        
        std::vector< std::vector<sparseVec> > final_fea;
        std::cerr << "Processing object: " << trainining_directory_+"/"+object_names_[i]+"/" << endl;
        int train_dim = object_ext.computeFeature(trainining_directory_+"/"+object_names_[i]+"/", final_fea, foreground_sample_num_);
        
        for( size_t ll = 0 ; ll < final_fea.size(); ++ll )
        {
            if( final_fea[ll].empty() == true )
                continue;
            std::stringstream mm;

            mm << ll;
            saveCvMatSparse(fea_out_directory_ + "/train_" + ss.str() + "_L" + mm.str() + ".smat", final_fea[ll], train_dim);
            final_fea[ll].clear();
        }
    }
    std::cerr << "Object Feature Extraction Done!" << std::endl;
    
    // extracting features for background class
    std::vector< std::vector<sparseVec> > bg_fea;
    int bg_dim = -1;
    for( size_t i = 0 ; i < background_names_.size() ; ++i )
    {
        std::cerr << "Processing background: " << trainining_directory_+"/"+background_names_[i]+"/" << endl;
        bg_dim = background_ext.computeFeature(trainining_directory_+"/"+background_names_[i]+"/", bg_fea, background_sample_num_);
    }
    
    for( size_t ll = 0 ; ll < bg_fea.size() ; ll++ )
    {
        if( bg_fea[ll].empty() == true )
            continue;
        std::stringstream mm;
        mm << ll;
        // background class index 0
        saveCvMatSparse(fea_out_directory_ + "/train_0_L" + mm.str() + ".smat", bg_fea[ll], bg_dim);
        bg_fea[ll].clear();
    }
    std::cerr << "Background Feature Extraction Done!" << std::endl;
}

void SpCompact::doSVM(const bool &is_background_svm)
{
     // weight of incorrectly classified examples (false positive cost)
     // CC = [1, 0.1, 0.01, 0.001, 0.0001]
    float CC = is_background_svm ? binary_cc_ : multi_cc_;
    std::vector< std::pair<int, int> > piece_inds;
    for( int ll = 0 ; ll < cur_order_max_ ; ll++ )
    {
        std::stringstream mm;
        mm << ll;
        std::vector<problem> train_prob_set;
        // looping over all classes including background classes
        for( size_t i = is_background_svm ? 0 : 1 ; i <= object_names_.size()  ; ++i )
        {
            std::stringstream ss;
            ss << i;
            
            std::string train_name = fea_out_directory_ + "train_"+ss.str()+"_L"+mm.str()+".smat";
            if( exists_test(train_name) == false )
                continue;
            
            std::cerr << "Reading: " << train_name << std::endl;
            std::vector<SparseDataOneClass> cur_data(1);
            
            int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds);
            if (is_background_svm)
                cur_data[0].label = i > 0 ? 2 : 1; 
            else
                cur_data[0].label = i + 1; // label below 1 = background, so object label must be > 1
            problem tmp;
            FormFeaSparseMat(cur_data, tmp, cur_data[0].fea_vec.size(), fea_dim);
            train_prob_set.push_back(tmp);
        }
        problem train_prob;
        train_prob.l = 0;
        mergeProbs(train_prob_set, train_prob);

        parameter param;
        GenSVMParamter(param, CC);
        std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;

        model* cur_model = train(&train_prob, &param);
        std::string svm_type = is_background_svm ? "binary_L" : "multi_L";

        save_model((svm_out_directory_ +"/" + svm_type+mm.str()+"_f.model").c_str(), cur_model);
        std::cerr << "Saved: " << svm_out_directory_ << "/" << svm_type+mm.str() << "_f.model" << std::endl;
        destroy_param(&param);
        free(train_prob.y);
        for( int i = 0 ; i < train_prob.l ; i++ )
            free(train_prob.x[i]);
        free(train_prob.x);
    }
}
