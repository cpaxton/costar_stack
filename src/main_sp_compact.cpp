#include <opencv2/core/core.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/stringVectorArgsReader.h"

// ros header for roslaunch capability
#include <ros/ros.h>

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
#include <boost/lambda/bind.hpp>
/*
std::size_t num_files_in_folder(std::string string_path)
{
    using namespace boost::filesystem;
    using namespace boost::lambda;

    path the_path( "/home/myhome" );

    int cnt = std::count_if(
        directory_iterator(the_path),
        directory_iterator(),
        bind( static_cast<bool(*)(const path&)>(is_regular_file), 
          bind( &directory_entry::path, _1 ) ) );
    // a little explanation is required here,
    // we need to use static_cast to specify which 
    // version of is_regular_file function we intend to use

    std::cout << cnt << std::endl;

    return 0;
}
*/


class feaExtractor{
public:
    feaExtractor(std::string shot_path, std::string sift_path, std::string fpfh_path);
    ~feaExtractor(){};

    // the in_path stores the organized point cloud data for one object or background class
    // the vector final_fea stores the computed features for the in_path class at different orders
    int computeFeature(std::string in_path, std::vector< std::vector<sparseVec> > &final_fea, int box_num);
    
    // modify order paramter
    void setOrder(int order_){order = order_;}

private:
    void readData(std::string path, ObjectSet &scene_set);
    boost::shared_ptr<Hier_Pooler> hie_producer;
    
    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set;
    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set;
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set;
    
    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
    
    // Can be fixed in different settings
    float radius;   //0.02m,    radius for CSHOT feature
    float down_ss;  //0.003m,   point cloud downsampled rate
    float ratio;    //0.1,      soft encoder ratio
    int order;      //2,        order degree for training
//    int box_num;    //10,       number of supervoxels extracted from each object data
    ///////////////////////////////////////////////////////////////////////////////
};

int objSampleNum = 66;
int bgSampleNum = 100;

void extractFea(std::string root_path, std::string out_fea_path,
	std::vector<std::string> obj_names, std::vector<std::string>  bg_names,
	feaExtractor object_ext, feaExtractor background_ext)
{
    // the sample number extracted in one pcd files. 1200 foreground 800 background
    int obj_sample_num = objSampleNum; //66;
    int bg_sample_num = bgSampleNum; //100;
    int cur_order_max = 3;

	for( size_t i = 0 ; i < obj_names.size() ; i++ )
    {
        std::stringstream ss;
        ss << i+1;
        
        std::vector< std::vector<sparseVec> > final_fea;
        std::cerr << "Processing object: " << root_path+obj_names[i]+"/" << endl;
        int train_dim = object_ext.computeFeature(root_path+obj_names[i]+"/", final_fea, obj_sample_num);
        cur_order_max = (int)final_fea.size();
        
        for( int ll = 0 ; ll < cur_order_max ; ll++ )
        {
            if( final_fea[ll].empty() == true )
                continue;
            std::stringstream mm;
;
            mm << ll;
            saveCvMatSparse(out_fea_path + "train_" + ss.str() + "_L" + mm.str() + ".smat", final_fea[ll], train_dim);
            final_fea[ll].clear();
        }
    }
    std::cerr << "Object Feature Extraction Done!" << std::endl;
    
    // extracting features for background class
    std::vector< std::vector<sparseVec> > bg_fea;
    int bg_dim = -1;
    for( size_t i = 0 ; i < bg_names.size() ; i++ )
    {
;
        std::cerr << "Processing background: " << root_path+bg_names[i]+"/" << endl;
        bg_dim = background_ext.computeFeature(root_path+bg_names[i]+"/", bg_fea, bg_sample_num);
    }
    
    for( size_t ll = 0 ; ll < bg_fea.size() ; ll++ )
    {
        if( bg_fea[ll].empty() == true )
            continue;
        std::stringstream mm;
        mm << ll;
        // background class index 0
        saveCvMatSparse(out_fea_path + "train_0_L" + mm.str() + ".smat", bg_fea[ll], bg_dim);
;
        bg_fea[ll].clear();
    }
    std::cerr << "Background Feature Extraction Done!" << std::endl;
;
}
//*std::vector
int main(int argc, char** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ros::init(argc,argv,"sp_compact_node");
    ros::NodeHandle nh("~");
    
    std::string root_path;
    float foregroundBackgroundCC;
    float multiclassCC;
    nh.param("root_path", root_path, std::string("data/training/"));
    nh.param("foreground_cc", foregroundBackgroundCC, 0.001f);
    nh.param("multiclass_cc", multiclassCC, 0.001f);
    nh.param("bg_sample_num", bgSampleNum, 66); //NUM_OBJECT_TRAINING_DATA*OBJ_SAMPLES = NUM_BACKGROUND_TRAINING_DATA*NUM_BG_SAMPLES
    nh.param("obj_sample_num", objSampleNum, 100); // these should be chosen so the number of samples is the same for each
    
    // adding object classes by reading from nodeHandle args
    std::vector<std::string> obj_names = stringVectorArgsReader (nh, "obj_names", std::string("drill"));
    
    // adding background classes by reading from nodeHandle args
    std::vector<std::string> bg_names = stringVectorArgsReader (nh, "bg_names", std::string("UR5"));

    std::string out_fea_path, out_svm_path;
    nh.param("out_fea_path",out_fea_path,std::string("fea_pool/"));
    std::stringstream ss;
    ss << root_path << "/";
    for (size_t i = 0; i < obj_names.size(); ++i)
    {
        ss << obj_names.at(i);
        if (i < obj_names.size()-1) ss << "_";
    }
    ss << "_svm/";
    out_svm_path = ss.str();
    // nh.param("out_svm_path",out_svm_path,std::string("svm_pool/"));

    boost::filesystem::create_directories(out_fea_path);
    boost::filesystem::create_directories(out_svm_path);
    
    // paths for dictionaries
    std::string shot_path, sift_path, fpfh_path;
    nh.param("shot_path",shot_path,std::string("data/UW_shot_dict/"));
    nh.param("sift_path",sift_path,std::string("data/UW_sift_dict/"));
    nh.param("fpfh_path",fpfh_path,std::string("data/UW_fpfh_dict/"));

    bool skip_fea;
    nh.param("skip_fea",skip_fea,false);
    
    feaExtractor object_ext(shot_path, sift_path, fpfh_path);
    feaExtractor background_ext(shot_path, sift_path, fpfh_path);
    
    // extracting features for object classes
    if (!skip_fea) extractFea(root_path,out_fea_path,obj_names,bg_names,object_ext,background_ext);

    // binary classification
    {
        int cur_order_max = 3;
        // Reading features to train svm
        float CC = foregroundBackgroundCC; // weight of incorrectly classified examples (false positive cost) // CC = [1, 0.1, 0.01, 0.001, 0.0001]
        std::vector< std::pair<int, int> > piece_inds;
        for( int ll = 0 ; ll < cur_order_max ; ll++ )
        {
            std::stringstream mm;
            mm << ll;
            std::vector<problem> train_prob_set;
            // looping over all classes including background classes
            for( size_t i = 0 ; i <= obj_names.size()  ; i++ )
            {
                std::stringstream ss;
                ss << i;
                
                std::string train_name = out_fea_path + "train_"+ss.str()+"_L"+mm.str()+".smat";
                if( exists_test(train_name) == false )
                    continue;
                
                std::cerr << "Reading: " << train_name << std::endl;
                std::vector<SparseDataOneClass> cur_data(1);
                
                int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds);
                cur_data[0].label = i > 0 ? 2 : 1; 

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
            save_model((out_svm_path + "binary_L"+mm.str()+"_f.model").c_str(), cur_model);
            std::cerr << "Saved: " << out_svm_path << "binary_L"+mm.str() << "_f.model" << std::endl;
            destroy_param(&param);
            free(train_prob.y);
            for( int i = 0 ; i < train_prob.l ; i++ )
                free(train_prob.x[i]);
            free(train_prob.x);
        }
    }
      
    if (obj_names.size() > 1) // multi classification
    {
        int cur_order_max = 3;
        // Reading features to train svm
        float CC = multiclassCC; // weight of incorrectly classified examples (false positive cost)
        std::vector< std::pair<int, int> > piece_inds;
        for( int ll = 0 ; ll < cur_order_max ; ll++ )
        {
            std::stringstream mm;
            mm << ll;
            std::vector<problem> train_prob_set;
            // looping over all classes including background classes
            for( size_t i = 1 ; i <= obj_names.size()  ; i++ )
            {
                std::stringstream ss;
                ss << i;
                
                std::string train_name = out_fea_path + "train_"+ss.str()+"_L"+mm.str()+".smat";
                if( exists_test(train_name) == false )
                    continue;
                
                std::cerr << "Reading: " << train_name << std::endl;
                std::vector<SparseDataOneClass> cur_data(1);
                
                int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds);
                cur_data[0].label = i + 1; // label below 1: background, so object label must be > 1. Need to be clarified

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
            save_model((out_svm_path + "multi_L"+mm.str()+"_f.model").c_str(), cur_model);
            std::cerr << "Saved: " << out_svm_path << "multi_L"+mm.str() << "_f.model" << std::endl;
            
            destroy_param(&param);
            free(train_prob.y);
            for( int i = 0 ; i < train_prob.l ; i++ )
                free(train_prob.x[i]);
            free(train_prob.x);
        }
    }

    
    std::cerr<<std::endl<<"Training complete"<<std::endl;
    ros::shutdown();
    return 0;
} 
//*/

feaExtractor::feaExtractor(std::string shot_path, std::string sift_path, std::string fpfh_path)
{
    radius = 0.02;
    down_ss = 0.003;
    ratio = 0.1;
    order = 2;
    
    // Initialize the CSHOT feature extractor
    hie_producer = boost::shared_ptr<Hier_Pooler> (new Hier_Pooler(radius));
    hie_producer->LoadDict_L0(shot_path, "200", "200");
    hie_producer->setRatio(ratio);
    
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


//box_num is the number of supervoxels extracted from each object data at each order
//usually for object data, box_num = 10; for background, box_num = 100~200; it is dependent on the actual memory size
int feaExtractor::computeFeature(std::string in_path, std::vector< std::vector<sparseVec> > &final_fea, int box_num)
{
    ObjectSet train_objects;
    readData(in_path, train_objects);
    
    int train_num = train_objects[0].size();
    int train_dim = -1;
    
    if( final_fea.size() != order + 1 )
        final_fea.resize(order+1);
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < train_num ; j++ )
    {
        pcl::PointCloud<PointT>::Ptr full_cloud = train_objects[0][j].cloud;
		std::cerr << "Processing (" << j + 1 << "/" << train_num <<")\n";
        
        // disable sift and fpfh pooling for now
        spPooler triple_pooler;
        triple_pooler.lightInit(full_cloud, *hie_producer, radius, down_ss);
        triple_pooler.build_SP_LAB(lab_pooler_set, false);
//        triple_pooler.build_SP_FPFH(fpfh_pooler_set, radius, false);
//        triple_pooler.build_SP_SIFT(sift_pooler_set, hie_producer, sift_det_vec, false);
        
        for( int ll = 0 ; ll <= order ; ll++ )
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
    train_objects.clear();
    
    return train_dim;
}



/*
 int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/ht10/");
    std::string out_path("/home/chi/JHUIT/new_ht10/");
    
    std::string obj_name("drill");
    
    pcl::console::parse_argument(argc, argv, "--m", obj_name);
    out_path = out_path + obj_name + "/";
    boost::filesystem::create_directories(out_path);
    
    std::string workspace(in_path + obj_name + "/");
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);

    size_t file_num = pcd_files.size();
    for( size_t j = 0 ; j < file_num ; j++ )
    {
        std::string core_name(pcd_files[j].substr(0, pcd_files[j].size()-4));

        std::string cloud_name(workspace + pcd_files[j]);
        std::string img_name(workspace+core_name+"_rgbcrop.png");
        std::string idx_name(workspace+"3D2D_"+core_name+".cvmat");

        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(cloud_name, *cloud);

        cv::Mat img = cv::imread(img_name);
        cv::Mat map3d2d;
        readMat(idx_name, map3d2d);
        cv::Mat map2d = cv::Mat::ones(img.rows, img.cols, CV_32SC1) * -1;
        for(int k = 0 ; k < map3d2d.rows ; k++ )
            map2d.at<int>(map3d2d.at<int>(k, 1), map3d2d.at<int>(k, 0) ) = k;

        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        full_cloud->resize(map2d.rows*map2d.cols);
        for(int r = 0, this_idx = 0 ; r < map2d.rows ; r++ ){
            for(int c = 0 ; c < map2d.cols ; c++, this_idx++ )
            {
                int idx2 = map2d.at<int>(r, c);
                if( idx2 >= 0 )
                {
                    full_cloud->at(this_idx).x = cloud->at(idx2).x;
                    full_cloud->at(this_idx).y = cloud->at(idx2).y;
                    full_cloud->at(this_idx).z = cloud->at(idx2).z;
                    full_cloud->at(this_idx).rgba = cloud->at(idx2).rgba;
                }
                else
                {
                    uint32_t rgba = img.at<uchar>(r, c*3+0) | img.at<uchar>(r, c*3+1) << 8 | img.at<uchar>(r, c*3+2) << 16;
                    full_cloud->at(this_idx).x = std::numeric_limits<float>::quiet_NaN();
                    full_cloud->at(this_idx).y = std::numeric_limits<float>::quiet_NaN();
                    full_cloud->at(this_idx).z = std::numeric_limits<float>::quiet_NaN();
                    full_cloud->at(this_idx).rgba = rgba;
                }
            }
        }
        full_cloud->height = map2d.rows;
        full_cloud->width = map2d.cols;
        full_cloud->is_dense = false;
        
        pcl::io::savePCDFile(out_path + "/" + pcd_files[j], *full_cloud, true);
    }
    
     
    
    return 1;
} 
*/
