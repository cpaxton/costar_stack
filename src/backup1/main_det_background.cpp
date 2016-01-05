#include <opencv2/core/core.hpp>

#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

#define IN_TO_BOX_RATIO 0.1
#define BOX_SIZE 0.05

size_t readSparseMat(std::string filename, std::vector<sparseVec> &fea_vec);

std::vector<pcl::PointIndices::Ptr> hardSampling(const MulInfoT &data, float in_box_ratio = 0.5)
{
    std::vector<pcl::PointIndices::Ptr> inlier_set, tmp;
    
    std::vector< std::pair<float, float> > box(1);
    box[0].first = 0.03; box[0].second = 0.03;
    tmp = CropSegs(data, box, 200, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.04; box[0].second = 0.04;
    tmp = CropSegs(data, box, 150, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.05; box[0].second = 0.05;
    tmp = CropSegs(data, box, 100, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.06; box[0].second = 0.06;
    tmp = CropSegs(data, box, 50, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.03; box[0].second = 0.04;
    tmp = CropSegs(data, box, 150, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.04; box[0].second = 0.03;
    tmp = CropSegs(data, box, 150, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.03; box[0].second = 0.05;
    tmp = CropSegs(data, box, 100, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    box[0].first = 0.05; box[0].second = 0.03;
    tmp = CropSegs(data, box, 100, in_box_ratio);
    inlier_set.insert(inlier_set.end(), tmp.begin(), tmp.end());
    tmp.clear();
    
    return inlier_set;
}

//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/background/");
    std::string out_path("../data_pool/ht2_background/");
    //std::string in_path("/home/chi/UW_scene/background/");
    //std::string out_path("UW_train_background/");
    
    std::string dict_path("BB_new_dict/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    float radius = 0.03;
    float down_ss = 0.005;
    float in_box_ratio = 0.5;
    //int max_rand_sample = 1000;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", in_box_ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    //pcl::console::parse_argument(argc, argv, "--max", max_rand_sample);
    
    bool debug_flag = false;
    if( pcl::console::find_switch(argc, argv, "-d") == true )
        debug_flag = true;
    
    Hier_Pooler hie_producer_3(radius);
    hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    hie_producer_3.setRatio(0);
    
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::vector< boost::shared_ptr<Pooler_L0> > pooler_set(FEA_LAYER+1);
    for( size_t i = 1 ; i < pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        pooler_set[i] = cur_pooler;
    }
    
    //read all pcd files in the current directory and sub-directories
    std::vector<std::string> pcd_files;
    std::vector<std::string> normal_files;
    boost::filesystem::path ip(in_path);
    std::vector< std::string > init_ret;
    find_dirs(ip, init_ret);
    ReadCloudNormal(in_path, pcd_files, normal_files);
    for( size_t i = 0 ; i < init_ret.size() ; i++ )
        ReadCloudNormal(in_path + init_ret[i] + "/", pcd_files, normal_files);
    size_t file_num = pcd_files.size();
    
//    std::pair<float, float> cur_box = readBoxFile( out_path + "model.box");
//    std::cerr << "Training Box: " << cur_box.first << "-" << cur_box.second << std::endl;
//    std::vector< std::pair<float, float> > box_size;
//    box_size.push_back(cur_box);
    
    pcl::VoxelGrid<PointT> sor;
    for( size_t i = 0 ; i < file_num ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        pcl::ExtractIndices<PointT> ext;
        ext.setNegative(false);
        
        std::cerr << "Loading-" << pcd_files[i] << std::endl;
            
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *full_cloud);
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        std::vector<int> idx_ff;
        pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);
        
        if( debug_flag )
        {
            cv::Mat rgb_frame = getImage(cloud, 1.0);
            cv::imshow("Show", rgb_frame);
            cv::waitKey();
        }
        
        pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(normal_files[i], *cloud_normal);
        
        MulInfoT tmp_data = convertPCD(cloud, cloud_normal);
        tmp_data.down_cloud = tmp_data.cloud;
//        std::vector<pcl::PointIndices::Ptr> inlier_set = CropSegs(tmp_data, box_size, max_rand_sample, in_box_ratio);
        std::vector<pcl::PointIndices::Ptr> inlier_set = hardSampling(tmp_data, in_box_ratio);
        
        ObjectSet new_set;
        new_set.clear();
        new_set.resize(1);
        for(std::vector<pcl::PointIndices::Ptr>::iterator it_inlier = inlier_set.begin() ; it_inlier < inlier_set.end() ; it_inlier++ )
        {
            pcl::PointCloud<PointT>::Ptr cur_seg(new pcl::PointCloud<PointT>());
            // Extract the inliers
            ext.setInputCloud (tmp_data.down_cloud);
            ext.setIndices (*it_inlier);
            ext.filter (*cur_seg);
            
            sor.setInputCloud(cur_seg);
            sor.setLeafSize(down_ss, down_ss, down_ss);
            pcl::PointCloud<PointT>::Ptr down_seg(new pcl::PointCloud<PointT>());
            sor.filter(*down_seg);
            if( down_seg->size() == cur_seg->size() )     //downsampling failed
            {
                std::cerr << "Downsampling Data Failed!" << std::endl;
                continue;
            }
            MulInfoT cur_data = convertPCD(tmp_data.cloud, tmp_data.cloud_normals);
            cur_data.down_cloud = down_seg;
            
            if( debug_flag )
            {
                cv::Mat img = getImage(down_seg);
                cv::imshow("Show", img);
                cv::waitKey(0);
            }
            
            PreCloud(cur_data, -1, false);
            new_set[0].push_back(cur_data);
        }
        
        size_t data_num = new_set[0].size();
        
        // Extracting the features
        std::vector< sparseVec> final_train_fea(data_num);
        int final_train_fea_dim = -1;
        #pragma omp parallel for schedule(dynamic, 1)
        for( size_t j = 0 ; j < data_num ; j++ )
        {
            MulInfoT *inst_ptr = &new_set[0][j];
            
            std::vector<cv::Mat> local_fea = hie_producer_3.getHierFea(*inst_ptr, 0);
            cv::Mat final_temp = multiPool(pooler_set, *inst_ptr, local_fea);
            local_fea.clear();
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
        }
        
        saveCvMatSparse(out_path + "background_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
        new_set.clear();
    }
    
    // Final Combining Background Model
    // Loading Background Model
    boost::filesystem::path op(out_path);
    std::vector< std::string > final_ret;
    find_files(op, ".smat", final_ret);
    
    std::vector< sparseVec> train_set, test_set;
    
    int idx = 0;
    int fea_dim = -1;
    for( std::vector< std::string >::iterator it_p = final_ret.begin() ; it_p < final_ret.end() ; it_p++, idx++ )
    {
        std::cerr << "Reading: " << out_path + *it_p << std::endl;
        std::vector< sparseVec > cur_data;
        int cur_fea_dim = readSparseMat(out_path + *it_p, cur_data);
        if( fea_dim > 0 && cur_fea_dim != fea_dim )
        {
            std::cerr << "cur_fea_dim != fea_dim" << std::endl;
            exit(0);
        }
        else if( fea_dim < 0 )
            fea_dim = cur_fea_dim;
            
        if( idx % 2 == 1 )
            test_set.insert(test_set.end(), cur_data.begin(), cur_data.end());
        else
            train_set.insert(train_set.end(), cur_data.begin(), cur_data.end());
    }
    std::cerr << "Train-Test: "<< train_set.size() << " " << test_set.size() << std::endl;
    
    saveCvMatSparse(out_path + "train_0_L0.smat", train_set, fea_dim);
    saveCvMatSparse(out_path + "test_0_L0.smat", test_set, fea_dim);

    return 1;
} 
//*/

//*
size_t readSparseMat(std::string filename, std::vector<sparseVec> &fea_vec)
{
    std::ifstream in(filename.c_str(), std::ios::in|std::ios::binary);
    if (in.is_open()==false)
       return 0;
    
    size_t size_t_len = sizeof(size_t);
    size_t float_len = sizeof(float);
    
    size_t cols;
    size_t rows;
    size_t num;
    // Read header
    in.read((char*)&cols, size_t_len);
    in.read((char*)&rows, size_t_len);
    in.read((char*)&num, size_t_len);
    
    size_t fea_len = cols;
    size_t data_num = rows;
    
    fea_vec.clear();
    fea_vec.resize(data_num);
    for( size_t i = 0 ; i < num; i++ )
    {
        size_t idx;
        float val;
        in.read((char*)&idx, size_t_len);
        in.read((char*)&val, float_len);
        size_t r = idx / fea_len;
        size_t c = idx % fea_len;
                    
        feature_node cur_node;
        cur_node.index = c;
        cur_node.value = val;
        fea_vec[r].push_back(cur_node);
    }
    
    return fea_len;
}



