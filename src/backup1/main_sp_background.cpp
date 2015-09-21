#include <opencv2/core/core.hpp>
#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

size_t readSparseMat(std::string filename, std::vector<sparseVec> &fea_vec);

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/background/");
    std::string out_path("../../data_pool/sp_hie_new/background/");
    std::string dict_path("BB_new_dict/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    float radius = 0.03;
    float down_ss = 0.005;
//    float box_size = 0.03;
    int box_num = 300;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    pcl::console::parse_argument(argc, argv, "--nn", box_num);
//    pcl::console::parse_argument(argc, argv, "--box", box_size);
    
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
    
    std::vector< std::pair<int, int> > bound_vec;
    bound_vec.push_back(std::pair<int, int> (50, 250));
    bound_vec.push_back(std::pair<int, int> (250, 500));
    bound_vec.push_back(std::pair<int, int> (500, 1000000));
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( size_t i = 0 ; i < file_num ; i++ )
    {
        double t1, t2;
        
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        pcl::VoxelGrid<PointT> sor;
        pcl::ExtractIndices<PointT> ext;
        ext.setNegative(false);
        
        std::cerr << "Loading-" << pcd_files[i] << std::endl;
            
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(normal_files[i], *cloud_normal);
        
        if( debug_flag )
        {
            cv::Mat rgb_frame = getImage(cloud, 1.0);
            cv::imshow("Show", rgb_frame);
            cv::waitKey();
        }
        
        MulInfoT tmp_data = convertPCD(cloud, cloud_normal);
        sor.setInputCloud(cloud);
        sor.setLeafSize(down_ss, down_ss, down_ss);
        sor.filter(*tmp_data.down_cloud);
        if( tmp_data.cloud->size() == tmp_data.down_cloud->size() )
        {
            std::cerr << "Downsampling Data Failed!" << std::endl;
            continue;
        }

        PreCloud(tmp_data, -1, false);
        std::vector<cv::Mat> main_fea = hie_producer_3.getHierFea(tmp_data, 0);
        
        for( int t = 0 ; t <= 2 ; t++ )
        {
            std::string cur_out_path;
            switch(t)
            {
                case 0:
                    cur_out_path = out_path + "low/";
                    break;
                case 1:
                    cur_out_path = out_path + "med/";
                    break;
                case 2:
                    cur_out_path = out_path + "high/";
                    break;
                default:exit(0);
            }
            
            boost::filesystem::create_directories(cur_out_path);
        
            std::vector<pcl::PointIndices::Ptr> inlier_set = CropSegs(tmp_data, bound_vec[t].first, bound_vec[t].second, box_num);
            std::vector< sparseVec> final_train_fea;
            int final_train_fea_dim = -1;
            for(std::vector<pcl::PointIndices::Ptr>::iterator it_inlier = inlier_set.begin() ; it_inlier < inlier_set.end() ; it_inlier++ )
            {
                pcl::PointCloud<PointT>::Ptr cur_seg(new pcl::PointCloud<PointT>());
                // Extract the inliers
                ext.setInputCloud (tmp_data.down_cloud);
                ext.setIndices (*it_inlier);
                ext.filter (*cur_seg);

                MulInfoT cur_data = convertPCD(tmp_data.cloud, tmp_data.cloud_normals);
                cur_data.down_cloud = cur_seg;

                if( debug_flag )
                {
                    cv::Mat img = getImage(cur_seg);
                    cv::imshow("Show", img);
                    cv::waitKey(0);
                }

                PreCloud(cur_data, -1, false);
                std::vector<cv::Mat> cur_fea = extFea(main_fea, (*it_inlier)->indices);
                
                cv::Mat final_temp = multiPool(pooler_set, cur_data, cur_fea);
                
                final_train_fea_dim = final_temp.cols;
                std::vector< sparseVec> final_sparse;
                sparseCvMat(final_temp, final_sparse);
                final_train_fea.push_back(final_sparse[0]);
            }
            
            saveCvMatSparse(cur_out_path + "/background_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
            final_train_fea.clear();
        }
        t2 = get_wall_time();
        std::cerr << "Time Elapsed: " << t2 - t1 << std::endl;
    }
    
    // Final Combining Background Model
    // Loading Background Model
    for( int t = 0 ; t <= 2 ; t++ )
    {
        std::string cur_out_path;
        switch(t)
        {
            case 0:
                cur_out_path = out_path + "low/";
                break;
            case 1:
                cur_out_path = out_path + "med/";
                break;
            case 2:
                cur_out_path = out_path + "high/";
                break;
            default:exit(0);
        }
        
        boost::filesystem::path op(cur_out_path);
        std::vector< std::string > final_ret;
        find_files(op, ".smat", final_ret);

        std::vector< sparseVec> train_set, test_set;

        int idx = 0;
        int fea_dim = -1;
        for( std::vector< std::string >::iterator it_p = final_ret.begin() ; it_p < final_ret.end() ; it_p++, idx++ )
        {
            std::cerr << "Reading: " << cur_out_path + *it_p << std::endl;
            std::vector< sparseVec > cur_data;
            int cur_fea_dim = readSparseMat(cur_out_path + *it_p, cur_data);
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

        saveCvMatSparse(cur_out_path + "train_0_L0.smat", train_set, fea_dim);
        saveCvMatSparse(cur_out_path + "test_0_L0.smat", test_set, fea_dim);
    }

    return 1;
} 

/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/background/");
    std::string out_path("../data_pool/ht2_background/");
    std::string dict_path("BB_new_dict/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    float radius = 0.03;
    float down_ss = 0.005;
//    float box_size = 0.03;
    int box_num = 50;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    pcl::console::parse_argument(argc, argv, "--nn", box_num);
//    pcl::console::parse_argument(argc, argv, "--box", box_size);
    
    float in_box_ratio = 0.5;
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
    
    pcl::VoxelGrid<PointT> sor;
    for( size_t i = 0 ; i < file_num ; i++ )
    {
        double t1, t2;
        
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        pcl::ExtractIndices<PointT> ext;
        ext.setNegative(false);
        
        std::cerr << "Loading-" << pcd_files[i] << std::endl;
            
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(normal_files[i], *cloud_normal);
        
        if( debug_flag )
        {
            cv::Mat rgb_frame = getImage(cloud, 1.0);
            cv::imshow("Show", rgb_frame);
            cv::waitKey();
        }
        
        MulInfoT tmp_data = convertPCD(cloud, cloud_normal);
        sor.setInputCloud(cloud);
        sor.setLeafSize(down_ss, down_ss, down_ss);
        sor.filter(*tmp_data.down_cloud);
        if( tmp_data.cloud->size() == tmp_data.down_cloud->size() )
        {
            std::cerr << "Downsampling Data Failed!" << std::endl;
            continue;
        }

        PreCloud(tmp_data, -1, false);
        std::vector<cv::Mat> main_fea = hie_producer_3.getHierFea(tmp_data, 0);
//        tmp_data.down_cloud = tmp_data.cloud;
        
        for( float box_size = 0.02; box_size <= 0.08; box_size += 0.01 )
        {
            std::cerr << "BOX: " << box_size << std::endl;
            std::stringstream bb;
            bb << (int)(box_size * 100);
            
            std::string cur_out_path(out_path + "00" + bb.str() + "/");
            boost::filesystem::create_directories(cur_out_path);
        
            std::vector< std::pair<float, float> > cur_box(1);
            cur_box[0].first = box_size; cur_box[0].second = box_size;
            std::vector<pcl::PointIndices::Ptr> inlier_set = CropSegs(tmp_data, cur_box, box_num, in_box_ratio);
    //        std::vector<pcl::PointIndices::Ptr> inlier_set = hardSampling(tmp_data, in_box_ratio);

            std::vector< std::vector<cv::Mat> > main_fea_set;
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

                MulInfoT cur_data = convertPCD(tmp_data.cloud, tmp_data.cloud_normals);
                cur_data.down_cloud = cur_seg;

                if( debug_flag )
                {
                    cv::Mat img = getImage(cur_seg);
                    cv::imshow("Show", img);
                    cv::waitKey(0);
                }

                PreCloud(cur_data, -1, false);
                new_set[0].push_back(cur_data);

                std::vector<cv::Mat> cur_fea = extFea(main_fea, (*it_inlier)->indices);
                main_fea_set.push_back(cur_fea);
            }

            size_t data_num = new_set[0].size();

            // Extracting the features
            std::vector< sparseVec> final_train_fea(data_num);
            int final_train_fea_dim = -1;
            #pragma omp parallel for schedule(dynamic, 1)
            for( size_t j = 0 ; j < data_num ; j++ )
            {
                MulInfoT *inst_ptr = &new_set[0][j];
    //            std::vector<cv::Mat> local_fea = hie_producer_3.getHierFea(*inst_ptr, 0);
                cv::Mat final_temp = multiPool(pooler_set, *inst_ptr, main_fea_set[j]);
                main_fea_set[j].clear();

                final_train_fea_dim = final_temp.cols;
                std::vector< sparseVec> final_sparse;
                sparseCvMat(final_temp, final_sparse);
                final_train_fea[j] = final_sparse[0];
            }

            saveCvMatSparse(cur_out_path + "/background_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
            final_train_fea.clear();
            new_set.clear();
        }
        t2 = get_wall_time();
        std::cerr << "Time Elapsed: " << t2 - t1 << std::endl;
    }
    
    // Final Combining Background Model
    // Loading Background Model
    for( float box_size = 0.02; box_size <= 0.08; box_size += 0.01 )
    {
        std::cerr << "BOX: " << box_size << std::endl;
        std::stringstream bb;
        bb << (int)(box_size * 100);
        
        std::string cur_out_path(out_path + "00" + bb.str() + "/");
        
        boost::filesystem::path op(cur_out_path);
        std::vector< std::string > final_ret;
        find_files(op, ".smat", final_ret);

        std::vector< sparseVec> train_set, test_set;

        int idx = 0;
        int fea_dim = -1;
        for( std::vector< std::string >::iterator it_p = final_ret.begin() ; it_p < final_ret.end() ; it_p++, idx++ )
        {
            std::cerr << "Reading: " << cur_out_path + *it_p << std::endl;
            std::vector< sparseVec > cur_data;
            int cur_fea_dim = readSparseMat(cur_out_path + *it_p, cur_data);
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

        saveCvMatSparse(cur_out_path + "train_0_L0.smat", train_set, fea_dim);
        saveCvMatSparse(cur_out_path + "test_0_L0.smat", test_set, fea_dim);
    }

    return 1;
} 
//*/

/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/background/");
    std::string out_path("../data_pool/ht2_background/");
    
    std::string dict_path("BB_new_dict/");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    float radius = 0.03;
    float down_ss = 0.005;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
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
    
    pcl::VoxelGrid<PointT> sor;
    for( size_t i = 0 ; i < file_num ; i++ )
    {
        std::stringstream ss;
        ss << i;
    
        pcl::ExtractIndices<PointT> ext;
        ext.setNegative(false);
        
        std::cerr << "Loading-" << pcd_files[i] << std::endl;
            
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        
        pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(normal_files[i], *cloud_normal);
        
        sor.setInputCloud(cloud);
        sor.setLeafSize(down_ss, down_ss, down_ss);
        pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
        sor.filter(*down_cloud);
        
//        MulInfoT tmp_data = convertPCD(cloud, cloud_normal);
//        tmp_data.down_cloud = tmp_data.cloud;
//        tmp_data.down_cloud = down_cloud;
        
        std::vector<pcl::PointCloud<PointT>::Ptr> segs_0;
        std::multimap<uint32_t, uint32_t> graph;
        SPCloud(down_cloud, segs_0, graph);
        
        std::cerr << "Down Size: "<< down_cloud->size() << std::endl;
        std::cerr << "SP Num: " << segs_0.size() << std::endl;
        size_t data_num = segs_0.size();
        
        // Extracting the features
        std::vector< sparseVec> final_train_fea(data_num);
        int final_train_fea_dim = -1;
        #pragma omp parallel for schedule(dynamic, 1)
        for( size_t j = 0 ; j < data_num ; j++ )
        {
            if( segs_0[j]->size() <= 100 )
                continue;
            
            MulInfoT cur_data = convertPCD(cloud, cloud_normal);
            cur_data.down_cloud = segs_0[j];
            
            PreCloud(cur_data, -1, false);
            
            std::vector<cv::Mat> local_fea = hie_producer_3.getHierFea(cur_data, 0);
            cv::Mat final_temp = multiPool(pooler_set, cur_data, local_fea);
            local_fea.clear();
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
        }
        saveCvMatSparse(out_path + "spBackground_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
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


//            for( int p = 0 ; p < local_fea[0].rows ; p++ )
//            {
//                for( int q = 0 ; q < local_fea[0].cols; q++ )
//                {
//                    if( main_fea_set[j][0].at<float>(p, q) != local_fea[0].at<float>(p, q) )
//                        std::cerr << "Error!" << std::endl;
//                }
//            }
//            std::cin.get();
