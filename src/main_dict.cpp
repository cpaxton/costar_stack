#include <opencv2/core/core.hpp>

#include "../include/features.h"
#include "../include/WKmeans.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

struct CLUSTER_R
{
    sparseK cluster;
    float r;
};

cv::Mat shot_cloud_uni(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normals, const pcl::PointCloud<pcl::ReferenceFrame>::Ptr lrf, float radius, int snum)
{
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
    //pcl::PointCloud<pcl::ReferenceFrame>::Ptr down_lrf(new pcl::PointCloud<pcl::ReferenceFrame>());
    
    if( cloud->size() < snum )
    {
        down_cloud = cloud;
        //down_lrf = lrf;
    }
    else
    {
        std::vector<size_t> rand_idx;
        GenRandSeq(rand_idx, cloud->size());
        rand_idx.erase(rand_idx.begin()+snum, rand_idx.end());
        
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        inliers->indices.resize(rand_idx.size());
        for( size_t i = 0 ; i < rand_idx.size() ; i++ )
            inliers->indices[i] = rand_idx[i];
        
        // Create the filtering object
        pcl::ExtractIndices<PointT> extract;
         // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*down_cloud);
        
        // Create the filtering object
        //pcl::ExtractIndices<pcl::ReferenceFrame> extract_lrf;
         // Extract the inliers
        //extract_lrf.setInputCloud (lrf);
        //extract_lrf.setIndices (inliers);
        //extract_lrf.setNegative (false);
        //extract_lrf.filter (*down_lrf);
    }
    
    pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

    // SHOT estimation object.
    pcl::SHOTEstimationOMP<PointT, pcl::Normal, pcl::SHOT352> shot;
    //shot.setNumberOfThreads(16);
    shot.setInputCloud(down_cloud);
    //shot.setLRFRadius(0.05);
    //shot.setInputReferenceFrames(down_lrf);
    
    shot.setSearchSurface(cloud);
    shot.setInputNormals(cloud_normals);
    // The radius that defines which of the keypoint's neighbors are described.
    // If too large, there may be clutter, and if too small, not enough points may be found.
    shot.setRadiusSearch(radius);

    shot.compute(*descriptors);
    
    std::vector<cv::Mat> fea_pool;
    //cv::Mat fea = cv::Mat::zeros(descriptors->size(), 352, CV_32FC1);
    for( size_t i = 0 ; i < descriptors->size() ; i++ )
    {
        //float *ptr = (float *)fea.row(i).data;
        cv::Mat cur_fea = cv::Mat::zeros(1, 352, CV_32FC1);
        float temp = descriptors->at(i).descriptor[0];  // check whether descriptor is valid
        if( temp == temp )
        {
            memcpy(cur_fea.data, descriptors->at(i).descriptor, sizeof(float)*352);
            fea_pool.push_back(cur_fea);
        }   
    }
    cv::Mat fea;
    cv::vconcat(fea_pool, fea);
    return fea;
}

void getJointPoolIdx(const cv::Mat domain, std::vector<size_t> &count, float scale = 7)
{
    int len = 1;
    float scale_len = 1.000001 / scale;
    for( int i = 0 ; i < domain.cols ; i++ )
        len *= scale;
    if( count.empty() == true )
        count.resize(len);
    
    for( int i = 0 ; i < domain.rows ; i++ )
    {
        int idx = 0;
        for( int j = 0 ; j < domain.cols ; j++ )
            idx = floor(domain.at<float>(i, j)/scale_len) + idx * scale;
        count[idx]=1;
    }
}

//int main(int argc, char** argv)
//{
//    int c1 = 0, c2 = 9;
//    
//    float radius = 0.02;
//    int maxnum = 50;
//    float size = 32;
//    std::string path("/home/chi/JHUIT/ht10/");
//    std::string out_path("JHU_densesift_dict");
//    
//    pcl::console::parse_argument(argc, argv, "--p", path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    pcl::console::parse_argument(argc, argv, "--m", maxnum);
//    pcl::console::parse_argument(argc, argv, "--r", radius);
//    pcl::console::parse_argument(argc, argv, "--size", size);
//    
//    boost::filesystem::create_directories(out_path);
//    
//    std::vector<cv::Mat> sift_set;
//    for( int b1 = c1 ; b1 <= c2 ; b1++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readJHUInstWithImg(path, train_objects, test_objects, b1, b1, true);
//        test_objects.clear();
//        
//        int num = train_objects[0].size();
//        #pragma omp parallel for schedule(dynamic, 1)
//        for( int j = 0 ; j < num ; j++ )
//        {
//            pcl::PointCloud<PointT>::Ptr cloud = train_objects[0][j].cloud;
//            cv::Mat cur_rgb = train_objects[0][j].img;
//            cv::Mat cur_map2d = train_objects[0][j].map2d;
//            cv::Mat cur_gray(cur_rgb.size(), CV_8UC1);
//            cv::cvtColor(cur_rgb, cur_gray, CV_BGR2GRAY);
//
//            cv::Mat map3d_2d = train_objects[0][j]._3d2d;
//            std::vector<size_t> rand_idx;
//            GenRandSeq(rand_idx, cloud->size());
//            
//            int tmp_num = (int)cloud->size();
//            int cur_num = tmp_num < maxnum ? tmp_num : maxnum;
//            
//            cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
//            // Compute keypoints and descriptor from the source image in advance
//            std::vector<cv::KeyPoint> sift_keys(cur_num);
//
//            for(int k = 0 ; k < cur_num ; k++ )
//            {
//                sift_keys[k].pt.x = map3d_2d.at<int>(rand_idx[k], 0);
//                sift_keys[k].pt.y = map3d_2d.at<int>(rand_idx[k], 1);
//                sift_keys[k].size = size;
//                sift_keys[k].angle = 0;
//            }
//            cv::Mat sift_descr;
//            sift_ext->compute(cur_gray, sift_keys, sift_descr);
//            for(int k = 0 ; k < sift_descr.rows ; k++ )
//                cv::normalize(sift_descr.row(k), sift_descr.row(k));          
//            
//            #pragma omp critical
//            {
//                sift_set.push_back(sift_descr);
//            }
//        }
//    }
//    
//    std::cerr << "Start Kmeans...";
//    cv::Mat final_sift;
//    cv::vconcat(sift_set, final_sift);
//    std::cerr << final_sift.rows << std::endl;
//    sift_set.clear();
//    
//    int KK[5] = {25, 50, 100, 200, 400};
//    for( int i = 0 ; i < 5 ; i++ )
//    {
//        std::cerr << "Clustering "<< KK[i] << std::endl;
//        cv::Mat center_sift, labels;
//        cv::kmeans(final_sift, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_sift );
//        
//        std::stringstream ss, ll;
//        ss << KK[i];
//
//        saveMat(out_path + "/dict_sift_L0_"+ss.str()+".cvmat", center_sift);
//    }
//    return 1;
//}


std::vector<cv::KeyPoint> extSIFTKeys(const cv::Mat &cur_gray, const std::vector<cv::SiftFeatureDetector*> &sift_det_vec)
{
    cv::Mat cur_response = cv::Mat::ones(cur_gray.size(), CV_32FC1)*-1;
    std::vector< std::pair<int, int> > active_idx;
    cv::Mat cur_idx = cv::Mat::ones(cur_gray.size(), CV_32SC1)*-1;
    
    std::vector<cv::KeyPoint> sift_keys;
    for( size_t i = 0 ; i < sift_det_vec.size() ; i++ )
    {
        std::vector<cv::KeyPoint> tmp_keys;
        sift_det_vec[i]->detect(cur_gray, tmp_keys);
        //std::cerr << i+1 << " " << tmp_keys.size() << std::endl;
        sift_keys.insert(sift_keys.end(), tmp_keys.begin(), tmp_keys.end());
    }  
//    std::cin.get();
    int count = 0;
    for(std::vector<cv::KeyPoint>::iterator it = sift_keys.begin() ; it < sift_keys.end() ; it++, count++ )
    {
        int row = round(it->pt.y);
        int col = round(it->pt.x);
        if( it->response > cur_response.at<float>(row, col) )
        {
            cur_response.at<float>(row, col) = it->response;
            if( cur_idx.at<int>(row, col) < 0 )
                active_idx.push_back(std::pair<int, int> (row, col));
            cur_idx.at<int>(row, col) = count;
        }
    }
    std::vector<cv::KeyPoint> final_keys(active_idx.size());
    count = 0;
    for(std::vector< std::pair<int, int> >::iterator it = active_idx.begin() ; it < active_idx.end() ; it++, count++ )
        final_keys[count] = sift_keys[cur_idx.at<int>(it->first, it->second)];
    
    return final_keys;
}

//int main(int argc, char** argv)
//{
//    int c1 = 0, c2 = 9;
//    
////    float radius = 0.02;
////    int maxnum = 25;
//    float sigma = 1.6;
//    std::string path("/home/chi/JHUIT/filtered_pcd/");
//    std::string out_path("JHU_sift_0916");
//    
//    pcl::console::parse_argument(argc, argv, "--p", path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
////    pcl::console::parse_argument(argc, argv, "--m", maxnum);
////    pcl::console::parse_argument(argc, argv, "--r", radius);
//    pcl::console::parse_argument(argc, argv, "--sigma", sigma);
//    
//    boost::filesystem::create_directories(out_path);
//    
//    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
//    for( float sigma = 0.9 ; sigma <= 1.6 ; sigma += 0.1 )
//    {	
//        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
//        	0, // nFeatures
//        	4, // nOctaveLayers
//        	-10000, // contrastThreshold 
//        	100000, //edgeThreshold
//        	sigma//sigma
//        	);
//        sift_det_vec.push_back(sift_det);	
//    }
//    
//    std::vector<cv::Mat> sift_set;
//    for( int b1 = c1 ; b1 <= c2 ; b1++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readJHUInstWithImg(path, train_objects, test_objects, b1, b1);
//        test_objects.clear();
//        
//        int num = train_objects[0].size();
//        #pragma omp parallel for schedule(dynamic, 1)
//        for( int j = 0 ; j < num ; j++ )
//        {
//            pcl::PointCloud<PointT>::Ptr cloud = train_objects[0][j].cloud;
//            cv::Mat cur_rgb = train_objects[0][j].img;
//            cv::Mat cur_map2d = train_objects[0][j].map2d;
//            cv::Mat cur_gray(cur_rgb.size(), CV_8UC1);
//            cv::cvtColor(cur_rgb, cur_gray, CV_BGR2GRAY);
//
////            cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
////                    0, // nFeatures
////                    4, // nOctaveLayers
////                    -10000, // contrastThreshold 
////                    100000, //edgeThreshold
////                    sigma//sigma
////                    );
//            cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
////            // Compute keypoints and descriptor from the source image in advance
////            std::vector<cv::KeyPoint> sift_keys;
////
////            sift_det->detect(cur_gray, sift_keys);
//            
//            std::vector<cv::KeyPoint> sift_keys = extSIFTKeys(cur_gray, sift_det_vec);
//            
//            for(size_t k = 0 ; k < sift_keys.size() ; )
//            {
//                int tmp_idx = cur_map2d.at<int>(round(sift_keys[k].pt.y), round(sift_keys[k].pt.x));
//                if( tmp_idx < 0 )
//                    sift_keys.erase(sift_keys.begin() + k);
//                else
//                    k++;
//                    
//            }
//            cv::Mat sift_descr;
//            sift_ext->compute(cur_gray, sift_keys, sift_descr);
//            for(int k = 0 ; k < sift_descr.rows ; k++ )
//                cv::normalize(sift_descr.row(k), sift_descr.row(k));          
//            
//            #pragma omp critical
//            {
//                sift_set.push_back(sift_descr);
//            }
//        }
//    }
//    
//    std::cerr << "Start Kmeans...";
//    cv::Mat final_sift;
//    cv::vconcat(sift_set, final_sift);
//    std::cerr << final_sift.rows << std::endl;
//    sift_set.clear();
//    
//    int KK[5] = {25, 50, 100, 200, 400};
//    for( int i = 0 ; i < 5 ; i++ )
//    {
//        std::cerr << "Clustering "<< KK[i] << std::endl;
//        cv::Mat center_sift, labels;
//        cv::kmeans(final_sift, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_sift );
//        
//        std::stringstream ss, ll;
//        ss << KK[i];
//
//        saveMat(out_path + "/dict_sift_L0_"+ss.str()+".cvmat", center_sift);
//    }
//    return 1;
//}

int main(int argc, char** argv)
{
    int c1 = 0, c2 = UW_INST_MAX;
    
    std::string path("/home/chi/UW_RGBD/rgbd-dataset/");
    std::string out_path("UW_new_sift_dict");
    int maxnum = 30;
    pcl::console::parse_argument(argc, argv, "--m", maxnum);
    
    pcl::console::parse_argument(argc, argv, "--p", path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    boost::filesystem::create_directories(out_path);
    
    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
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
    std::ifstream model_list((path + "models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    sparseK cluster_sift;
    int count = 0;
//    std::vector<cv::Mat> sift_set;
    for( int b1 = 0 ; b1 < UW_INST_MAX ; b1++ )
    {
        std::string class_name;
        model_list >> class_name;
        
        if( b1 > c2 || b1 < c1)
            continue;
        
        std::cerr << "Loading-"<< b1 <<": "<< class_name << std::endl;
        
        std::string workspace(path + class_name + "/");
        std::vector<std::string> rgb_files;
        
        find_files(workspace, "_crop.png", rgb_files);
        size_t file_num = rgb_files.size();
        
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < file_num ; j++ )
        {
            std::string rfile(workspace+rgb_files[j]);
            
            cv::Mat rgb = cv::imread(rfile);
            
            cv::Mat cur_gray(rgb.size(), CV_8UC1);
            cv::cvtColor(rgb, cur_gray, CV_BGR2GRAY);

            cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
            std::vector<cv::KeyPoint> sift_keys = extSIFTKeys(cur_gray, sift_det_vec);
            std::vector<cv::KeyPoint> new_sift_keys;
            
            if( sift_keys.size() > maxnum )
            {
                std::vector<size_t> rand_idx;
                GenRandSeq(rand_idx, sift_keys.size());
                for( int k = 0 ; k < maxnum ; k++ )
                    new_sift_keys.push_back(sift_keys[rand_idx[k]]);
            }
            else
                new_sift_keys = sift_keys;
            
            cv::Mat sift_descr;
            sift_ext->compute(cur_gray, new_sift_keys, sift_descr);
            for(int k = 0 ; k < sift_descr.rows ; k++ )
                cv::normalize(sift_descr.row(k), sift_descr.row(k));          
            
            #pragma omp critical
            {
//                sift_set.push_back(sift_descr);
                count++;
                if( count % 100 == 0 )
                    std::cerr << count << " ";
                
                int len = sift_descr.rows;
                for( int p = 0 ; p < len ; p++)
                    cluster_sift.AddData(sift_descr.row(p));
            }
        }
    }
    
//    std::cerr << "Start Kmeans...";
//    cv::Mat final_sift;
//    cv::vconcat(sift_set, final_sift);
//    std::cerr << final_sift.rows << std::endl;
//    sift_set.clear();
    
    int KK[5] = {25, 50, 100, 200, 400};
    for( int i = 0 ; i < 5 ; i++ )
    {
//        std::cerr << "Clustering "<< KK[i] << std::endl;
//        cv::Mat center_sift, labels;
//        cv::kmeans(final_sift, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_sift);
        std::cerr << "Clustering "<< KK[i] << std::endl;
        cv::Mat center_sift;
        HierKmeans(cluster_sift.proto_set, center_sift, KK[i]);
        
        std::stringstream ss, ll;
        ss << KK[i];

        saveMat(out_path + "/dict_sift_L0_"+ss.str()+".cvmat", center_sift);
    }
    return 1;
}


//int main(int argc, char** argv)
//{
//    int c1 = 0, c2 = 299;
//    
//    float radius = 0.02;
//    int maxnum = 25;
//    std::string path("/home/chi/UW_RGBD/filtered_pcd/");
//    std::string out_path("UW_fpfh_dict/");
//    boost::filesystem::create_directories(out_path);
//    
//    pcl::console::parse_argument(argc, argv, "--p", path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    pcl::console::parse_argument(argc, argv, "--m", maxnum);
//    pcl::console::parse_argument(argc, argv, "--r", radius);
//    
////    cv::Mat tmp;
////    readMat(out_path+"dict_color_L0_200.cvmat", tmp);
////    for(int i = 0 ; i < tmp.rows ; i++ )
////    {
////        std::cerr << tmp.row(i) << std::endl;
////        std::cerr << tmp.row(i).cols << " " << tmp.rows <<" " << cv::norm(tmp.row(i), cv::NORM_L2) << std::endl;
////        std::cin.get();
////    }
////    return 1;
////*
//    
//    int layer = -1;
//
//    int count = 0;
////    std::vector<cv::Mat> fpfh_set;
//    sparseK cluster_fpfh;
//    
//    for( int b1 = c1 ; b1 <= c2 ; b1++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readUWInst(path, train_objects, test_objects, b1, b1, 2);
//        test_objects.clear();
//        
//        int num = train_objects[0].size();
//        #pragma omp parallel for schedule(dynamic, 1)
//        for( int j = 0 ; j < num ; j++ )
//        {
////            std::cerr << j << std::endl;
//            pcl::PointCloud<PointT>::Ptr cloud = train_objects[0][j].cloud;
//            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
//            computeNormals(cloud, cloud_normals, radius);
//   
//            pcl::PointCloud<PointT>::Ptr down_cloud (new pcl::PointCloud<PointT>());
//            pcl::VoxelGrid<PointT> sor;
//            sor.setInputCloud(cloud);
//            sor.setLeafSize(0.005, 0.005, 0.005);
//            sor.filter(*down_cloud);
//            
//            pcl::PointCloud<PointT>::Ptr random_keys (new pcl::PointCloud<PointT>());
//            
//            if( maxnum > (int)down_cloud->size() )
//                random_keys = down_cloud;
//            else
//            {
//                std::vector<size_t> rand_idx;
//                GenRandSeq(rand_idx, down_cloud->size());
//                for( int i = 0 ; i < maxnum ; i++ )
//                    random_keys->push_back(down_cloud->at(rand_idx[i]));
//            }
////            std::cerr << random_keys->size() << std::endl;
//            cv::Mat fpfh = fpfh_cloud(cloud, random_keys, cloud_normals, radius, true);
//            
//            
//            #pragma omp critical
//            {
////                fpfh_set.push_back(fpfh);
//                count++;
//                if( count % 100 == 0 )
//                    std::cerr << count << " ";
//                
//                int len = fpfh.rows;
//                for( int p = 0 ; p < len ; p++)
//                    cluster_fpfh.AddData(fpfh.row(p));
//                    
//            }
//        }
//    }
//    
//    std::cerr << "Start Kmeans..." << std::endl;
//    
//    int KK[5] = {25, 50, 100, 200, 400};
//    for( int i = 0 ; i < 5 ; i++ )
//    {
//        std::cerr << "Clustering "<< KK[i] << std::endl;
//        cv::Mat centers;
//        HierKmeans(cluster_fpfh.proto_set, centers, KK[i]);
//        
//        std::stringstream ss;
//        ss << KK[i];
//        saveMat(out_path + "dict_fpfh_L0_"+ss.str()+".cvmat", centers);
//    }
//    
//    
////    cv::Mat final_fpfh;
////    cv::vconcat(fpfh_set, final_fpfh);
////    fpfh_set.clear();
////    
////    int KK[5] = {25, 50, 100, 200, 400};
////    for( int i = 0 ; i < 5 ; i++ )
////    {
////        std::cerr << "Clustering "<< KK[i] << std::endl;
////        cv::Mat center_depth, center_color, labels, center_joint, center_fpfh;
////        cv::kmeans(final_fpfh, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_fpfh );
////        
////        std::stringstream ss, ll;
////        ss << KK[i];
////        saveMat(out_path + "dict_fpfh_L0_"+ss.str()+".cvmat", center_fpfh);
////    }
//
//    return 1;
//}


//int main(int argc, char** argv)
//{
//    int c1 = 0, c2 = 9;
////    int KK = 200;
//    
//    float radius = 0.02;
//    float ratio = 0.1;
//    float down_ss = 0.005;
//    int maxnum = 70;
//    std::string path("/home/chi/JHUIT/ht10/");
//    std::string out_path("JHU_fpfh_dict/");
//    
//    cv::Mat tmp;
//    readMat(out_path+"dict_fpfh_L0_200.cvmat", tmp);
//    for(int i = 0 ; i < tmp.rows ; i++ )
//    {
//        std::cerr << tmp.row(i) << std::endl;
//        std::cerr << tmp.row(i).cols << " " << tmp.rows <<" " << cv::norm(tmp.row(i), cv::NORM_L2) << std::endl;
//        std::cin.get();
//    }
//    
////    pcl::console::parse_argument(argc, argv, "--K", KK);
//    pcl::console::parse_argument(argc, argv, "--p", path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    pcl::console::parse_argument(argc, argv, "--m", maxnum);
//    pcl::console::parse_argument(argc, argv, "--r", radius);
//    pcl::console::parse_argument(argc, argv, "--rt", ratio);
//    
//    boost::filesystem::create_directories(out_path);
//    
//    int layer = -1;
////    pcl::console::parse_argument(argc, argv, "--l", layer);
//     
////    Hier_Pooler hie_producer(radius);
////    hie_producer.setRatio(ratio);
////    hie_producer.LoadDict_L0(out_path, "200", "200");
//    
//    
////    std::vector<cv::Mat> depth_set;
////    std::vector<cv::Mat> color_set;
////    std::vector<cv::Mat> joint_set;
//    std::vector<cv::Mat> fpfh_set;
//    for( int b1 = c1 ; b1 <= c2 ; b1++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readJHUInst(path, train_objects, test_objects, b1, b1, true);
//        test_objects.clear();
//        
//        int num = train_objects[0].size();
//        #pragma omp parallel for schedule(dynamic, 1)
//        for( int j = 0 ; j < num ; j++ )
//        {
//            pcl::PointCloud<PointT>::Ptr cloud = train_objects[0][j].cloud;
//            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
//            computeNormals(cloud, cloud_normals, radius);
//   
//            pcl::PointCloud<PointT>::Ptr down_cloud (new pcl::PointCloud<PointT>());
//            pcl::VoxelGrid<PointT> sor;
//            sor.setInputCloud(cloud);
//            sor.setLeafSize(down_ss, down_ss, down_ss);
//            sor.filter(*down_cloud);
//            
//            pcl::PointCloud<PointT>::Ptr random_keys (new pcl::PointCloud<PointT>());
//            
//            if( maxnum > (int)down_cloud->size() )
//                random_keys = down_cloud;
//            else
//            {
//                std::vector<size_t> rand_idx;
//                GenRandSeq(rand_idx, down_cloud->size());
//                for( int i = 0 ; i < maxnum ; i++ )
//                    random_keys->push_back(down_cloud->at(rand_idx[i]));
//            }
//            cv::Mat fpfh = fpfh_cloud(cloud, random_keys, cloud_normals, radius, true);
//            
////            cv::Mat depth_raw_fea, color_raw_fea;
////            hie_producer.sampleRaw_L0(cloud, cloud_normals, depth_raw_fea, color_raw_fea, maxnum, radius);
////            std::vector<cv::Mat> fea_L0 = hie_producer.EncodeLayer_L0(depth_raw_fea, color_raw_fea);
////            
////            cv::Mat joint_raw;
////            cv::hconcat(fea_L0[0], fea_L0[1], joint_raw);
//            
////            MulInfoT cur_data = convertPCD(cloud, cloud_normals);
////            std::vector<cv::Mat> rawfea_L0 = hie_producer.getRawFea(cur_data, layer, maxnum);
//            
//            #pragma omp critical
//            {
////                depth_set.push_back(rawfea_L0[0]);
////                color_set.push_back(rawfea_L0[1]);
////                joint_set.push_back(joint_raw);
//                fpfh_set.push_back(fpfh);
//            }
//        }
//    }
//    
//    std::cerr << "Start Kmeans..." << std::endl;
////    cv::Mat final_depth, final_color;
////    cv::vconcat(depth_set, final_depth);
////    depth_set.clear();
////    cv::vconcat(color_set, final_color);
////    color_set.clear();
////    cv::Mat final_joint;
////    cv::vconcat(joint_set, final_joint);
////    joint_set.clear();
//    
//    cv::Mat final_fpfh;
//    cv::vconcat(fpfh_set, final_fpfh);
//    fpfh_set.clear();
//    
////    int KK[5] = {200, 400, 600, 800, 1000};
//    int KK[5] = {25, 50, 100, 200, 400};
//    for( int i = 0 ; i < 5 ; i++ )
//    {
//        std::cerr << "Clustering "<< KK[i] << std::endl;
//        cv::Mat center_depth, center_color, labels, center_joint, center_fpfh;
////        cv::kmeans(final_depth, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_depth );
////        cv::kmeans(final_color, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_color );
////        cv::kmeans(final_joint, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_joint );
//        cv::kmeans(final_fpfh, KK[i], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 500, 1e-6), 1, cv::KMEANS_PP_CENTERS, center_fpfh );
//        
//        std::stringstream ss, ll;
//        ss << KK[i];
////        ll << layer;
//
////        saveMat(out_path + "dict_depth_L0_"+ss.str()+".cvmat", center_depth);
////        saveMat(out_path + "dict_color_L0_"+ss.str()+".cvmat", center_color);
////        saveMat(out_path + "dict_joint_L0_"+ss.str()+".cvmat", center_joint);
//        saveMat(out_path + "dict_fpfh_L0_"+ss.str()+".cvmat", center_fpfh);
//    }
//    return 1;
//}

//int main(int argc, char** argv)
//{
//    int c1 = 0, c2 = 9;
//    int KK = 200;
//    
//    float radius = 0.02;
//    int maxnum = 25;
//    std::string path("/home/chi/UW_RGBD/filtered_pcd/");
//    std::string out_path("UW_shot_dict");
//    boost::filesystem::create_directories(out_path);
//    
//    pcl::console::parse_argument(argc, argv, "--K", KK);
//    pcl::console::parse_argument(argc, argv, "--p", path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    pcl::console::parse_argument(argc, argv, "--m", maxnum);
//    pcl::console::parse_argument(argc, argv, "--r", radius);
//    
////    pcl::console::parse_argument(argc, argv, "--c1", c1);
////    pcl::console::parse_argument(argc, argv, "--c2", c2);
//    
//    sparseK cluster_depth, cluster_color;
//    Hier_Pooler hie_producer(radius);
//    int count = 0;
//    for( int b1 = 0 ; b1 <= 299 ; b1++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readUWInstWithImg(path, train_objects, test_objects, b1, b1, 1);
//        test_objects.clear();
//        
//        int num = train_objects[0].size();
//        #pragma omp parallel for schedule(dynamic, 1)
//        for( int j = 0 ; j < num ; j++ )
//        {
//            pcl::PointCloud<PointT>::Ptr cloud = train_objects[0][j].cloud;
//            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
//            computeNormals(cloud, cloud_normals, radius);
//        
//            MulInfoT cur_data = convertPCD(cloud, cloud_normals);
//            
//            std::vector<cv::Mat> rawfea_L0 = hie_producer.getRawFea(cur_data, -1, maxnum);
//            #pragma omp critical
//            {
//                count++;
//                if( count % 100 == 0 )
//                    std::cerr << count << " ";
//                
//                int len = rawfea_L0[0].rows;
//                for( int p = 0 ; p < len ; p++)
//                {
//                    cluster_depth.AddData(rawfea_L0[0].row(p));
//                    cluster_color.AddData(rawfea_L0[1].row(p));
//                }
//            }
//        }
//    }
//    
//    std::cerr << "Start Kmeans..." << std::endl;
//    
//    cv::Mat center_depth, center_color;
//    HierKmeans(cluster_depth.proto_set, center_depth, KK);
//    HierKmeans(cluster_color.proto_set, center_color, KK);
//    
//    std::stringstream ss, ll;
//    ss << KK;
////    ll << layer;
//    
//    saveMat(out_path + "/dict_depth_L0_"+ss.str()+".cvmat", center_depth);
//    saveMat(out_path + "/dict_color_L0_"+ss.str()+".cvmat", center_color);
//    
//    return 1;
//}

//int main(int argc, char** argv)
//{
//    int c1 = 0, c2 = 9;
//    int KK = 200;
//    
//    float radius = 0.02;
//    int maxnum = 25;
//    std::string path("/home/chi/JHUIT/filtered_pcd/");
//    std::string out_path("JHU_dict");
//    boost::filesystem::create_directories(out_path);
//    
//    pcl::console::parse_argument(argc, argv, "--K", KK);
//    pcl::console::parse_argument(argc, argv, "--p", path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    pcl::console::parse_argument(argc, argv, "--m", maxnum);
//    pcl::console::parse_argument(argc, argv, "--r", radius);
//    
////    pcl::console::parse_argument(argc, argv, "--c1", c1);
////    pcl::console::parse_argument(argc, argv, "--c2", c2);
//    
//    int layer = -1;
////    pcl::console::parse_argument(argc, argv, "--l", layer);
//     
//    sparseK cluster_depth, cluster_color;
//    Hier_Pooler hie_producer(radius);
//    int count = 0;
//    for( int b1 = c1 ; b1 <= c2 ; b1++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readJHUInst(path, train_objects, test_objects, b1, b1);
//        test_objects.clear();
//        
//        int num = train_objects[0].size();
//        #pragma omp parallel for schedule(dynamic, 1)
//        for( int j = 0 ; j < num ; j++ )
//        {
//            pcl::PointCloud<PointT>::Ptr cloud = train_objects[0][j].cloud;
//            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
//            computeNormals(cloud, cloud_normals, radius);
//        
//            MulInfoT cur_data = convertPCD(cloud, cloud_normals);
//            
//            std::vector<cv::Mat> rawfea_L0 = hie_producer.getRawFea(cur_data, layer, maxnum);
//            #pragma omp critical
//            {
//                count++;
//                if( count % 100 == 0 )
//                    std::cerr << count << " ";
//                
//                int len = rawfea_L0[0].rows;
//                for( int p = 0 ; p < len ; p++)
//                {
//                    cluster_depth.AddData(rawfea_L0[0].row(p));
//                    cluster_color.AddData(rawfea_L0[1].row(p));
//                }
//            }
//        }
//    }
//    
//    std::cerr << "Start Kmeans..." << std::endl;
//    
//    cv::Mat center_depth, center_color;
//    HierKmeans(cluster_depth.proto_set, center_depth, KK);
//    HierKmeans(cluster_color.proto_set, center_color, KK);
//    
//    std::stringstream ss, ll;
//    ss << KK;
//    ll << layer;
//    
//    saveMat(out_path + "/dict_depth_L0_"+ss.str()+".cvmat", center_depth);
//    saveMat(out_path + "/dict_color_L0_"+ss.str()+".cvmat", center_color);
//    
//    return 1;
//}

/*
int main(int argc, char** argv)
{
    int c1 = 0, c2 = BB_INST_MAX-1;
    int KK = 100;
    
    float radius = 0.03;
    int maxnum = 25;
    std::string path("/home/chi/BigBIRD/processed");
    
    std::string out_path("BB_new_dict");
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    pcl::console::parse_argument(argc, argv, "--K", KK);
    pcl::console::parse_argument(argc, argv, "--p", path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--m", maxnum);
    pcl::console::parse_argument(argc, argv, "--r", radius);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    int layer = -1;
    pcl::console::parse_argument(argc, argv, "--l", layer);
     
    sparseK cluster_depth, cluster_color;
    Hier_Pooler hie_producer(radius);
    int count = 0;
    for( int b1 = c1 ; b1 <= c2 ; b1++ )
    {
        ObjectSet train_objects, test_objects;
        readBB(path, train_objects, test_objects, b1, b1);
        test_objects.clear();
        
        int num = train_objects[0].size();
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < num ; j++ )
        {
            std::vector<cv::Mat> rawfea_L0 = hie_producer.getRawFea(train_objects[0][j], layer, maxnum);
            #pragma omp critical
            {
                count++;
                if( count % 100 == 0 )
                    std::cerr << count << " ";
                
                int len = rawfea_L0[0].rows;
                for( int p = 0 ; p < len ; p++)
                {
                    cluster_depth.AddData(rawfea_L0[0].row(p));
                    cluster_color.AddData(rawfea_L0[1].row(p));
                }
            }
        }
    }
    
    std::cerr << "Start Kmeans..." << std::endl;
    
    cv::Mat center_depth, center_color;
    HierKmeans(cluster_depth.proto_set, center_depth, KK);
    HierKmeans(cluster_color.proto_set, center_color, KK);
    
    std::stringstream ss, ll;
    ss << KK;
    ll << layer;
    
    saveMat(out_path + "/dict_depth_L0_"+ss.str()+".cvmat", center_depth);
    saveMat(out_path + "/dict_color_L0_"+ss.str()+".cvmat", center_color);
    
    return 1;
}

//*/

/*
int main(int argc, char** argv)
{
    int c1 = 0, c2 = BB_INST_MAX-1;
    int KK = 400;
    
    float radius = 0.03;
    int maxnum = 25;
    std::string path("/home/chi/BigBIRD/processed");
    //std::string path("/home/chi/UW_RGBD/filtered_pcd");
    
    //std::string out_path("UW_dict");
    std::string out_path("BB_new_dict");
    
    int shot_type = 0;
    if( pcl::console::find_switch(argc, argv, "-cshot") == true )
        shot_type = 1;
    
    if( pcl::console::find_switch(argc, argv, "-dcshot") == true )
        shot_type = 2;
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    pcl::console::parse_argument(argc, argv, "--K", KK);
    pcl::console::parse_argument(argc, argv, "--p", path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--m", maxnum);
    pcl::console::parse_argument(argc, argv, "--r", radius);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    int layer = 0;
    pcl::console::parse_argument(argc, argv, "--l", layer);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    sparseK cluster_0, cluster_1, cluster_2, cluster_3;
    
    Hier_Pooler hie_producer(radius);
    if( layer >= 1)
        hie_producer.LoadDict_L0(out_path, "200", "200");
    if( layer >= 2 )
    {
        std::vector<std::string> K_L1;
        K_L1.push_back("400");
        K_L1.push_back("400");
        K_L1.push_back("400");
        K_L1.push_back("400");
        hie_producer.LoadDict_L1(out_path, K_L1);
    }
    int count = 0;
    for( int b1 = c1 ; b1 <= c2 ; b1++ )
    {
        ObjectSet train_objects, test_objects;
        readBB(path, train_objects, test_objects, b1, b1);
        test_objects.clear();
        
        int num = train_objects[0].size();
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < num ; j++ )
        {
            std::vector<cv::Mat> rawfea_L1 = hie_producer.getRawFea(train_objects[0][j], layer, maxnum);
                
            #pragma omp critical
            {
                count++;
                if( count % 100 == 0 )
                    std::cerr << count << " ";
                
                int len = rawfea_L1[0].rows;
                for( int p = 0 ; p < len ; p++)
                {
                    cluster_0.AddData(rawfea_L1[0].row(p));
                    cluster_1.AddData(rawfea_L1[1].row(p));
                    cluster_2.AddData(rawfea_L1[2].row(p));
                    cluster_3.AddData(rawfea_L1[3].row(p));
                }
            }
        }
    }
    
    std::cerr << "Start Kmeans..." << std::endl;
    
    cv::Mat center_0, center_1, center_2, center_3;
    HierKmeans(cluster_0.proto_set, center_0, KK);
    HierKmeans(cluster_1.proto_set, center_1, KK);
    HierKmeans(cluster_2.proto_set, center_2, KK);
    HierKmeans(cluster_3.proto_set, center_3, KK);
    
    std::stringstream ss, ll;
    ss << KK;
    ll << layer;
    
    saveMat( out_path + "/dict_colorInLAB_L"+ll.str()+"_"+ss.str()+".cvmat", center_0 );
    saveMat( out_path + "/dict_depthInLAB_L"+ll.str()+"_"+ss.str()+".cvmat", center_1 );
    saveMat( out_path + "/dict_colorInXYZ_L"+ll.str()+"_"+ss.str()+".cvmat", center_2 );
    saveMat( out_path + "/dict_depthInXYZ_L"+ll.str()+"_"+ss.str()+".cvmat", center_3 );
    
    return 1;
}

//*/

/*
int main(int argc, char** argv)
{
    int c1 = 0, c2 = BB_INST_MAX-1;
    int KK = 300;
    float prob = 0.1;
    //std::string path("/home/chi/BigBIRD/processed");
    std::string path("/home/chi/UW_RGBD/filtered_pcd");
    
    pcl::console::parse_argument(argc, argv, "--K", KK);
    pcl::console::parse_argument(argc, argv, "--p", prob);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    CloudSet clouds;
    NormalSet normals;
    //readBBRaw(path, "n_test", clouds, normals, c1, c2);
    readUWInstAll(path, clouds, normals, c1, c2);
    
    int num = clouds.size();
    std::vector<cv::Mat> fea_pool(num);
    cv::Mat dict = gen3DNormCube(8);
    cv::flann::Index dict_tree;
    cv::flann::LinearIndexParams indexParams;
    dict_tree.build(dict, indexParams);
    
    int len = dict.rows;
    int dictK = len *0.15;
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        //temp = fpfh_cloud(clouds[j], normals[j], 0.05);
        cv::Mat temp = shot_cloud_ss(clouds[j], normals[j], 0.005, 0.02);
        //cv::Mat temp = getHSIHist(clouds[j], dict_tree, len, dictK, 0.005);
        std::vector<int> rand_idx;
        GenRandSeq(rand_idx, temp.rows);
        
        int cur_len = temp.rows * prob;
        if( cur_len == 0 )
            cur_len = temp.rows;
        fea_pool[j] = cv::Mat::zeros(cur_len, temp.cols, CV_32FC1);
        for( int k = 0 ; k < cur_len ; k++)
            temp.row(rand_idx[k]).copyTo(fea_pool[j].row(k));
        clouds[j]->clear();
        normals[j]->clear();
    }
    clouds.clear();
    normals.clear();
    cv::Mat final_train_fea;
    cv::vconcat(fea_pool, final_train_fea);            
    std::cerr << final_train_fea.rows << " " << final_train_fea.cols << std::endl;
    
    std::cerr << "Start Kmeans..." << std::endl;
    //int K[5] = {500, 1000, 2000, 4000, 8000};
    int K[1] = {KK};
    for( size_t j = 0 ; j < 1 ; j++ )
    {
        cv::Mat final_center, labels;
        cv::kmeans(final_train_fea, K[j], labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1e-6), 5, cv::KMEANS_PP_CENTERS, final_center );
        std::stringstream ss;
        ss << K[j];
        //saveMat( "dict/kcenter_shot_all_"+ss.str()+".cvmat", final_center );
        saveMat( "dict/kcenter_hsihist_"+ss.str()+".cvmat", final_center );
    }
    
    return 1;
}
//*/




