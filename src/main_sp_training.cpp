#include <opencv2/core/core.hpp>

#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

#define MAX_LAYER 7

int main(int argc, char** argv)
{
    std::string out_path("../../data_pool/");
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    boost::filesystem::create_directories(out_path);
    
    std::string model_path("/home/chi/JHUIT/ht10/");
    std::string background_path("/home/chi/JHUIT/scene/");
    std::vector<std::string> background_name;
    background_name.push_back("UR5");
//    background_name.push_back("office_background");
//    background_name.push_back("office_raw");
//    background_name.push_back("office_imposter");
//    background_name.push_back("labpod_background");
//    background_name.push_back("labpod_raw");
//    background_name.push_back("labpod_imposter");
//    background_name.push_back("barrett_background");
//    background_name.push_back("barrett_raw");
//    background_name.push_back("barrett_imposter");
//    background_name.push_back("box_background");
//    background_name.push_back("box_raw");
//    background_name.push_back("box_imposter");
//    background_name.push_back("shelf_background");
//    background_name.push_back("shelf_raw");
//    background_name.push_back("shelf_imposter");
    
    std::string shot_path("UW_shot_dict/");
    std::string sift_path("UW_new_sift_dict/");
    std::string fpfh_path("UW_fpfh_dict/");
    
/***************************************************************************************************************/
    float radius = 0.02;
    float down_ss = 0.003;
    float ratio = 0.1;
    int box_num = 10;
//    float sigma = 0.9;
    pcl::console::parse_argument(argc, argv, "--nn", box_num);
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
//    pcl::console::parse_argument(argc, argv, "--sigma", sigma);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
/***************************************************************************************************************/
    Hier_Pooler hie_producer(radius);
    hie_producer.LoadDict_L0(shot_path, "200", "200");
    hie_producer.setRatio(ratio);
/***************************************************************************************************************/    
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
/***************************************************************************************************************/
    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set(1+1);
    for( size_t i = 1 ; i < sift_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
        sift_pooler_set[i] = cur_pooler;
    }
    sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_400.cvmat"); 
/***************************************************************************************************************/
    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set(1+1);
    for( size_t i = 1 ; i < fpfh_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
        fpfh_pooler_set[i] = cur_pooler;
    }
    fpfh_pooler_set[1]->LoadSeedsPool(fpfh_path+"dict_fpfh_L0_400.cvmat");
/***************************************************************************************************************/
    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        lab_pooler_set[i] = cur_pooler;
    }
/***************************************************************************************************************/
    
    if( false )
    {
        int train_dim = -1;
        std::vector< std::vector<cv::Mat> > raw_fea(10);
        for( int i = 0 ; i <= 9 ; i++ )
        {
            std::stringstream ss;
            ss << i+1;

            ObjectSet train_objects, test_objects;
            readJHUInstWithImg(model_path, train_objects, test_objects, i, i, true);
            std::cerr << "Loading Completed... " << std::endl;

            int train_num = train_objects[0].size();
            std::cerr << "Train " << i << " --- " << train_num << std::endl;

            if( train_num > 0 )
            {
                std::vector< std::vector<sparseVec> > final_train(MAX_LAYER);
                #pragma omp parallel for schedule(dynamic, 1)
                for( int j = 0 ; j < train_num ; j++ )
                {
                    pcl::PointCloud<PointT>::Ptr mycloud = train_objects[0][j].cloud;
                    cv::Mat map2d = train_objects[0][j].map2d;
                    cv::Mat img = train_objects[0][j].img;

                    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                    full_cloud->resize(map2d.rows*map2d.cols);
                    for(int r = 0, this_idx = 0 ; r < map2d.rows ; r++ ){
                        for(int c = 0 ; c < map2d.cols ; c++, this_idx++ )
                        {
                            int idx2 = map2d.at<int>(r, c);
                            if( idx2 >= 0 )
                            {
                                full_cloud->at(this_idx).x = mycloud->at(idx2).x;
                                full_cloud->at(this_idx).y = mycloud->at(idx2).y;
                                full_cloud->at(this_idx).z = mycloud->at(idx2).z;
                                full_cloud->at(this_idx).rgba = mycloud->at(idx2).rgba;
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

                    spPooler triple_pooler;
                    triple_pooler.init(full_cloud, hie_producer, radius, down_ss);
                    triple_pooler.build_SP_LAB(lab_pooler_set, false);
                    triple_pooler.build_SP_FPFH(fpfh_pooler_set, radius, false);
                    triple_pooler.build_SP_SIFT(sift_pooler_set, hie_producer, sift_det_vec, false);

                    for( int ll = 0 ; ll <= 4 ; ll++ )
                    {
                        std::vector<cv::Mat> sp_fea = triple_pooler.sampleSPFea(ll, box_num, false, true);
                        std::vector<cv::Mat> sp_raw;
                        if( ll == 0 )
                            sp_raw = triple_pooler.sampleSPFea(ll, box_num, false, false);
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
                                final_train[ll].push_back(this_sparse[0]);
                                if( ll == 0 )
                                    raw_fea[i].insert(raw_fea[i].end(), sp_raw.begin(), sp_raw.end());
                            }
                        }
                    }
                }

                for( size_t ll = 0 ; ll < final_train.size() ; ll++ )
                {
                    if( final_train[ll].empty() == true )
                        continue;
                    std::stringstream mm;
                    mm << ll;
                    saveCvMatSparse(out_path + "train_" + ss.str() + "_L" + mm.str() + ".smat", final_train[ll], train_dim);
                    final_train[ll].clear();
                }
            }
            else
            {
                std::cerr << "JHU Model Data Reading Failed!" << std::endl;
                exit(0);
            }
            train_objects.clear();
        }
    }
    
    if( true )
    {
        int test_dim = -1;
        for( size_t k = 0 ; k < background_name.size() ; k++ )
        {
            std::stringstream kk;
            kk << k;
            
            boost::filesystem::create_directories(out_path + "/" + background_name[k]);
            
//            std::vector<std::string> files;
//            getNonNormalPCDFiles(background_path + background_name[k] +"/", files);
            std::vector< std::vector< sparseVec> > final_test(MAX_LAYER);
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < 200 ; j++ )
            {
                std::stringstream ss;
                ss << j;

                std::string filename(background_path + background_name[k] +"/" + background_name[k] + "_" + ss.str() + ".pcd");
                std::cerr << "Loading " << filename << std::endl;
                if( exists_test(filename) == false )//|| exists_test(filename_n) == false )
                {
                    pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
                    continue;
                }
                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(filename, *full_cloud);

                spPooler triple_pooler;
                triple_pooler.init(full_cloud, hie_producer, radius, down_ss);
                triple_pooler.build_SP_LAB(lab_pooler_set, false);

                for( int ll = 0 ; ll <= 2 ; ll++ )
                {
                    std::vector<cv::Mat> sp_fea = triple_pooler.sampleSPFea(ll, box_num * 10);
                    for( std::vector<cv::Mat>::iterator it = sp_fea.begin(); it < sp_fea.end() ; it++ )
                    {
                        if( test_dim > 0 && it->cols != test_dim )
                        {
                            std::cerr << "Error: test_dim > 0 && cur_final.cols != test_dim   " << test_dim << " " << it->cols << std::endl;
                            exit(0);
                        }
                        else if( test_dim < 0 )
                        {
                            #pragma omp critical
                            {
                                test_dim = it->cols;
                                std::cerr << "Fea Dim: " << test_dim << std::endl;
                            }
                        }	
                        std::vector< sparseVec> this_sparse;
                        sparseCvMat(*it, this_sparse);
                        #pragma omp critical
                        {
                            final_test[ll].push_back(this_sparse[0]);
                        }
                    }
                }
                triple_pooler.reset();
            }
            for( size_t ll = 0 ; ll < final_test.size() ; ll++ )
            {
                if( final_test[ll].empty() == true )
                    continue;
                std::stringstream mm;
                mm << ll;
                saveCvMatSparse(out_path + "/" + background_name[k] + "/train_0_L" + mm.str() +".smat", final_test[ll], test_dim);
                final_test[ll].clear();
            }
        }
    }
//    model *binary_model = TrainBinarySVM(out_path, CC2, true);
//    free_and_destroy_model(&binary_model);
    
    return 1;
} 

