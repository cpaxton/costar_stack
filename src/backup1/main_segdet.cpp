#include "../include/utility.h"
#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

float color[4][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}}; 

void poolRGBLayer(Pooler_L0 &pooler, const MulInfoT &inst, const cv::Mat &local_fea, std::vector<cv::Mat> &pool_fea_vec)
{    
    //tick true if you wanna max pooling
    bool max_flag = true;
    
    cv::Mat temp = pooler.PoolOneDomain(inst.rgb, local_fea, 1, max_flag);   //lab -> depth
    pool_fea_vec.push_back(temp);
}

void poolXYZLayer(Pooler_L0 &pooler, const MulInfoT &inst, const cv::Mat &local_fea, std::vector<cv::Mat> &pool_fea_vec)
{    
    //tick true if you wanna max pooling
    bool max_flag = true;
    
    cv::Mat temp = pooler.PoolOneDomain(inst.xyz, local_fea, 1, max_flag);   //lab -> depth
    pool_fea_vec.push_back(temp);
}

//*
int main(int argc, char** argv)
{
    float radius = 0.03;
    float down_ss = 0.003;
    float ratio = 0.1;
    
    std::string in_path("/home/chi/jon_data/");
    std::string workspace("../data_pool/jon_s003_rt010/");
    std::string dict_path("BB_new_dict/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--w", workspace);
    
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    std::string model_path = workspace + "SVM_Model/";
    std::string out_path =  workspace + "result_pool/" ;
                           
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    Hier_Pooler fea_producer(radius);
    fea_producer.LoadDict_L0(dict_path, "200", "200");
    fea_producer.setRatio(ratio);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    Pooler_L0 shot_pooler_L8(8); // 0.03-L1
    Pooler_L0 shot_pooler_L7(7); // 0.03-L1
    Pooler_L0 shot_pooler_L6(6); // 0.03-L2
    Pooler_L0 shot_pooler_L5(5); // 0.03-L0
    Pooler_L0 shot_pooler_L4(4); // 0.03-L0
    Pooler_L0 shot_pooler_L3(3); // 0.03-L1
    Pooler_L0 shot_pooler_L2(2); // 0.03-L1
    Pooler_L0 shot_pooler_L1(1); // 0.03-L2

//////////////////////////////////////////////////////////////////////////////////////////////////////
    model* cur_model = load_model((model_path + "svm_model.model").c_str());
    int fea_dim = cur_model->nr_feature - 1;    //subtract the bias term
    std::cerr << "Feature Dimension: " << fea_dim << std::endl;
    
    float bias = cur_model->bias;
    feature_node bias_term;
    bias_term.index = fea_dim +1;
    bias_term.value = bias;
    feature_node end_node;
    end_node.index = -1;
    end_node.value = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////    
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    int v1(0), v2(1);
    viewer->createViewPort (0.0, 0.0, 0.5, 1.0, v1);
    viewer->createViewPort (0.5, 0.0, 1.0, 1.0, v2);
    
    int min_num = 200;
    
    boost::filesystem::path p0(in_path);
    std::vector< std::string > ret1;
    find_dirs(p0, ret1);
    
    int total_count = 0;
    for( size_t i = 0 ; i < ret1.size() ; i++ )
    {
        boost::filesystem::path p1(in_path + ret1[i] + "/");
        std::vector< std::string > ret2;
        find_dirs(p1, ret2);
        for( size_t k = 0 ; k < ret2.size() ; k++ )
        {
            std::string cur_path(in_path + ret1[i] + "/" + ret2[k] + "/");
            std::vector<std::string> pcd_files;
            getNonNormalPCDFiles(cur_path, pcd_files);
            
            size_t file_num = pcd_files.size();
            
            for( size_t j = 0 ; j < file_num ; j++ )
            {
                std::string filename(cur_path + pcd_files[j]);
                std::string filename_n(cur_path+"normal_"+pcd_files[j]);

                std::cerr << "Loading... "<< filename << std::endl;
                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(filename, *full_cloud);
                
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                std::vector<int> idx_ff;
                pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);
        
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
                if( exists_test(filename_n) == false )
                {
                    computeNormals(cloud, cloud_normal, 0.03);
                    pcl::io::savePCDFile(filename_n, *cloud_normal, true);
                }
                else
                    pcl::io::loadPCDFile(filename_n, *cloud_normal);
                
                viewer->removeAllPointClouds(v1);
                viewer->removeAllPointClouds(v2);
                
                pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
                tree->setInputCloud (cloud);

                pcl::EuclideanClusterExtraction<PointT> ec;
                ec.setClusterTolerance (0.015);     // 1.5cm
                ec.setMinClusterSize (min_num);
                ec.setMaxClusterSize (INF_);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud);
                std::vector<pcl::PointIndices> cluster_indices;
                ec.extract (cluster_indices);
                
                int count = 0;
                srand(time(NULL));
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, ++count)
                {
                    std::stringstream ss;
                    ss << count ;
                    
                    pcl::PointCloud<PointT>::Ptr cur_seg(new pcl::PointCloud<PointT>());
                    pcl::PointCloud<NormalT>::Ptr cur_seg_normal(new pcl::PointCloud<NormalT>());
                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                    {
                        cur_seg->push_back (cloud->at(*pit));
                        cur_seg_normal->push_back(cloud_normal->at(*pit));
                    }
                    
                    MulInfoT temp = convertPCD(cur_seg, cur_seg_normal);
                    PreCloud(temp, down_ss, false);
                     
                    MulInfoT *inst_ptr = &temp;
                    std::vector<cv::Mat> pool_fea_vec, local_fea;
                    local_fea = fea_producer.getHierFea(*inst_ptr, 0);

                    // RGB pool depth
                    poolRGBLayer(shot_pooler_L1, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L2, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L3, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L4, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L5, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L6, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L7, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L8, *inst_ptr, local_fea[0], pool_fea_vec);

                    // RGB pool color
                    poolRGBLayer(shot_pooler_L1, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L2, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L3, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L4, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L5, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L6, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L7, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolRGBLayer(shot_pooler_L8, *inst_ptr, local_fea[1], pool_fea_vec);

                    // XYZ pool depth
                    poolXYZLayer(shot_pooler_L1, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L2, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L3, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L4, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L5, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L6, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L7, *inst_ptr, local_fea[0], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L8, *inst_ptr, local_fea[0], pool_fea_vec);

                    // XYZ pool color
                    poolXYZLayer(shot_pooler_L1, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L2, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L3, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L4, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L5, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L6, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L7, *inst_ptr, local_fea[1], pool_fea_vec);
                    poolXYZLayer(shot_pooler_L8, *inst_ptr, local_fea[1], pool_fea_vec);

                    cv::Mat final_temp;
                    cv::hconcat(pool_fea_vec, final_temp);

                    sparseVec svm_fea;
                    for( int c = 0 ; c < final_temp.cols ; c++ ){
                        float cur_val = final_temp.at<float>(0, c);
                        if( fabs(cur_val) >= 1e-6 && cur_val == cur_val) 
                        {
                            feature_node cur_node;
                            cur_node.index = c+1;  
                            cur_node.value = cur_val;
                            svm_fea.push_back(cur_node);
                        }
                    }
                    svm_fea.push_back(bias_term);
                    svm_fea.push_back(end_node);

                    feature_node *cur_fea = new feature_node[svm_fea.size()];
                    std::copy(svm_fea.begin(), svm_fea.end(), cur_fea);
                    double *dec_values = new double[cur_model->nr_class];
                    double cur_label = predict_values(cur_model, cur_fea, dec_values);
                    int pred_label =  floor(cur_label +0.001 - 1);
                    
                    viewer->addPointCloud(cur_seg, "seg" + ss.str(), v1);
                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color[pred_label][0], color[pred_label][1], color[pred_label][2], "seg" + ss.str(), v1);
                }
                viewer->addPointCloud(cloud, "cloud", v2);
                //viewer->spin();
                
                std::stringstream idx_str;
                idx_str << total_count;
                viewer->saveScreenshot(out_path+idx_str.str()+".png");
                total_count++;
            }
        }
    }
   
    
    free_and_destroy_model(&cur_model);
    return 1;
}