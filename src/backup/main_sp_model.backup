#include <opencv2/core/core.hpp>
#include <pcl-1.7/pcl/point_cloud.h>

#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

void sampleSp(const ObjectSet &ori_set, ObjectSet &new_set, float down_ss = 0.005);
void sampleSeg(const ObjectSet &ori_set, ObjectSet &new_set, const std::pair<float, float> &cur_box, int max_rand_sample = 30, float down_ss = 0.005);
std::vector<cv::Mat> extFea(const std::vector<cv::Mat> &main_fea, const std::vector<int> &idx);

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/filtered_pcd/");
    std::string out_path("../data_pool/ln_3x3_aux/fea/");
    std::string dict_path("BB_new_dict/");
    
    int c1 = 0, c2 = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    int dataset_id = -1;
    float radius = 0.03;
    float down_ss = 0.005;
    float ratio = 0;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    int box_num = 50;
    pcl::console::parse_argument(argc, argv, "--nn", box_num);
    bool debug_flag = false;
    if( pcl::console::find_switch(argc, argv, "-d") == true )
        debug_flag = true;
    
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->initCameraParameters();
//    viewer->addCoordinateSystem(0.1);
//    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
//    viewer->setSize(640, 480);
    
    Hier_Pooler hie_producer_3(radius);
    hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    hie_producer_3.setRatio(ratio);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::vector< boost::shared_ptr<Pooler_L0> > pooler_set(FEA_LAYER+1);
    for( size_t i = 1 ; i < pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        pooler_set[i] = cur_pooler;
    }
    
    std::vector< std::pair<int, int> > bound_vec;
    bound_vec.push_back(std::pair<int, int> (50, 250));
    bound_vec.push_back(std::pair<int, int> (250, 500));
    bound_vec.push_back(std::pair<int, int> (500, 1000000));
    
    int label_count = 1;
    for( int i = c1 ; i <= c2  ; i++, label_count++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << label_count;
        
        ObjectSet train_objects, test_objects;
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i, 1);
                break;
            case 1:
                readBB(in_path, train_objects, test_objects, i, i);
                break;
            case 2:
                readJHUInst(in_path, train_objects, test_objects, i, i);
                break;
            default:
                std::cerr << "No Corresponding Dataset!" << std::endl;
                exit(0);
        }
        std::cerr << "Loading Completed... " << std::endl;
        
        int train_num = train_objects[0].size();
        std::cerr << "Train " << i << " --- " << train_num << std::endl;
        
        if( train_num > 0 )
        {
            std::vector< sparseVec> final_train_low;
            std::vector< sparseVec> final_train_med;
            std::vector< sparseVec> final_train_high;
            int final_train_fea_dim = -1;
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < train_num ; j++ )
            {
                pcl::VoxelGrid<PointT> sor;
                pcl::ExtractIndices<PointT> ext;
                ext.setNegative(false);
                
                MulInfoT *inst_ptr = &train_objects[0][j];
                
                MulInfoT tmp_data = convertPCD(inst_ptr->cloud, inst_ptr->cloud_normals);
                if( down_ss > 0 )
                {
                    sor.setInputCloud(tmp_data.cloud);
                    sor.setLeafSize(down_ss, down_ss, down_ss);
                    sor.filter(*tmp_data.down_cloud);
                }
                else
                    tmp_data.down_cloud = tmp_data.cloud;
                PreCloud(tmp_data, -1, false);
                std::vector<cv::Mat> main_fea = hie_producer_3.getHierFea(tmp_data, 0);
                cv::Mat this_temp = multiPool(pooler_set, tmp_data, main_fea);
                        
                final_train_fea_dim = this_temp.cols;
                std::vector< sparseVec> this_sparse;
                sparseCvMat(this_temp, this_sparse);
                #pragma omp critical
                {
                final_train_high.push_back(this_sparse[0]);
                }
                continue;
               
                //#pragma omp parallel for schedule(dynamic, 1)
                for( int t = 0 ; t <= 2 ; t++ )
                {
                    std::vector<pcl::PointIndices::Ptr> inlier_set = CropSegs(tmp_data, bound_vec[t].first, bound_vec[t].second, box_num);
                    //std::cerr << "******" << inlier_set.size() << std::endl;
                    if( inlier_set.empty() == true )
                        continue;

                    std::vector< sparseVec> tmp_final;
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
                            std::cerr << cur_seg->size() << std::endl;
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
                        tmp_final.push_back(final_sparse[0]);
                    }
                    #pragma omp critical
                    {
                        switch(t)
                        {
                            case 0:
                                final_train_low.insert(final_train_low.end(), tmp_final.begin(), tmp_final.end());
                                break;
                            case 1:
                                final_train_med.insert(final_train_med.end(), tmp_final.begin(), tmp_final.end());
                                break;
                            case 2:
                                final_train_high.insert(final_train_high.end(), tmp_final.begin(), tmp_final.end());
                                break;
                            default:exit(0);
                        }
                    }
                }
            }
            std::cerr << final_train_low.size() << " " << final_train_med.size() << " " << final_train_high.size() << std::endl;
//            saveCvMatSparse(out_path + "train_"+ss.str()+"_low.smat", final_train_low, final_train_fea_dim);
//            saveCvMatSparse(out_path + "train_"+ss.str()+"_med.smat", final_train_med, final_train_fea_dim);
//            saveCvMatSparse(out_path + "train_"+ss.str()+"_high.smat", final_train_high, final_train_fea_dim);
            saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train_high, final_train_fea_dim);
            final_train_low.clear();
            final_train_med.clear();
            final_train_high.clear();
        }
        train_objects.clear();
        
        int test_num = test_objects[0].size();
        std::cerr << "Test " << i << " --- " << test_num << std::endl;
        
        if( test_num > 0 )
        {
            std::vector< sparseVec> final_test_low;
            std::vector< sparseVec> final_test_med;
            std::vector< sparseVec> final_test_high;
            int final_test_fea_dim = -1;
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < test_num ; j++ )
            {
//                std::cerr << j << " ";
                pcl::VoxelGrid<PointT> sor;
                pcl::ExtractIndices<PointT> ext;
                ext.setNegative(false);
                
                MulInfoT *inst_ptr = &test_objects[0][j];
                
                MulInfoT tmp_data = convertPCD(inst_ptr->cloud, inst_ptr->cloud_normals);
                if( down_ss > 0 )
                {
                    sor.setInputCloud(tmp_data.cloud);
                    sor.setLeafSize(down_ss, down_ss, down_ss);
                    sor.filter(*tmp_data.down_cloud);
                }
                else
                    tmp_data.down_cloud = tmp_data.cloud;
                
                PreCloud(tmp_data, -1, false);
                std::vector<cv::Mat> main_fea = hie_producer_3.getHierFea(tmp_data, 0);
               
                cv::Mat this_temp = multiPool(pooler_set, tmp_data, main_fea);
                        
                final_test_fea_dim = this_temp.cols;
                std::vector< sparseVec> this_sparse;
                sparseCvMat(this_temp, this_sparse);
                #pragma omp critical
                {
                final_test_high.push_back(this_sparse[0]);
                }
                continue;
               
                //#pragma omp parallel for schedule(dynamic, 1)
                for( int t = 0 ; t <= 2 ; t++ )
                {
                    std::vector<pcl::PointIndices::Ptr> inlier_set = CropSegs(tmp_data, bound_vec[t].first, bound_vec[t].second, box_num);
                    //std::cerr << "******" << inlier_set.size() << std::endl;
                    if( inlier_set.empty() == true )
                        continue;

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
                            std::cerr << cur_seg->size() << std::endl;
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
                    std::vector< sparseVec> tmp_final(data_num);
                    //#pragma omp parallel for schedule(dynamic, 1)
                    for( size_t k = 0 ; k < data_num ; k++ )
                    {
                        cv::Mat final_temp = multiPool(pooler_set, new_set[0][k], main_fea_set[k]);
                        main_fea_set[k].clear();

                        final_test_fea_dim = final_temp.cols;
                        std::vector< sparseVec> final_sparse;
                        sparseCvMat(final_temp, final_sparse);
                        tmp_final[k] = final_sparse[0];
                    }
                    #pragma omp critical
                    {
                        switch(t)
                        {
                            case 0:
                                final_test_low.insert(final_test_low.end(), tmp_final.begin(), tmp_final.end());
                                break;
                            case 1:
                                final_test_med.insert(final_test_med.end(), tmp_final.begin(), tmp_final.end());
                                break;
                            case 2:
                                final_test_high.insert(final_test_high.end(), tmp_final.begin(), tmp_final.end());
                                break;
                            default:exit(0);
                        }
                    }
                }
            }
            std::cerr << final_test_low.size() << " " << final_test_med.size() << " " << final_test_high.size() << std::endl;
//            saveCvMatSparse(out_path + "test_"+ss.str()+"_low.smat", final_test_low, final_test_fea_dim);
//            saveCvMatSparse(out_path + "test_"+ss.str()+"_med.smat", final_test_med, final_test_fea_dim);
//            saveCvMatSparse(out_path + "test_"+ss.str()+"_high.smat", final_test_high, final_test_fea_dim);
            saveCvMatSparse(out_path + "test_"+ss.str()+"_L0.smat", final_test_high, final_test_fea_dim);
            final_test_low.clear();
            final_test_med.clear();
            final_test_high.clear();
        }
        test_objects.clear();
       
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
    }
    
    return 1;
} 

/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/filtered_pcd/");
    std::string out_path("../data_pool/ln_3x3_aux/fea/");
    std::string dict_path("BB_new_dict/");
    
    int c1 = 0, c2 = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    int dataset_id = -1;
    float radius = 0.03;
    float down_ss = 0.005;
    float ratio = 0;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
//    float boxw = 0.03, boxh = 0.03;
    float box_size = 0.03;
    int box_num = 50;
    pcl::console::parse_argument(argc, argv, "--nn", box_num);
    pcl::console::parse_argument(argc, argv, "--box", box_size);
//    pcl::console::parse_argument(argc, argv, "--ww", boxw);
//    pcl::console::parse_argument(argc, argv, "--hh", boxh);
    
//    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->initCameraParameters();
//    viewer->addCoordinateSystem(0.1);
//    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
//    viewer->setSize(640, 480);
    
    Hier_Pooler hie_producer_3(radius);
    hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    hie_producer_3.setRatio(ratio);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::vector< boost::shared_ptr<Pooler_L0> > pooler_set(FEA_LAYER+1);
    for( size_t i = 1 ; i < pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        pooler_set[i] = cur_pooler;
    }
    
    int label_count = 1;
    for( int i = c1 ; i <= c2  ; i++, label_count++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << label_count;
        
        ObjectSet pre_train_objects, pre_test_objects;
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, pre_train_objects, pre_test_objects, i, i, 1);
                break;
            case 1:
                readBB(in_path, pre_train_objects, pre_test_objects, i, i);
                break;
            case 2:
                readJHUInst(in_path, pre_train_objects, pre_test_objects, i, i);
                break;
            default:
                std::cerr << "No Corresponding Dataset!" << std::endl;
                exit(0);
        }
        std::cerr << "Loading Completed... " << std::endl;
        
        ObjectSet train_objects, test_objects;
        
        //sampleSp(pre_train_objects, train_objects, down_ss);
        //sampleSp(pre_test_objects, test_objects, down_ss);
        //hardSampling(pre_train_objects, train_objects, down_ss);
        //hardSampling(pre_test_objects, test_objects, down_ss);
        
//        for( int j = 0 ; j < pre_train_objects[0].size() ; j++ )
//        {
//            std::cerr << j << std::endl;
//            MulInfoT *inst_ptr = &pre_train_objects[0][j];
//            viewer->removeAllPointClouds();
//            viewer->addPointCloud(inst_ptr->cloud, "cloud");
//            viewer->spin();
//        }
        
        std::pair<float, float> box;
        box.first = box_size; box.second = box_size;
        sampleSeg(pre_train_objects, train_objects, box, box_num, down_ss);
        sampleSeg(pre_test_objects, test_objects, box, box_num, down_ss);
        std::cerr << pre_train_objects[0].size() << " " << train_objects[0].size() << std::endl;
        std::cerr << pre_test_objects[0].size() << " " << test_objects[0].size() << std::endl;
        
//        for( size_t j = 0 ; j < train_objects[0].size() ; j++ )
//        {
//            std::cerr << j << std::endl;
//            MulInfoT *inst_ptr = &train_objects[0][j];
//            viewer->removeAllPointClouds();
//            viewer->addPointCloud(inst_ptr->down_cloud, "cloud");
//            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
//            viewer->spin();
//        }
        
        pre_train_objects.clear();
        pre_test_objects.clear();
        
        int train_num = train_objects[0].size();
        std::cerr << "Class " << i << " --- " << train_num << std::endl;
        
        int count[40] = {0};
        for( int j = 0 ; j < train_num ; j++ )
        {
            MulInfoT *inst_ptr = &train_objects[0][j];
            if( inst_ptr->down_cloud->size() >= 2000 )
                count[39]++;
            else
                count[inst_ptr->down_cloud->size()/50]++;
        }
        for( int t = 0 ; t < 39 ; t++ )
            std::cerr << count[t] << " ";
        std::cerr << std::endl;
        continue;
        if( train_num > 0 )
        {
            std::vector< sparseVec> final_train_fea(train_num);
            int final_train_fea_dim = -1;
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < train_num ; j++ )
            {
                MulInfoT *inst_ptr = &train_objects[0][j];
                
                std::vector<cv::Mat> local_fea = hie_producer_3.getHierFea(*inst_ptr, 0);
                cv::Mat final_temp = multiPool(pooler_set, *inst_ptr, local_fea);
                local_fea.clear();

                final_train_fea_dim = final_temp.cols;
                std::vector< sparseVec> final_sparse;
                sparseCvMat(final_temp, final_sparse);
                final_train_fea[j] = final_sparse[0];
            }
            //saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
            final_train_fea.clear();
        }
        train_objects.clear();
        
        int test_num = test_objects[0].size();
        if( test_num > 0 )
        {
            std::vector< sparseVec> final_test_fea(test_num);
            int final_test_fea_dim = -1;
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < test_num ; j++ )
            {
                MulInfoT *inst_ptr = &test_objects[0][j];
                
                std::vector<cv::Mat> local_fea = hie_producer_3.getHierFea(*inst_ptr, 0);
                cv::Mat final_temp = multiPool(pooler_set, *inst_ptr, local_fea);
                local_fea.clear();

                final_test_fea_dim = final_temp.cols;
                std::vector< sparseVec> final_sparse;
                sparseCvMat(final_temp, final_sparse);
                final_test_fea[j] = final_sparse[0];
            }
            saveCvMatSparse(out_path + "test_"+ss.str()+"_L0.smat", final_test_fea, final_test_fea_dim);
            final_test_fea.clear();
        }
        test_objects.clear();        
        
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
    }
    
    return 1;
} 
//*/

void sampleSeg(const ObjectSet &ori_set, ObjectSet &new_set, const std::pair<float, float> &cur_box, int max_rand_sample, float down_ss)
{
    float in_box_ratio = 0.5;
    
    std::vector< std::pair<float, float> > box_size;
    box_size.push_back(cur_box);
    
    new_set.clear();
    new_set.resize(1);
    
    // Create the filtering object
    pcl::ExtractIndices<PointT> ext;
    ext.setNegative(false);
            
    for( std::vector<MulInfoT>::const_iterator itt = ori_set[0].begin() ; itt < ori_set[0].end() ; itt++ )
    {
        MulInfoT tmp_data = *itt;
        PreCloud(tmp_data, down_ss, false);
        std::cerr << tmp_data.down_cloud->size() << " ";
        if( tmp_data.down_cloud->empty() )
            continue;
        
        std::vector<pcl::PointIndices::Ptr> inlier_set = CropSegs(tmp_data, box_size, max_rand_sample, in_box_ratio);
        if( inlier_set.empty() == true )
            new_set[0].push_back(tmp_data);
        
        for(std::vector<pcl::PointIndices::Ptr>::iterator it_inlier = inlier_set.begin() ; it_inlier < inlier_set.end() ; it_inlier++ )
        {
            pcl::PointCloud<PointT>::Ptr cur_seg(new pcl::PointCloud<PointT>());
             // Extract the inliers
            ext.setInputCloud (tmp_data.down_cloud);
            ext.setIndices (*it_inlier);
            ext.filter (*cur_seg);
            
            MulInfoT cur_data = convertPCD(tmp_data.cloud, tmp_data.cloud_normals);
            cur_data.down_cloud = cur_seg;
            PreCloud(cur_data, -1, false);
            
            new_set[0].push_back(cur_data);
        }
    }
    std::cerr << std::endl;
}

void hardSampling(const ObjectSet &ori_set, ObjectSet &new_set, float down_ss = 0.005)
{
    new_set.clear();
    new_set.resize(1);
    
    ObjectSet tmp_set;
    std::pair<float, float> box;
    box.first = 0.03; box.second = 0.03;
    sampleSeg(ori_set, tmp_set, box, 40, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.04; box.second = 0.04;
    sampleSeg(ori_set, tmp_set, box, 30, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.05; box.second = 0.05;
    sampleSeg(ori_set, tmp_set, box, 20, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.06; box.second = 0.06;
    sampleSeg(ori_set, tmp_set, box, 10, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.03; box.second = 0.04;
    sampleSeg(ori_set, tmp_set, box, 30, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.04; box.second = 0.03;
    sampleSeg(ori_set, tmp_set, box, 30, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.03; box.second = 0.05;
    sampleSeg(ori_set, tmp_set, box, 20, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
    
    box.first = 0.05; box.second = 0.03;
    sampleSeg(ori_set, tmp_set, box, 20, down_ss);
    new_set[0].insert(new_set[0].end(), tmp_set[0].begin(), tmp_set[0].end());
}
