#include <opencv2/core/core.hpp>
#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

void sampleSeg(const ObjectSet &ori_set, ObjectSet &new_set, const std::pair<float, float> &cur_box, int max_rand_sample = 30, float down_ss = 0.005)
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
}

//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/filtered_pcd/");
    std::string out_path("../data_pool/ln_3x3_aux/fea/");
    std::string dict_path("BB_new_dict/");
    
    int c1 = 0, c2 = -1;
    int dataset_id = -1;
    float radius = 0.03;
    float down_ss = 0.005;
    float ratio = 0;
    int max_rand_num = 30;
    std::pair<float, float> cur_box(-1, -1);
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    pcl::console::parse_argument(argc, argv, "--mrand", max_rand_num);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    cur_box = readBoxFile( out_path + "model.box");
    std::cerr << "Training Box: " << cur_box.first << "-" << cur_box.second << std::endl;
    if( cur_box.first <= 0 || cur_box.second <= 0 )
    {
        std::cerr << "Please Specify Searching Box!" << std::endl;
        exit(0);
    }
    
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
        sampleSeg(pre_train_objects, train_objects, cur_box, max_rand_num, down_ss);
        int train_num = train_objects[0].size();
        std::cerr << "Class " << i << " --- " << train_num << std::endl;
        
        sampleSeg(pre_test_objects, test_objects, cur_box, max_rand_num, down_ss);
        pre_train_objects.clear();
        pre_test_objects.clear();
        
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
            saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
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


//            for( int k = 1 ; k <= 4 ; k++ )
//                poolOneLayer(*pooler_set[k], cur_seg, local_fea[0], pool_fea_vec);
//            for( int k = 1 ; k <= 4 ; k++ )
//                poolOneLayer(*pooler_set[k], cur_seg, local_fea[1], pool_fea_vec);
           
//            poolOneLayer(shot_pooler_L1, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L2, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L3, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L4, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L5, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L6, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L7, cur_seg, local_fea[0], pool_fea_vec);
//            poolOneLayer(shot_pooler_L8, cur_seg, local_fea[0], pool_fea_vec);

//            poolOneLayer(shot_pooler_L1, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L2, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L3, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L4, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L5, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L6, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L7, cur_seg, local_fea[1], pool_fea_vec);
//            poolOneLayer(shot_pooler_L8, cur_seg, local_fea[1], pool_fea_vec);
//
//            cv::Mat final_temp;
//            cv::hconcat(pool_fea_vec, final_temp);


//    Pooler_L0 shot_pooler_L8(8); // 0.03-L1
//    Pooler_L0 shot_pooler_L7(7); // 0.03-L1
//    Pooler_L0 shot_pooler_L6(6); // 0.03-L2
//    Pooler_L0 shot_pooler_L5(5); // 0.03-L0
//    Pooler_L0 shot_pooler_L4(4); // 0.03-L0
//    Pooler_L0 shot_pooler_L3(3); // 0.03-L1
//    Pooler_L0 shot_pooler_L2(2); // 0.03-L1
//    Pooler_L0 shot_pooler_L1(1); // 0.03-L2
// 