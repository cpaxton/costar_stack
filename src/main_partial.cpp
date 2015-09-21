#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

pcl::visualization::PCLVisualizer::Ptr viewer;

#define MAX_POOLER_NUM 50
#define MAX_FEA_NUM 10

//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/filtered_pcd/");
    std::string out_path("JHU_fea_pool/");
    std::string dict_path("BB_new_dict/");
    //std::string dict_path("dict");
    
    int c1 = 0, c2 = -1;
    int dataset_id = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
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
    
    Hier_Pooler hie_producer_2(0.02);
    Hier_Pooler hie_producer_3(0.03);
    Hier_Pooler hie_producer_4(0.04);
    Hier_Pooler hie_producer_5(0.05);
    Hier_Pooler hie_producer_6(0.06);
    std::vector<int> dim_L0_2 = hie_producer_2.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_3 = hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_4 = hie_producer_4.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_5 = hie_producer_5.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_6 = hie_producer_6.LoadDict_L0(dict_path, "200", "200");
    
    hie_producer_3.setRatio(ratio);
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
    
    /*
    int fea_len = + (shot_pooler_L8.getHSIPoolLen()*2)*(dim_L0_2[0]+dim_L0_2[1])
                  + (shot_pooler_L7.getHSIPoolLen()*2)*(dim_L0_2[0]+dim_L0_2[1]) 
                  + (shot_pooler_L6.getHSIPoolLen()*2)*(dim_L0_3[0]+dim_L0_3[1]) 
                  + (shot_pooler_L5.getHSIPoolLen()*2)*(dim_L0_3[0]+dim_L0_3[1]) 
                  + (shot_pooler_L4.getHSIPoolLen()*2)*(dim_L0_4[0]+dim_L0_4[1]) 
                  + (shot_pooler_L3.getHSIPoolLen()*2)*(dim_L0_4[0]+dim_L0_4[1]) 
                  + (shot_pooler_L2.getHSIPoolLen()*2)*(dim_L0_5[0]+dim_L0_5[1]) 
                  + (shot_pooler_L1.getHSIPoolLen()*2)*(dim_L0_6[0]+dim_L0_6[1]) 
                  ;   
    
    std::cerr << "Total Feature Length: "<< fea_len << std::endl;
    */
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        ObjectSet train_objects, test_objects;
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i);
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
        if( train_num == 0 )
        {
            std::cerr << "Empty Set!" << std::endl;
            exit(0);
        }
        
        std::vector< sparseVec> final_train_fea(train_num);
        int final_train_fea_dim = -1;
        
        //double time_total = 0;
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < train_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec, local_fea;
            
            MulInfoT *inst_ptr = &train_objects[0][j];
            PreCloud(*inst_ptr, down_ss, false);
            
            //double cur_t1, cur_t2;
            //cur_t1 = get_wall_time();
            
            local_fea = hie_producer_3.getHierFea(*inst_ptr, 0);
            
            // RGB pool depth
            poolRGBLayer(shot_pooler_L1, *inst_ptr, local_fea[0], pool_fea_vec);
            poolRGBLayer(shot_pooler_L2, *inst_ptr, local_fea[0], pool_fea_vec);
            poolRGBLayer(shot_pooler_L3, *inst_ptr, local_fea[0], pool_fea_vec);
            poolRGBLayer(shot_pooler_L4, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L5, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L6, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L7, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L8, *inst_ptr, local_fea[0], pool_fea_vec);
            
            // RGB pool color
            poolRGBLayer(shot_pooler_L1, *inst_ptr, local_fea[1], pool_fea_vec);
            poolRGBLayer(shot_pooler_L2, *inst_ptr, local_fea[1], pool_fea_vec);
            poolRGBLayer(shot_pooler_L3, *inst_ptr, local_fea[1], pool_fea_vec);
            poolRGBLayer(shot_pooler_L4, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L5, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L6, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L7, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L8, *inst_ptr, local_fea[1], pool_fea_vec);
            
            // XYZ pool depth
            //poolXYZLayer(shot_pooler_L1, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L2, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L3, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L4, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L5, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L6, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L7, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L8, *inst_ptr, local_fea[0], pool_fea_vec);
            
            // XYZ pool color
            //poolXYZLayer(shot_pooler_L1, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L2, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L3, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L4, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L5, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L6, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L7, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L8, *inst_ptr, local_fea[1], pool_fea_vec);
            
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            
            //cur_t2 = get_wall_time();
            //time_total += cur_t2 - cur_t1;
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
            
        }
        saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
        train_objects.clear();
        
        int test_num = test_objects[0].size();
        std::vector< sparseVec> final_test_fea(test_num);
        int final_test_fea_dim = -1;
        
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < test_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec, local_fea;
            //double cur_t1, cur_t2;
            //cur_t1 = get_wall_time();
            
            MulInfoT *inst_ptr = &test_objects[0][j];
            PreCloud(*inst_ptr, down_ss, false);
            
            local_fea = hie_producer_3.getHierFea(*inst_ptr, 0);
            
            // RGB pool depth
            poolRGBLayer(shot_pooler_L1, *inst_ptr, local_fea[0], pool_fea_vec);
            poolRGBLayer(shot_pooler_L2, *inst_ptr, local_fea[0], pool_fea_vec);
            poolRGBLayer(shot_pooler_L3, *inst_ptr, local_fea[0], pool_fea_vec);
            poolRGBLayer(shot_pooler_L4, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L5, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L6, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L7, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L8, *inst_ptr, local_fea[0], pool_fea_vec);
            
            // RGB pool color
            poolRGBLayer(shot_pooler_L1, *inst_ptr, local_fea[1], pool_fea_vec);
            poolRGBLayer(shot_pooler_L2, *inst_ptr, local_fea[1], pool_fea_vec);
            poolRGBLayer(shot_pooler_L3, *inst_ptr, local_fea[1], pool_fea_vec);
            poolRGBLayer(shot_pooler_L4, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L5, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L6, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L7, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolRGBLayer(shot_pooler_L8, *inst_ptr, local_fea[1], pool_fea_vec);
            
            // XYZ pool depth
            //poolXYZLayer(shot_pooler_L1, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L2, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L3, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L4, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L5, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L6, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L7, *inst_ptr, local_fea[0], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L8, *inst_ptr, local_fea[0], pool_fea_vec);
            
            // XYZ pool color
            //poolXYZLayer(shot_pooler_L1, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L2, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L3, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L4, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L5, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L6, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L7, *inst_ptr, local_fea[1], pool_fea_vec);
            //poolXYZLayer(shot_pooler_L8, *inst_ptr, local_fea[1], pool_fea_vec);
            
            local_fea.clear();
           
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
         
            //cur_t2 = get_wall_time();
            //time_total += cur_t2 - cur_t1;
            
            final_test_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_test_fea[j] = final_sparse[0];
        }
        saveCvMatSparse(out_path + "test_"+ss.str()+"_L0.smat", final_test_fea, final_test_fea_dim);
        final_test_fea.clear();
        test_objects.clear();        
        
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
        
        //std::cerr << "AVG Time: " << time_total / (train_num + test_num) << std::endl;
    }
    
    return 1;
}
//*/

/*
int main(int argc, char** argv)
{
    //std::string in_path("/home/chi/BigBIRD/processed");
    //std::string out_path("BB_fea_pool");
    //std::string dict_path("BB_new_dict");
    //std::string dict_path("BB_dict");
    
    std::string in_path("/home/chi/UW_RGBD/filtered_pcd");
    std::string out_path("UW_fea_pool");
    std::string dict_path("UW_new_dict");
    //std::string dict_path("UW_dict");
    
    int c1 = 0, c2 = UW_INST_MAX-1;
    float ratio = 0.15;
    int dataset_id = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    Hier_Pooler hie_producer_2(0.02);
    Hier_Pooler hie_producer_3(0.03);
    Hier_Pooler hie_producer_4(0.04);
    Hier_Pooler hie_producer_5(0.05);
    Hier_Pooler hie_producer_6(0.06);
    
    std::vector<int> dim_L0_2 = hie_producer_2.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_3 = hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_4 = hie_producer_4.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_5 = hie_producer_5.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_6 = hie_producer_6.LoadDict_L0(dict_path, "200", "200");
    
    Pooler_L0 depth_xyz_pooler_1(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_2(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_3(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_4(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_5(-1); // 0.03-L0
    int depth_xyz_len_1 = depth_xyz_pooler_1.LoadSeedsPool(dict_path+"/filter_depth_xyz_center_0.cvmat");
    int depth_xyz_len_2 = depth_xyz_pooler_2.LoadSeedsPool(dict_path+"/filter_depth_xyz_center_1.cvmat");
    int depth_xyz_len_3 = depth_xyz_pooler_3.LoadSeedsPool(dict_path+"/filter_depth_xyz_center_2.cvmat");
    int depth_xyz_len_4 = depth_xyz_pooler_4.LoadSeedsPool(dict_path+"/filter_depth_xyz_center_3.cvmat");
    int depth_xyz_len_5 = depth_xyz_pooler_5.LoadSeedsPool(dict_path+"/filter_depth_xyz_center_5.cvmat");
    Pooler_L0 depth_lab_pooler_1(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_2(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_3(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_4(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_5(-1); // 0.03-L0
    int depth_lab_len_1 = depth_lab_pooler_1.LoadSeedsPool(dict_path+"/filter_depth_lab_center_0.cvmat");
    int depth_lab_len_2 = depth_lab_pooler_2.LoadSeedsPool(dict_path+"/filter_depth_lab_center_1.cvmat");
    int depth_lab_len_3 = depth_lab_pooler_3.LoadSeedsPool(dict_path+"/filter_depth_lab_center_2.cvmat");
    int depth_lab_len_4 = depth_lab_pooler_4.LoadSeedsPool(dict_path+"/filter_depth_lab_center_3.cvmat");
    int depth_lab_len_5 = depth_lab_pooler_5.LoadSeedsPool(dict_path+"/filter_depth_lab_center_5.cvmat");
    
    Pooler_L0 color_xyz_pooler_1(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_2(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_3(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_4(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_5(-1); // 0.03-L0
    int color_xyz_len_1 = color_xyz_pooler_1.LoadSeedsPool(dict_path+"/filter_color_xyz_center_0.cvmat");
    int color_xyz_len_2 = color_xyz_pooler_2.LoadSeedsPool(dict_path+"/filter_color_xyz_center_1.cvmat");
    int color_xyz_len_3 = color_xyz_pooler_3.LoadSeedsPool(dict_path+"/filter_color_xyz_center_2.cvmat");
    int color_xyz_len_4 = color_xyz_pooler_4.LoadSeedsPool(dict_path+"/filter_color_xyz_center_3.cvmat");
    int color_xyz_len_5 = color_xyz_pooler_5.LoadSeedsPool(dict_path+"/filter_color_xyz_center_5.cvmat");
    Pooler_L0 color_lab_pooler_1(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_2(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_3(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_4(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_5(-1); // 0.03-L0
    int color_lab_len_1 = color_lab_pooler_1.LoadSeedsPool(dict_path+"/filter_color_lab_center_0.cvmat");
    int color_lab_len_2 = color_lab_pooler_2.LoadSeedsPool(dict_path+"/filter_color_lab_center_1.cvmat");
    int color_lab_len_3 = color_lab_pooler_3.LoadSeedsPool(dict_path+"/filter_color_lab_center_2.cvmat");
    int color_lab_len_4 = color_lab_pooler_4.LoadSeedsPool(dict_path+"/filter_color_lab_center_3.cvmat");
    int color_lab_len_5 = color_lab_pooler_5.LoadSeedsPool(dict_path+"/filter_color_lab_center_5.cvmat");
    
    //int color_len_2 = color_pooler_2.LoadSeedsPool(dict_path+"/refined_color_center_1.cvmat");
    //int color_len_4 = color_pooler_4.LoadSeedsPool(dict_path+"/refined_color_center_2.cvmat");
    //int color_len_6 = color_pooler_6.LoadSeedsPool(dict_path+"/refined_color_center_3.cvmat");
    //int color_len_8 = color_pooler_8.LoadSeedsPool(dict_path+"/refined_color_center_4.cvmat");
    //int color_len_10 = color_pooler_10.LoadSeedsPool(dict_path+"/refined_color_center_5.cvmat");  
    //int color_len_2 = color_pooler_2.LoadHybridPool(dict_path+"/refined_color_center_1.cvmat", dict_path+"/refined_color_range_1.cvmat", 2.0);
    //int color_len_4 = color_pooler_4.LoadHybridPool(dict_path+"/refined_color_center_2.cvmat", dict_path+"/refined_color_range_2.cvmat", 3.0);
    //int color_len_6 = color_pooler_6.LoadHybridPool(dict_path+"/refined_color_center_3.cvmat", dict_path+"/refined_color_range_3.cvmat", 4.0);
    //int color_len_8 = color_pooler_8.LoadHybridPool(dict_path+"/refined_color_center_4.cvmat", dict_path+"/refined_color_range_4.cvmat", 5.0);
    //int color_len_10 = color_pooler_10.LoadHybridPool(dict_path+"/refined_color_center_5.cvmat", dict_path+"/refined_color_range_5.cvmat", 6.0);
    
    // for both depth and color features
    int fea_len_L0 = + (depth_xyz_len_1+depth_lab_len_1)*dim_L0_3[0]+ (color_xyz_len_1+color_lab_len_1)*dim_L0_3[1]
                     + (depth_xyz_len_2+depth_lab_len_2)*dim_L0_3[0]+ (color_xyz_len_2+color_lab_len_2)*dim_L0_3[1]
                     + (depth_xyz_len_3+depth_lab_len_3)*dim_L0_3[0]+ (color_xyz_len_3+color_lab_len_3)*dim_L0_3[1]
                     + (depth_xyz_len_4+depth_lab_len_4)*dim_L0_3[0]+ (color_xyz_len_4+color_lab_len_4)*dim_L0_3[1]
                     + (depth_xyz_len_5+depth_lab_len_5)*dim_L0_3[0]+ (color_xyz_len_5+color_lab_len_5)*dim_L0_3[1]
                    ;
    
    int fea_len = fea_len_L0;// + fea_len_L1;// + fea_len_L2;
    std::cerr << "Total Feature Length: "<< fea_len << std::endl;
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        ObjectSet train_objects, test_objects;
        
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i);
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
        if( train_num == 0 )
        {
            std::cerr << "Empty Set!" << std::endl;
            exit(0);
        }
        
        std::vector< sparseVec> final_train_fea(train_num);
        int final_train_fea_dim = -1;
        //#pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < train_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec;
            
            //cv::Mat mixed_domain;
            //cv::hconcat(train_objects[0][j].uv, train_objects[0][j].rgb, mixed_domain);
            cv::Mat xyz_domain = train_objects[0][j].uv;
            cv::Mat color_domain = train_objects[0][j].rgb;
            
            int count = 0;
            std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(train_objects[0][j], 0);
            {
            cv::Mat temp_0 = depth_lab_pooler_1.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_1.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_1.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_1.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_2.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_2.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_2.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_2.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_3.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_3.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_3.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_3.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_4.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_4.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_4.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_4.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            std::cerr << count << std::endl;
            
            {
            cv::Mat temp_0 = depth_lab_pooler_5.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_5.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_5.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_5.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            std::cerr << count << std::endl;
            cur_fea_003.clear();
            
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
            
        }
        saveCvMatSparse(out_path + "/train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
        train_objects.clear();
        
        
        int test_num = test_objects[0].size();
        std::vector< sparseVec> final_test_fea(test_num);
        int final_test_fea_dim = -1;
        
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < test_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec;
            //cv::Mat mixed_domain;
            //cv::hconcat(test_objects[0][j].uv, test_objects[0][j].rgb, mixed_domain);
            cv::Mat xyz_domain = test_objects[0][j].uv;
            cv::Mat color_domain = test_objects[0][j].rgb;
            
            std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(test_objects[0][j], 0);
            {
            cv::Mat temp_0 = depth_lab_pooler_1.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_1.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_1.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_1.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            }
            {
            cv::Mat temp_0 = depth_lab_pooler_2.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_2.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_2.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_2.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            }
            {
            cv::Mat temp_0 = depth_lab_pooler_3.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_3.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_3.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_3.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            }
            {
            cv::Mat temp_0 = depth_lab_pooler_4.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_4.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_4.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_4.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            }
            
            {
            cv::Mat temp_0 = depth_lab_pooler_5.PoolOneDomain(color_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_5.PoolOneDomain(color_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_5.PoolOneDomain(xyz_domain, cur_fea_003[0], 2, true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_5.PoolOneDomain(xyz_domain, cur_fea_003[1], 2, true);
            pool_fea_vec.push_back(temp_4);
            }
           
            cur_fea_003.clear();
            
            
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            
            final_test_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_test_fea[j] = final_sparse[0];
        }
        saveCvMatSparse(out_path + "/test_"+ss.str()+"_L0.smat", final_test_fea, final_test_fea_dim);
        final_test_fea.clear();
        test_objects.clear();
        
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
    }
    
    return 1;
}
//*/

/*
int main(int argc, char** argv)
{
    //std::string in_path("/home/chi/BigBIRD/processed");
    //std::string out_path("BB_fea_pool");
    //std::string dict_path("BB_new_dict");
    //std::string dict_path("BB_dict");
    
    std::string in_path("/home/chi/UW_RGBD/filtered_pcd");
    std::string out_path("UW_fea_pool");
    std::string dict_path("UW_new_dict");
    //std::string dict_path("UW_dict");
    
    int c1 = 0, c2 = UW_INST_MAX-1;
    float ratio = 0.15;
    int dataset_id = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    Hier_Pooler hie_producer_2(0.02);
    Hier_Pooler hie_producer_3(0.03);
    Hier_Pooler hie_producer_4(0.04);
    Hier_Pooler hie_producer_5(0.05);
    Hier_Pooler hie_producer_6(0.06);
    
    std::vector<int> dim_L0_2 = hie_producer_2.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_3 = hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_4 = hie_producer_4.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_5 = hie_producer_5.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_6 = hie_producer_6.LoadDict_L0(dict_path, "200", "200");
    
    float radius_vec[5] = {0.15, 0.2, 0.25, 0.3, 1.0};
    
    Pooler_L0 depth_xyz_pooler_1(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_2(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_3(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_4(-1);  // 0.03-L0
    Pooler_L0 depth_xyz_pooler_5(-1); // 0.03-L0
    int depth_xyz_len_1 = depth_xyz_pooler_1.LoadHybridPool(dict_path+"/refined_depth_xyz_center_20.cvmat", radius_vec[0]);
    int depth_xyz_len_2 = depth_xyz_pooler_2.LoadHybridPool(dict_path+"/refined_depth_xyz_center_15.cvmat", radius_vec[1]);
    int depth_xyz_len_3 = depth_xyz_pooler_3.LoadHybridPool(dict_path+"/refined_depth_xyz_center_10.cvmat", radius_vec[2]);
    int depth_xyz_len_4 = depth_xyz_pooler_4.LoadHybridPool(dict_path+"/refined_depth_xyz_center_5.cvmat", radius_vec[3]);
    int depth_xyz_len_5 = depth_xyz_pooler_5.LoadHybridPool(dict_path+"/refined_depth_xyz_center_1.cvmat", radius_vec[4]);
    Pooler_L0 depth_lab_pooler_1(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_2(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_3(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_4(-1);  // 0.03-L0
    Pooler_L0 depth_lab_pooler_5(-1); // 0.03-L0
    int depth_lab_len_1 = depth_lab_pooler_1.LoadHybridPool(dict_path+"/refined_depth_lab_center_20.cvmat", radius_vec[0]);
    int depth_lab_len_2 = depth_lab_pooler_2.LoadHybridPool(dict_path+"/refined_depth_lab_center_15.cvmat", radius_vec[1]);
    int depth_lab_len_3 = depth_lab_pooler_3.LoadHybridPool(dict_path+"/refined_depth_lab_center_10.cvmat", radius_vec[2]);
    int depth_lab_len_4 = depth_lab_pooler_4.LoadHybridPool(dict_path+"/refined_depth_lab_center_5.cvmat", radius_vec[3]);
    int depth_lab_len_5 = depth_lab_pooler_5.LoadHybridPool(dict_path+"/refined_depth_lab_center_1.cvmat", radius_vec[4]);
    
    Pooler_L0 color_xyz_pooler_1(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_2(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_3(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_4(-1);  // 0.03-L0
    Pooler_L0 color_xyz_pooler_5(-1); // 0.03-L0
    int color_xyz_len_1 = color_xyz_pooler_1.LoadHybridPool(dict_path+"/refined_color_xyz_center_20.cvmat", radius_vec[0]);
    int color_xyz_len_2 = color_xyz_pooler_2.LoadHybridPool(dict_path+"/refined_color_xyz_center_15.cvmat", radius_vec[1]);
    int color_xyz_len_3 = color_xyz_pooler_3.LoadHybridPool(dict_path+"/refined_color_xyz_center_10.cvmat", radius_vec[2]);
    int color_xyz_len_4 = color_xyz_pooler_4.LoadHybridPool(dict_path+"/refined_color_xyz_center_5.cvmat", radius_vec[3]);
    int color_xyz_len_5 = color_xyz_pooler_5.LoadHybridPool(dict_path+"/refined_color_xyz_center_1.cvmat", radius_vec[4]);
    Pooler_L0 color_lab_pooler_1(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_2(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_3(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_4(-1);  // 0.03-L0
    Pooler_L0 color_lab_pooler_5(-1); // 0.03-L0
    int color_lab_len_1 = color_lab_pooler_1.LoadHybridPool(dict_path+"/refined_color_lab_center_20.cvmat", radius_vec[0]);
    int color_lab_len_2 = color_lab_pooler_2.LoadHybridPool(dict_path+"/refined_color_lab_center_15.cvmat", radius_vec[1]);
    int color_lab_len_3 = color_lab_pooler_3.LoadHybridPool(dict_path+"/refined_color_lab_center_10.cvmat", radius_vec[2]);
    int color_lab_len_4 = color_lab_pooler_4.LoadHybridPool(dict_path+"/refined_color_lab_center_5.cvmat", radius_vec[3]);
    int color_lab_len_5 = color_lab_pooler_5.LoadHybridPool(dict_path+"/refined_color_lab_center_1.cvmat", radius_vec[4]);
   
    
    // for both depth and color features
    int fea_len_L0 = + (depth_xyz_len_1+depth_lab_len_1)*dim_L0_3[0]+ (color_xyz_len_1+color_lab_len_1)*dim_L0_3[1]
                     + (depth_xyz_len_2+depth_lab_len_2)*dim_L0_3[0]+ (color_xyz_len_2+color_lab_len_2)*dim_L0_3[1]
                     + (depth_xyz_len_3+depth_lab_len_3)*dim_L0_3[0]+ (color_xyz_len_3+color_lab_len_3)*dim_L0_3[1]
                     + (depth_xyz_len_4+depth_lab_len_4)*dim_L0_3[0]+ (color_xyz_len_4+color_lab_len_4)*dim_L0_3[1]
                     + (depth_xyz_len_5+depth_lab_len_5)*dim_L0_3[0]+ (color_xyz_len_5+color_lab_len_5)*dim_L0_3[1]
                    ;
    
    int fea_len = fea_len_L0;// + fea_len_L1;// + fea_len_L2;
    std::cerr << "Total Feature Length: "<< fea_len << std::endl;
    
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        ObjectSet train_objects, test_objects;
        
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i);
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
        if( train_num == 0 )
        {
            std::cerr << "Empty Set!" << std::endl;
            exit(0);
        }
        
        std::vector< sparseVec> final_train_fea(train_num);
        int final_train_fea_dim = -1;
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < train_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec;
            
            //cv::Mat mixed_domain;
            //cv::hconcat(train_objects[0][j].uv, train_objects[0][j].rgb, mixed_domain);
            cv::Mat xyz_domain = train_objects[0][j].uv;
            cv::Mat color_domain = train_objects[0][j].rgb;
            
            std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(train_objects[0][j], 0);
            
            int count = 0;
            {
            cv::Mat temp_0 = depth_lab_pooler_1.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_1.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            //std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_1.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_1.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            //std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_2.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_2.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            //std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_2.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_2.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            //std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_3.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_3.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            //std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_3.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_3.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            //std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_4.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_4.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            //std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_4.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_4.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            //std::cerr << count << std::endl;
            
            {
            cv::Mat temp_0 = depth_lab_pooler_5.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            count += temp_0.cols;
            cv::Mat temp_1 = color_lab_pooler_5.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            count += temp_1.cols;
            //std::cerr << count << std::endl;
            cv::Mat temp_3 = depth_xyz_pooler_5.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            count += temp_3.cols;
            cv::Mat temp_4 = color_xyz_pooler_5.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            count += temp_4.cols;
            }
            //std::cerr << count << std::endl;
            
            cur_fea_003.clear();
            
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
            
        }
        saveCvMatSparse(out_path + "/train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
        train_objects.clear();
        
        
        int test_num = test_objects[0].size();
        std::vector< sparseVec> final_test_fea(test_num);
        int final_test_fea_dim = -1;
        
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < test_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec;
            //cv::Mat mixed_domain;
            //cv::hconcat(test_objects[0][j].uv, test_objects[0][j].rgb, mixed_domain);
            cv::Mat xyz_domain = test_objects[0][j].uv;
            cv::Mat color_domain = test_objects[0][j].rgb;
            
            std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(test_objects[0][j], 0);
            
            {
            cv::Mat temp_0 = depth_lab_pooler_1.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_1.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_1.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_1.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            }
            {
            cv::Mat temp_0 = depth_lab_pooler_2.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_2.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_2.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_2.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            }
            //std::cerr << count << std::endl;
            {
            cv::Mat temp_0 = depth_lab_pooler_3.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_3.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_3.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_3.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            }
            {
            cv::Mat temp_0 = depth_lab_pooler_4.PoolHybridDomain(color_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_0);
            cv::Mat temp_1 = color_lab_pooler_4.PoolHybridDomain(color_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_1);
            cv::Mat temp_3 = depth_xyz_pooler_4.PoolHybridDomain(xyz_domain, cur_fea_003[0], true);
            pool_fea_vec.push_back(temp_3);
            cv::Mat temp_4 = color_xyz_pooler_4.PoolHybridDomain(xyz_domain, cur_fea_003[1], true);
            pool_fea_vec.push_back(temp_4);
            }
           
            cur_fea_003.clear();
            
            
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            
            final_test_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_test_fea[j] = final_sparse[0];
        }
        saveCvMatSparse(out_path + "/test_"+ss.str()+"_L0.smat", final_test_fea, final_test_fea_dim);
        final_test_fea.clear();
        test_objects.clear();
        
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
    }
    
    return 1;
}
//*/
/*
int main(int argc, char** argv)
{
    //std::string in_path("/home/chi/BigBIRD/processed");
    //std::string out_path("BB_fea_pool");
    //std::string dict_path("BB_new_dict");
    //std::string dict_path("dict");
    
    std::string in_path("/home/chi/UW_RGBD/filtered_pcd");
    std::string out_path("UW_fea_pool");
    std::string dict_path("UW_new_dict");
    //std::string dict_path("UW_dict");
    
    int c1 = 0, c2 = BB_INST_MAX-1;
    float ratio = 0.15;
    int dataset_id = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    
    Hier_Pooler hie_producer_2(0.02);
    Hier_Pooler hie_producer_3(0.03);
    Hier_Pooler hie_producer_4(0.04);
    Hier_Pooler hie_producer_5(0.05);
    Hier_Pooler hie_producer_6(0.06);
    std::vector<int> dim_L0_2 = hie_producer_2.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_3 = hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_4 = hie_producer_4.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_5 = hie_producer_5.LoadDict_L0(dict_path, "200", "200");
    std::vector<int> dim_L0_6 = hie_producer_6.LoadDict_L0(dict_path, "200", "200");
    
    //std::vector<int> dim_L1 = hie_producer_2.LoadDict_L1(dict_path, K_L1);
    //std::vector<int> dim_L2 = hie_producer_2.LoadDict_L2(dict_path, K_L2);
    
    Pooler_L0 shot_pooler1(5); // 0.02-L0
    Pooler_L0 shot_pooler2(4); // 0.02-L1
    Pooler_L0 shot_pooler3(3); // 0.02-L1
    Pooler_L0 shot_pooler4(2); // 0.02-L2
    Pooler_L0 shot_pooler5(1); // 0.02-L2
    
    // for both depth and color features
    int fea_len_L0 = + shot_pooler1.getHSIPoolLen()*(dim_L0_2[0]+dim_L0_2[1])*2
                     + shot_pooler2.getHSIPoolLen()*(dim_L0_3[0]+dim_L0_3[1])*2
                     + shot_pooler3.getHSIPoolLen()*(dim_L0_4[0]+dim_L0_4[1])*2
                     + shot_pooler4.getHSIPoolLen()*(dim_L0_5[0]+dim_L0_5[1])*2
                     + shot_pooler5.getHSIPoolLen()*(dim_L0_6[0]+dim_L0_6[1])*2
                    ;
    
    //int total_L0 = dim_L0[0] + dim_L0[1];
    //int fea_len_L0 = + shot_pooler1.getHSIPoolLen()*total_L0*2
    //                 + shot_pooler2.getHSIPoolLen()*total_L0*2
    //                 + shot_pooler3.getHSIPoolLen()*total_L0*2
    //                 + shot_pooler4.getHSIPoolLen()*total_L0*2
    //                 + shot_pooler5.getHSIPoolLen()*total_L0*2
    //                ;
    //int fea_len_L0 = shot_pooler1.getHSIPoolLen()*total_L0 + shot_pooler1.getXYPoolLen()*total_L0
    //                 + shot_pooler2.getHSIPoolLen()*total_L0 + shot_pooler2.getXYPoolLen()*total_L0
    //                 + shot_pooler3.getHSIPoolLen()*total_L0 + shot_pooler3.getXYPoolLen()*total_L0
    //                 + shot_pooler4.getHSIPoolLen()*total_L0 + shot_pooler4.getXYPoolLen()*total_L0
    //                 + shot_pooler5.getHSIPoolLen()*total_L0 + shot_pooler5.getXYPoolLen()*total_L0
    //                ;
    
    
    //int total_L1 = dim_L1[0]+dim_L1[1]+dim_L1[2]+dim_L1[3];
    //int fea_len_L1 =  shot_pooler3.getHSIPoolLen()*total_L1*2
    //                + shot_pooler4.getHSIPoolLen()*total_L1*2
    //                + shot_pooler5.getHSIPoolLen()*total_L1*2
    //                ;
    
    //int total_L2_2 = dim_L2_2[0]+dim_L2_2[1]+dim_L2_2[2]+dim_L2_2[3];
    //int fea_len_L2 = + shot_pooler4.getHSIPoolLen()*total_L2_2*2
    //                + shot_pooler5.getHSIPoolLen()*total_L2_2*2
    //                ;
    int fea_len = fea_len_L0;// + fea_len_L1;// + fea_len_L2;
    
    std::cerr << "Total Feature Length: "<< fea_len << std::endl;
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        ObjectSet train_objects, test_objects;
        
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i);
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
        if( train_num == 0 )
        {
            std::cerr << "Empty Set!" << std::endl;
            exit(0);
        }
        //cv::Mat final_train_fea = cv::Mat::zeros(train_num, fea_len, CV_32FC1);
        std::vector< sparseVec> final_train_fea(train_num);
        int final_train_fea_dim = -1;
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < train_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec;
            
            //lab :L0-5
            std::vector<cv::Mat> cur_fea_002 = hie_producer_2.getHierFea(train_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler1.PoolOneDomain(train_objects[0][j].rgb, cur_fea_002[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler1.PoolOneDomain(train_objects[0][j].uv, cur_fea_002[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_002.clear();
            
            //lab :L0-4
            std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(train_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler2.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler2.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_003.clear();
            
            //lab :L0-3
            std::vector<cv::Mat> cur_fea_004 = hie_producer_4.getHierFea(train_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler3.PoolOneDomain(train_objects[0][j].rgb, cur_fea_004[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler3.PoolOneDomain(train_objects[0][j].uv, cur_fea_004[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_004.clear();
            
            //lab :L0-2
            std::vector<cv::Mat> cur_fea_005 = hie_producer_5.getHierFea(train_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler4.PoolOneDomain(train_objects[0][j].rgb, cur_fea_005[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler4.PoolOneDomain(train_objects[0][j].uv, cur_fea_005[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_005.clear();
            
            //lab :L0-2
            std::vector<cv::Mat> cur_fea_006 = hie_producer_6.getHierFea(train_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler5.PoolOneDomain(train_objects[0][j].rgb, cur_fea_006[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler5.PoolOneDomain(train_objects[0][j].uv, cur_fea_006[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_006.clear();
            
            
            //std::cin.get();
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
            
            //final_temp.copyTo(final_trai -jhun_fea.row(j));
        }
        saveCvMatSparse(out_path + "/train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
        train_objects.clear();
        //final_train_fea.release();
        
        int test_num = test_objects[0].size();
        std::vector< sparseVec> final_test_fea(test_num);
        int final_test_fea_dim = -1;
        //cv::Mat final_test_fea = cv::Mat::zeros(test_num, fea_len, CV_32FC1);
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < test_num ; j++ )
        {
            std::vector<cv::Mat> pool_fea_vec;
            //lab :L0-5
            std::vector<cv::Mat> cur_fea_002 = hie_producer_2.getHierFea(test_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler1.PoolOneDomain(test_objects[0][j].rgb, cur_fea_002[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler1.PoolOneDomain(test_objects[0][j].uv, cur_fea_002[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_002.clear();
            
            //lab :L0-4
            std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(test_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler2.PoolOneDomain(test_objects[0][j].rgb, cur_fea_003[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler2.PoolOneDomain(test_objects[0][j].uv, cur_fea_003[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_003.clear();
            
            //lab :L0-3
            std::vector<cv::Mat> cur_fea_004 = hie_producer_4.getHierFea(test_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler3.PoolOneDomain(test_objects[0][j].rgb, cur_fea_004[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler3.PoolOneDomain(test_objects[0][j].uv, cur_fea_004[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_004.clear();
            
            //lab :L0-2
            std::vector<cv::Mat> cur_fea_005 = hie_producer_5.getHierFea(test_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler4.PoolOneDomain(test_objects[0][j].rgb, cur_fea_005[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler4.PoolOneDomain(test_objects[0][j].uv, cur_fea_005[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_005.clear();
            
            //lab :L0-2
            std::vector<cv::Mat> cur_fea_006 = hie_producer_6.getHierFea(test_objects[0][j], 0);
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler5.PoolOneDomain(test_objects[0][j].rgb, cur_fea_006[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            for( int k = 0 ; k < 2 ; k++ )
            {
                cv::Mat temp = shot_pooler5.PoolOneDomain(test_objects[0][j].uv, cur_fea_006[k], 1, true);
                pool_fea_vec.push_back(temp);
            }
            cur_fea_006.clear();
            
            
            cv::Mat final_temp;
            cv::hconcat(pool_fea_vec, final_temp);
            //final_temp.copyTo(final_test_fea.row(j));
            final_test_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_test_fea[j] = final_sparse[0];
        }
        saveCvMatSparse(out_path + "/test_"+ss.str()+"_L0.smat", final_test_fea, final_test_fea_dim);
        final_test_fea.clear();
        test_objects.clear();
        
        //saveCvMatSparse(out_path + "/test_"+ss.str()+"_L0.smat", final_test_fea);
        //final_test_fea.release();
        
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
    }
    
    return 1;
}

//*/


//std::cerr << count << std::endl;
            //{
            //    cv::Mat normal_idx = shot_pooler1.getGenericPoolMat(train_objects[0][j].normal);
            //    for( int k = 0 ; k < 2 ; k++ )
            //    {
            //        cv::Mat temp = shot_pooler1.PoolOneDomain(normal_idx, cur_fea_003[k], 3, true);
            //        pool_fea_vec.push_back(temp);
            //        count += temp.cols;
            //    }
            //}

/*
             
            //std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(train_objects[0][j], 0);
            /
            {
                //std::vector<cv::Mat> cur_fea_002 = hie_producer_2.getHierFea(train_objects[0][j], 0);
                
                //lab :L0-8
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler1.PoolOneDomain(train_objects[0][j].rgb, cur_fea_002[k], 1, true);
                    cv::Mat temp = shot_pooler1.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler1.PoolOneDomain(train_objects[0][j].uv, cur_fea_002[k], 1, true);
                    cv::Mat temp = shot_pooler1.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }

                //lab :L0-7
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler2.PoolOneDomain(train_objects[0][j].rgb, cur_fea_002[k], 1, true);
                    cv::Mat temp = shot_pooler2.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler2.PoolOneDomain(train_objects[0][j].uv, cur_fea_002[k], 1, true);
                    cv::Mat temp = shot_pooler2.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                
            }
            
            {
                std::vector<cv::Mat> cur_fea_003 = hie_producer_2.getHierFea(train_objects[0][j], 0);
                //lab :L0-6
                for( int k = 0 ; k < 2 ; k++ )
                {
                    cv::Mat temp = shot_pooler3.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    cv::Mat temp = shot_pooler3.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }

                //lab :L0-5
                for( int k = 0 ; k < 2 ; k++ )
                {
                    cv::Mat temp = shot_pooler4.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    cv::Mat temp = shot_pooler4.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                
            }
            {   
                std::vector<cv::Mat> cur_fea_003 = hie_producer_3.getHierFea(train_objects[0][j], 0);
                //std::cerr << count << std::endl;
                //lab :L0-4
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler5.PoolOneDomain(train_objects[0][j].rgb, cur_fea_004[k], 1, true);
                    cv::Mat temp = shot_pooler5.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler5.PoolOneDomain(train_objects[0][j].uv, cur_fea_004[k], 1, true);
                    cv::Mat temp = shot_pooler5.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
            }
            {
                std::vector<cv::Mat> cur_fea_003 = hie_producer_4.getHierFea(train_objects[0][j], 0);
                //lab :L0-3
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler6.PoolOneDomain(train_objects[0][j].rgb, cur_fea_004[k], 1, true);
                    cv::Mat temp = shot_pooler6.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler6.PoolOneDomain(train_objects[0][j].uv, cur_fea_004[k], 1, true);
                    cv::Mat temp = shot_pooler6.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                
            }
            {
                //lab :L0-2
                std::vector<cv::Mat> cur_fea_003 = hie_producer_5.getHierFea(train_objects[0][j], 0);
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler7.PoolOneDomain(train_objects[0][j].rgb, cur_fea_005[k], 1, true);
                    cv::Mat temp = shot_pooler7.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler7.PoolOneDomain(train_objects[0][j].uv, cur_fea_005[k], 1, true);
                    cv::Mat temp = shot_pooler7.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
            }
            
            {
                //lab :L0-1
                std::vector<cv::Mat> cur_fea_003 = hie_producer_6.getHierFea(train_objects[0][j], 0);
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler8.PoolOneDomain(train_objects[0][j].rgb, cur_fea_006[k], 1, true);
                    cv::Mat temp = shot_pooler8.PoolOneDomain(train_objects[0][j].rgb, cur_fea_003[k], 1, true);
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
                //std::cerr << count << std::endl;
                for( int k = 0 ; k < 2 ; k++ )
                {
                    //cv::Mat temp = shot_pooler8.PoolOneDomain(train_objects[0][j].uv, cur_fea_006[k], 1, true);
                    cv::Mat temp = shot_pooler8.PoolOneDomain(train_objects[0][j].uv, cur_fea_003[k], 1, true);    
                    pool_fea_vec.push_back(temp);
                    count += temp.cols;
                }
              
            }
 
 
 
 */
