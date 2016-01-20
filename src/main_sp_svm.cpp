#include "sp_segmenter/features.h"

model* TrainMultiSVM(std::string fea_path, int l1, int l2, float CC, bool tacc_flag);
//model* TrainBinarySVM(std::string fea_path, int l1, int l2, float CC, bool tacc_flag);
model* TrainBinarySVM(std::string fea_path, std::vector<std::string> background_path, int l1, int l2, float CC, bool tacc_flag);

#define HARD
#ifndef HARD

int main(int argc, char** argv)
{
    std::string in_path("../../data_pool/ht10_small_fea/");
    std::string scene_name("office");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    
    std::string out_path(scene_name +"_svm/");
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    boost::filesystem::create_directories(out_path);
    
    std::vector<std::string> background_path;
    background_path.push_back(in_path + scene_name + "_background/");
    background_path.push_back(in_path + scene_name + "_raw/");
    background_path.push_back(in_path + scene_name + "_imposter/");
    
    float CCB = 0.01, CCM = 0.01;
    pcl::console::parse_argument(argc, argv, "--CCB", CCB);
    pcl::console::parse_argument(argc, argv, "--CCM", CCM);
    
    for( int ll = 0 ; ll <= 2 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;

        if( ll == 0 )
            CCB = 0.01;
        else if( ll == 1 )
            CCB = 0.001;
        else
            CCB = 0.0001;

        model *cur_model = TrainBinarySVM(in_path, background_path, ll, ll, CCB, true);
        save_model((out_path + "binary_L"+ss.str()+".model").c_str(), cur_model);
    }
    
    for( int ll = 0 ; ll <= 4 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;
        
        model *cur_model = TrainMultiSVM(in_path, ll, ll, CCM, true);
        save_model((out_path + "multi_L"+ss.str()+".model").c_str(), cur_model);
    }
    
    return 1;
} 

#else

int main(int argc, char** argv)
{
//    std::string in_path("../../data_pool/ht10_small_fea/");
//    std::string scene_name("office");
    std::string in_path("../../data_pool/UR5/");
    std::string scene_name("UR5");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    
    std::string out_path(scene_name +"_svm/");
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    boost::filesystem::create_directories(out_path);
    
    std::vector<std::string> background_path;
    background_path.push_back(in_path + scene_name + "_background/");
//    background_path.push_back(in_path + scene_name + "_background/");
//    background_path.push_back(in_path + scene_name + "_raw/");
//    background_path.push_back(in_path + scene_name + "_imposter/");
//    background_path.push_back(out_path + "hard/");
    
    float CCB = 0.01, CCM = 0.01;
    pcl::console::parse_argument(argc, argv, "--CCB", CCB);
    pcl::console::parse_argument(argc, argv, "--CCM", CCM);
    
    for( int ll = 0 ; ll <= 2 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;

        if( ll == 0 )
            CCB = 0.1;
        else if( ll == 1 )
            CCB = 0.01;
        else
            CCB = 0.001;

        model *cur_model = TrainBinarySVM(in_path, background_path, ll, ll, CCB, true);
        save_model((out_path + "binary_L"+ss.str()+"_f.model").c_str(), cur_model);
    }
   
    return 1;
} 

#endif

model* TrainMultiSVM(std::string fea_path, int l1, int l2, float CC, bool tacc_flag)
{
    std::vector< std::pair<int, int> > piece_inds;
//    piece_inds.push_back(std::pair<int, int> (0, 130000));
    std::vector<problem> train_prob_set;
    for( int i = 1 ; i <= 10  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        for( int ll = l1 ; ll <= l2 ; ll++ )
        {
            std::stringstream mm;
            mm << ll;
        
            std::string train_name = fea_path + "train_"+ss.str()+"_L"+mm.str()+".smat";
            if( exists_test(train_name) == false )
                continue;
            
            std::cerr << "Reading: " << train_name << std::endl;
            std::vector<SparseDataOneClass> cur_data(1);
            int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds);
            cur_data[0].label = i; 

            problem tmp;
            FormFeaSparseMat(cur_data, tmp, cur_data[0].fea_vec.size(), fea_dim);
            train_prob_set.push_back(tmp);
        }
    }
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    parameter param;
    GenSVMParamter(param, CC);
    std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;
    
    std::stringstream gg1, gg2;
    gg1 << l1;
    gg2 << l2;
    model* cur_model = train(&train_prob, &param);
//    save_model((out_path+"multi_"+gg1.str()+"_"+gg2.str()+".model").c_str(), cur_model);
    
    if ( tacc_flag == true ){
    	int corr_t[OBJ_INST_MAX] = {0}, count_t[OBJ_INST_MAX] = {0};
    	for(int j = 0 ; j < train_prob.l ; j++ )
    	{
            int true_label = floor(train_prob.y[j]+0.0001);
            count_t[true_label]++;
            double pred_tmp = predict(cur_model, train_prob.x[j]);
            int pred_label = floor(pred_tmp+0.0001);
            if( pred_label == true_label )
                corr_t[true_label]++;
    	}
    	int total_corr = 0;
    	for( int b1 = 1 ; b1 <= 10 ; b1++ )
    	{
            total_corr += corr_t[b1];
            std::cerr<<b1<<" "<<count_t[b1]<<" "<<corr_t[b1]/(count_t[b1]+0.0)<<std::endl;
    	}
    	std::cerr<<"Final Multi Training Accuracy: "<<(total_corr+0.0)/train_prob.l<<std::endl;
    }
    
    destroy_param(&param);
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    return cur_model;
}

model* TrainBinarySVM(std::string fea_path, std::vector<std::string> background_path, int l1, int l2, float CC, bool tacc_flag)
{
    std::vector< std::pair<int, int> > piece_inds1;
    std::vector< std::pair<int, int> > piece_inds2;
    piece_inds2.push_back(std::pair<int, int> (0, 90000));
    std::vector<problem> train_prob_set;
    
    for( size_t i = 0 ; i < background_path.size()  ; i++ )
    {
        for( int ll = l1 ; ll <= l2 ; ll++ )
        {
            std::stringstream mm;
            mm << ll;
        
            std::string train_name = background_path[i] + "train_0_L"+mm.str()+".smat";
            if( exists_test(train_name) == false )
                continue;
            
            std::cerr << "Reading: " << train_name << std::endl;
            std::vector<SparseDataOneClass> cur_data(1);
            
            int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds1); 
                                 
            cur_data[0].label = 1; 

            problem tmp;
            FormFeaSparseMat(cur_data, tmp, cur_data[0].fea_vec.size(), fea_dim);
            train_prob_set.push_back(tmp);
        }
    }
    
    for( int i = 1 ; i <= 10  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        for( int ll = l1 ; ll <= l2 ; ll++ )
        {
            std::stringstream mm;
            mm << ll;
        
            std::string train_name = fea_path + "train_"+ss.str()+"_L"+mm.str()+".smat";
            if( exists_test(train_name) == false )
                continue;
            
            std::cerr << "Reading: " << train_name << std::endl;
            std::vector<SparseDataOneClass> cur_data(1);
            
            int fea_dim = i == 0 ? readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds1) 
                                 : readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds2);
            cur_data[0].label = i == 0 ? 1 : 2; 

            problem tmp;
            FormFeaSparseMat(cur_data, tmp, cur_data[0].fea_vec.size(), fea_dim);
            train_prob_set.push_back(tmp);
        }
    }
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    parameter param;
    GenSVMParamter(param, CC);
    std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;
    
    std::stringstream gg1, gg2;
    gg1 << l1;
    gg2 << l2;
    model* cur_model = train(&train_prob, &param);
//    save_model((out_path+"binary_"+gg1.str()+"_"+gg2.str()+".model").c_str(), cur_model);
    
    if ( tacc_flag == true ){
    	int corr_t[OBJ_INST_MAX] = {0}, count_t[OBJ_INST_MAX] = {0};
    	for(int j = 0 ; j < train_prob.l ; j++ )
    	{
            int true_label = floor(train_prob.y[j]+0.0001 - 1);
            count_t[true_label]++;
            double pred_tmp = predict(cur_model, train_prob.x[j]);
            int pred_label = floor(pred_tmp+0.0001 - 1);
            if( pred_label == true_label )
                corr_t[true_label]++;
    	}
    	int total_corr = 0;
    	for( int b1 = 0 ; b1 <= 1 ; b1++ )
    	{
            total_corr += corr_t[b1];
            std::cerr<<b1<<" "<<count_t[b1]<<" "<<corr_t[b1]/(count_t[b1]+0.0)<<std::endl;
    	}
    	std::cerr<<"Final Binary Training Accuracy: "<<(total_corr+0.0)/train_prob.l<<std::endl;
    }
    
    destroy_param(&param);
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    return cur_model;
}



/*model* TrainLinearSVM(std::string fea_path, int c1, int c2, float CC, bool tacc_flag)
{
    std::vector< std::pair<int, int> > piece_inds;
    std::vector<problem> train_prob_set;
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        std::string train_name = fea_path + "train_"+ss.str()+"_L0.smat";
        
        std::cerr << "Reading: " << train_name << std::endl;
        std::vector<SparseDataOneClass> cur_data(1);
        int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds);
        cur_data[0].label = i; 

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
    save_model((fea_path+"svm_model.model").c_str(), cur_model);
    
    if ( tacc_flag == true ){
    	int corr_t[OBJ_INST_MAX] = {0}, count_t[OBJ_INST_MAX] = {0};
    	for(int j = 0 ; j < train_prob.l ; j++ )
    	{
            int true_label = floor(train_prob.y[j]+0.0001);
            count_t[true_label]++;
            double pred_tmp = predict(cur_model, train_prob.x[j]);
            int pred_label = floor(pred_tmp+0.0001);
            if( pred_label == true_label )
                corr_t[true_label]++;
    	}
    	int total_corr = 0;
    	for( int b1 = c1 ; b1 <= c2 ; b1++ )
    	{
            total_corr += corr_t[b1];
            std::cerr<<b1<<" "<<count_t[b1]<<" "<<corr_t[b1]/(count_t[b1]+0.0)<<std::endl;
    	}
    	std::cerr<<"Final Training Accuracy: "<<(total_corr+0.0)/train_prob.l<<std::endl;
    }
    
    destroy_param(&param);
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    return cur_model;
}*/

/*for( int i = 0 ; i <= 9  ; i++ )
    {
        continue;
        std::stringstream ss;
        ss << i+1;
        
        if( exists_test(out_path + "train_"+ss.str()+"_L0.smat") == true )
            continue;
        
        ObjectSet train_objects, test_objects;
        readJHUInstWithImg(model_path, train_objects, test_objects, i, i, true);
        std::cerr << "Loading Completed... " << std::endl;
        
        int train_num = train_objects[0].size();
        std::cerr << "Train " << i << " --- " << train_num << std::endl;
        
        if( train_num > 0 )
        {
            std::vector< sparseVec> final_train;
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
                triple_pooler.build_SP_SIFT(sift_pooler_set, hie_producer, sigma, false);
                
                std::vector<cv::Mat> sp_fea_L2 = triple_pooler.sampleSPFea(2, box_num);
                std::vector<cv::Mat> sp_fea_L3 = triple_pooler.sampleSPFea(3, box_num);
                std::vector<cv::Mat> sp_fea_L4 = triple_pooler.sampleSPFea(4, box_num);
                
                std::vector<cv::Mat> sp_fea;
                sp_fea.insert(sp_fea.end(), sp_fea_L2.begin(), sp_fea_L2.end());
                sp_fea.insert(sp_fea.end(), sp_fea_L3.begin(), sp_fea_L3.end());
                sp_fea.insert(sp_fea.end(), sp_fea_L4.begin(), sp_fea_L4.end());
                for( std::vector<cv::Mat>::iterator it = sp_fea.begin(); it < sp_fea.end() ; it++ )
                {
                    if( fea_dim > 0 && it->cols != fea_dim )
                    {
                        std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << it->cols << std::endl;
                        exit(0);
                    }
                    else if( fea_dim < 0 )
                    {
                        #pragma omp critical
                        {
                            fea_dim = it->cols;
                            std::cerr << "Fea Dim: " << fea_dim << std::endl;
                        }
                    }	
                    std::vector< sparseVec> this_sparse;
                    sparseCvMat(*it, this_sparse);
                    #pragma omp critical
                    {
                        final_train.push_back(this_sparse[0]);
                    }
                }
                
            }
            
            saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train, fea_dim);
            final_train.clear();
        }
        else
        {
            std::cerr << "JHU Model Data Reading Failed!" << std::endl;
            exit(0);
        }
        train_objects.clear();
    }*/
