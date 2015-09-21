#include <opencv2/core/core.hpp>

#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

/*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = -1;
    float CC_L1 = 100.0, CC_L2 = 0.1;
    
    std::string workspace("../data_pool/ln_3x3_kdtree/");
    pcl::console::parse_argument(argc, argv, "--p", workspace);
    pcl::console::parse_argument(argc, argv, "--C", CC_L2);
    
    std::string fea_path(workspace + "fea/");
    std::string out_path = workspace + "SVM_Model/";  
    std::string out_file(out_path + "svm_model.model");
    
    if( exists_dir(fea_path) == false )
        return 0;    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    bool tacc_flag = false;
    if( pcl::console::find_switch(argc, argv, "-tacc") == true )
        tacc_flag = true;   
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    std::vector< std::pair<int, int> > piece_inds;
    
    std::vector<problem> train_prob_set;
    // Loading Background Model
    
    std::pair<float, float> cur_box = readBoxFile( fea_path + "model.box");
    std::cerr << "Training Box: " << cur_box.first << "-" << cur_box.second << std::endl;
    
    int label_count = 1;
    // Loading Object Model From all Training and Testing Data
    for( int i = c1 ; i <= c2  ; i++, label_count++ )
    {
        std::stringstream ss;
        ss << i;
        
        for( int j = 0 ; j < 10 ; j++ )
        {
            std::stringstream tt;
            tt << j;
        
            std::string train_name(fea_path + "train_"+ss.str()+"_L"+ tt.str()+ ".smat");
            std::string test_name(fea_path + "test_"+ss.str()+"_L"+ tt.str()+ ".smat");
            
            if( exists_test(train_name) == true && exists_test(test_name) == true )
            {
                std::cerr << "Reading: " << train_name << std::endl;
                std::cerr << "Reading: " << test_name << std::endl;
                std::vector<SparseDataOneClass> cur_train(1), cur_test(1);
                int fea_dim_1 = readSoluSparse_piecewise(fea_path + "train_"+ss.str()+"_L"+tt.str()+".smat", cur_train[0].fea_vec, piece_inds);
                int fea_dim_2 = readSoluSparse_piecewise(fea_path + "test_" +ss.str()+"_L"+tt.str()+".smat", cur_test[0].fea_vec, piece_inds);
                
                cur_train[0].label = label_count;       // leave background class as label 1, all objects are above label 1
                cur_test[0].label = label_count;
                
                problem tmp1, tmp2;
                FormFeaSparseMat(cur_train, tmp1, cur_train[0].fea_vec.size(), fea_dim_1);
                train_prob_set.push_back(tmp1);
                FormFeaSparseMat(cur_test, tmp2, cur_test[0].fea_vec.size(), fea_dim_2);
                train_prob_set.push_back(tmp2);
                
            }
            else
                break;
            
        }
    }
    
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    std::cerr<< "Train Num: " << train_prob.l << std::endl;
    std::cerr<< "Fea Dim: " << train_prob.n << std::endl;
    
    parameter param_L1;
    GenSVMParamter(param_L1, CC_L1);
    param_L1.solver_type = L1R_L2LOSS_SVC;
   
    std::cerr<<std::endl<<"Starting L1 Training..."<<std::endl;
    
    model* L1_model = train(&train_prob, &param_L1);
    std::vector<bool> valid_dim(L1_model->nr_feature, false);
    int count = 0;
    for(int i = 0 ; i < L1_model->nr_feature ; i++ )
        if( L1_model->w[i] != 0 )
        {
            count++;
            valid_dim[i] = true;
        }
    std::cerr << "COUNT: " << count << std::endl;
    //disable train_prob's invalid dims
    for(int i = 0 ; i < train_prob.l ; i++ )
    {
        int idx = 0;
        while(true)
        {
            //std::cerr << idx << " ";
            int cur_idx = train_prob.x[i][idx].index;
            if( cur_idx == -1 )
                break;
            else if( valid_dim[cur_idx] == false )
                train_prob.x[i][idx].value = 0.0;
            idx++;
        }
    }
    free_and_destroy_model(&L1_model);
      
    parameter param_L2;
    GenSVMParamter(param_L2, CC_L2);
    
    std::cerr<<std::endl<<"Starting L2 Training..."<<std::endl;
    model* L2_model = train(&train_prob, &param_L2);
       
    save_model((out_file).c_str(), L2_model);
    if ( tacc_flag == true ){
    	int corr_t[OBJ_INST_MAX] = {0}, count_t[OBJ_INST_MAX] = {0};
    	for(int j = 0 ; j < train_prob.l ; j++ )
    	{
        	int true_label = floor(train_prob.y[j]+0.001-1);
        	count_t[true_label]++;
        	double pred_tmp = predict(L2_model, train_prob.x[j]);
        	int pred_label = floor(pred_tmp+0.001-1);
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
    
    free_and_destroy_model(&L2_model);

    //Deallocation
    destroy_param(&param_L1);
    destroy_param(&param_L2);
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    
    std::ofstream fp((out_path + "model.box").c_str());
    
    fp << cur_box.first << " " << cur_box.second << std::endl;
    fp.close();
    
    return 1;
}

//*/

//*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = -1;
    float CC = 0.1;
    
    std::string workspace("../data_pool/ln_3x3_aux/");
    
    pcl::console::parse_argument(argc, argv, "--C", CC);
    pcl::console::parse_argument(argc, argv, "--p", workspace);
    
    std::string fea_path(workspace + "fea/");
    std::string out_path = workspace + "SVM_Model/";  
    std::string out_file(out_path + "svm_model.model");
    
    if( exists_dir(fea_path) == false )
        return 0;    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    bool tacc_flag = false;
    if( pcl::console::find_switch(argc, argv, "-tacc") == true )
        tacc_flag = true;   
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    std::vector< std::pair<int, int> > piece_inds;
    
    std::vector<problem> train_prob_set;
    // Loading Background Model
    
    if( exists_test(fea_path + "model.box") == true )
    { 
        std::pair<float, float> cur_box = readBoxFile( fea_path + "model.box");
        std::cerr << "Training Box: " << cur_box.first << "-" << cur_box.second << std::endl;
        
        std::ofstream fp((out_path + "model.box").c_str());
        fp << cur_box.first << " " << cur_box.second << std::endl;
        fp.close();
    }
    
    int label_count = 1;
    // Loading Object Model From all Training and Testing Data
    for( int i = c1 ; i <= c2  ; i++, label_count++ )
    {
        std::stringstream ss;
        ss << i;
        
        for( int j = 0 ; j < 10 ; j++ )
        {
            std::stringstream tt;
            tt << j;
        
            std::string train_name(fea_path + "train_"+ss.str()+"_L"+ tt.str()+ ".smat");
            std::string test_name(fea_path + "test_"+ss.str()+"_L"+ tt.str()+ ".smat");
            
            if( exists_test(train_name) == true ) //&& exists_test(test_name) == true )
            {
                std::cerr << "Reading: " << train_name << std::endl;
                std::vector<SparseDataOneClass> cur_data(1);
                int fea_dim = readSoluSparse_piecewise(train_name, cur_data[0].fea_vec, piece_inds);
                cur_data[0].label = label_count; 
                
                problem tmp;
                FormFeaSparseMat(cur_data, tmp, cur_data[0].fea_vec.size(), fea_dim);
                train_prob_set.push_back(tmp);
            }
            if( exists_test(test_name) == true ) //&& exists_test(test_name) == true )
            {
                std::cerr << "Reading: " << test_name << std::endl;
                std::vector<SparseDataOneClass> cur_data(1);
                int fea_dim = readSoluSparse_piecewise(test_name, cur_data[0].fea_vec, piece_inds);
                cur_data[0].label = label_count; 
                
                problem tmp;
                FormFeaSparseMat(cur_data, tmp, cur_data[0].fea_vec.size(), fea_dim);
                train_prob_set.push_back(tmp);
            }
        }
    }
    
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    std::cerr<< "Train Num: " << train_prob.l << std::endl;
    std::cerr<< "Fea Dim: " << train_prob.n << std::endl;
    
    parameter param;
    GenSVMParamter(param, CC);
    //param.solver_type = L1R_L2LOSS_SVC;
    
    std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;
    
    model* cur_model = train(&train_prob, &param);
    save_model((out_file).c_str(), cur_model);
    
    if ( tacc_flag == true ){
    	int corr_t[OBJ_INST_MAX] = {0}, count_t[OBJ_INST_MAX] = {0};
    	for(int j = 0 ; j < train_prob.l ; j++ )
    	{
        	int true_label = floor(train_prob.y[j]+0.001);
        	count_t[true_label]++;
        	double pred_tmp = predict(cur_model, train_prob.x[j]);
        	int pred_label = floor(pred_tmp+0.001);
        	if( pred_label == true_label )
            		corr_t[true_label]++;
    	}
    	int total_corr = 0;
    	for( int b1 = 1 ; b1 <= c2 - c1 + 1 ; b1++ )
    	{
        	total_corr += corr_t[b1];
        	std::cerr<<b1<<" "<<count_t[b1]<<" "<<corr_t[b1]/(count_t[b1]+0.0)<<std::endl;
    	}
    	std::cerr<<"Final Training Accuracy: "<<(total_corr+0.0)/train_prob.l<<std::endl;
    }
    
    free_and_destroy_model(&cur_model);

    //Deallocation
    destroy_param(&param);
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    
    return 1;
}

//*/
   

/*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = -1;
    float CC = 0.1;
    
    //std::string obj_path("../data_pool/JHU_newbox_4layer/");
    //std::string aux_obj_path("../data_pool/JHU_newbox_add/");
    
    std::string workspace("../data_pool/JHU_box4x6_s30/");
    
    std::string ori_fea_path(workspace + "JHU_fea_ori/");
    std::string aux_aux_path(workspace + "JHU_fea_aux/");
    std::string background_path("../data_pool/JHU_train_background/");
    std::string out_path("../data_pool/SVM_Model/");
    std::string out_file("svm_model.model");
    pcl::console::parse_argument(argc, argv, "--C", CC);
    pcl::console::parse_argument(argc, argv, "--p", obj_path);
    pcl::console::parse_argument(argc, argv, "--o", out_file);
    if( exists_dir(obj_path) == false )
        return 0;    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);

    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    std::vector< std::pair<int, int> > piece_inds;
    
    std::vector<problem> train_prob_set;
    
    // Loading Background Model
    //boost::filesystem::path p(background_path);
    //std::vector< std::string > ret;
    //find_files(p, ".smat", ret);
    
    int background_label = 1;
    for( std::vector< std::string >::iterator it_p = ret.begin() ; it_p < ret.end() ; it_p++ )
    {
        std::cerr << "Reading: " << background_path + *it_p << std::endl;
        std::vector<SparseDataOneClass> cur_train(1);
        int fea_dim = readSoluSparse_piecewise(background_path + *it_p, cur_train[0].fea_vec, piece_inds);
        
        std::cerr << background_label << " " << fea_dim << std::endl;
        cur_train[0].label = background_label;       // leave background class as label 1, all objects are above label 1
        
        problem tmp1;
        FormFeaSparseMat(cur_train, tmp1, cur_train[0].fea_vec.size(), fea_dim);
        train_prob_set.push_back(tmp1);
    }
    
    int label_count = 1;
    // Loading Object Model From all Training and Testing Data
    for( int i = c1 ; i <= c2  ; i++, label_count++ )
    {
        std::stringstream ss;
        ss << i;
        
        std::cerr << "Reading: " << obj_path + "train_"+ss.str()+"_L0.smat" << std::endl;
        std::cerr << "Reading: " << obj_path + "test_"+ss.str()+"_L0.smat" << std::endl;
        std::vector<SparseDataOneClass> cur_train(1), cur_test(1);
        int fea_dim_1 = readSoluSparse_piecewise(obj_path + "train_"+ss.str()+"_L0.smat", cur_train[0].fea_vec, piece_inds);
        int fea_dim_2 = readSoluSparse_piecewise(obj_path + "test_"+ss.str()+"_L0.smat", cur_test[0].fea_vec, piece_inds);
        
        if( fea_dim_1 != fea_dim_2 )
        {
            std::cerr << "fea_dim_1 != fea_dim_2!" << std::endl;
            exit(0);
        }
        
        cur_train[0].label = label_count;       // leave background class as label 1, all objects are above label 1
        cur_test[0].label = label_count;
        {
            problem tmp1, tmp2;
            FormFeaSparseMat(cur_train, tmp1, cur_train[0].fea_vec.size(), fea_dim_1);
            train_prob_set.push_back(tmp1);
            FormFeaSparseMat(cur_test, tmp2, cur_test[0].fea_vec.size(), fea_dim_2);
            train_prob_set.push_back(tmp2);
        }
        /*
        std::vector<SparseDataOneClass> aux_train(1), aux_test(1);
        int aux_train_dim = readSoluSparse_piecewise(aux_obj_path + "train_"+ss.str()+"_L0.smat", aux_train[0].fea_vec, piece_inds);
        if( aux_train_dim == fea_dim_1 )
        {
            std::cerr << "Reading: " << aux_obj_path + "train_"+ss.str()+"_L0.smat" << std::endl;
            aux_train[0].label = label_count;
            problem tmp;
            FormFeaSparseMat(aux_train, tmp, aux_train[0].fea_vec.size(), aux_train_dim);
            train_prob_set.push_back(tmp);
        }
        int aux_test_dim = readSoluSparse_piecewise(aux_obj_path + "test_"+ss.str()+"_L0.smat", aux_test[0].fea_vec, piece_inds);
        if( aux_test_dim == fea_dim_1 )
        {
            std::cerr << "Reading: " << aux_obj_path + "test_"+ss.str()+"_L0.smat" << std::endl;
            aux_test[0].label = label_count;
            problem tmp;
            FormFeaSparseMat(aux_test, tmp, aux_test[0].fea_vec.size(), aux_test_dim);
            train_prob_set.push_back(tmp);
        }
        
        std::cerr << i << " " << fea_dim_1 << std::endl;
    }
    
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    std::cerr<< "Train Num: " << train_prob.l << std::endl;
    std::cerr<< "Fea Dim: " << train_prob.n << std::endl;
    
    parameter param;
    GenSVMParamter(param, CC);
    
    param.nr_weight = 3;
    param.weight_label = new int[param.nr_weight];
    param.weight = new double[param.nr_weight];
    param.weight_label[0] = 1;
    param.weight[0] = 1;
    param.weight_label[1] = 2;
    param.weight[1] = 10;
    param.weight_label[2] = 3;
    param.weight[2] = 10;
    
    std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;
    
    model* cur_model = train(&train_prob, &param);
    save_model((out_path + out_file).c_str(), cur_model);
    
    free_and_destroy_model(&cur_model);

    //Deallocation
    destroy_param(&param);
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    
    return 1;
}
*/

