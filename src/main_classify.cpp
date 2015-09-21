#include "../include/index.h"

int main(int argc, char** argv)
{   
    int c1 = 0, c2 = BB_INST_MAX-1;
    float CC = 0.001;
    
    std::string in_path("BB_fea_pool/");
    pcl::console::parse_argument(argc, argv, "--C", CC);
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    if( exists_dir(in_path) == false )
        return 0;    

    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    int fs = 0, fe = -1;
    pcl::console::parse_argument(argc, argv, "--fs", fs);
    pcl::console::parse_argument(argc, argv, "--fe", fe);
    
//    std::string id("");
//    pcl::console::parse_argument(argc, argv, "--i", id); 

    std::vector< std::pair<int, int> > piece_inds;
    //piece_inds.push_back(std::pair<int, int> (156000, 157600));
    if( fs < fe )
    	piece_inds.push_back(std::pair<int, int> (fs, fe));
    //piece_inds.push_back(std::pair<int, int> (200,400));
    //piece_inds.push_back(std::pair<int, int> (2000, 3600)); 
    //piece_inds.push_back(std::pair<int, int> (9000, 14400)); 
    //piece_inds.push_back(std::pair<int, int> (27200, 40000));
    
    //piece_inds.push_back(std::pair<int, int> (65000,90000));
    //piece_inds.push_back(std::pair<int, int> (409600, 546800)); 
    //piece_inds.push_back(std::pair<int, int> (684000, 770400)); 
    //piece_inds.push_back(std::pair<int, int> (856800, 906800));
    //piece_inds.push_back(std::pair<int, int> (956800, 982400)); 
    //piece_inds.push_back(std::pair<int, int> (1008000, 1018800)); 
    //piece_inds.push_back(std::pair<int, int> (1029600, 1032800));
    //piece_inds.push_back(std::pair<int, int> (1036000, 1036400));
    
    //piece_inds.push_back(std::pair<int, int> (204800, 409600));
    //piece_inds.push_back(std::pair<int, int> (546800, 684000)); 
    //piece_inds.push_back(std::pair<int, int> (770400, 856800)); 
    //piece_inds.push_back(std::pair<int, int> (906800, 956800));
    //piece_inds.push_back(std::pair<int, int> (982400, 1008000)); 
    //piece_inds.push_back(std::pair<int, int> (1018800, 1029600)); 
    //piece_inds.push_back(std::pair<int, int> (1032800, 1036000));
    //piece_inds.push_back(std::pair<int, int> (1036400, 1036800));
    
    
    bool tacc_flag = false;
    if( pcl::console::find_switch(argc, argv, "-tacc") == true )
        tacc_flag = true;    

    size_t total_train, total_test;
    
    std::vector<problem> train_prob_set;
    for(int id = 0 ; id < 10 ; id++ )
    {
        std::stringstream id_str;
        id_str << id;
    
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        if(exists_test( in_path + "train_"+ss.str()+"_L"+id_str.str()+".smat" )==false)
	     continue;
	
        std::cerr << "Reading: " << in_path + "train_"+ss.str()+"_L"+id_str.str()+".smat" << std::endl;
        std::vector<SparseDataOneClass> cur_train(1);
        int fea_dim = readSoluSparse_piecewise(in_path + "train_"+ss.str()+"_L"+id_str.str()+".smat", cur_train[0].fea_vec, piece_inds);
        
        std::cerr << i << " " << fea_dim << std::endl;
        cur_train[0].label = i+1;
        
   	problem tmp_problem;
        FormFeaSparseMat(cur_train, tmp_problem, cur_train[0].fea_vec.size(), fea_dim);
	train_prob_set.push_back(tmp_problem);
    }
    
    }
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    total_train = train_prob.l;
    std::cerr<< "Fea Dim: " << train_prob.n << std::endl;
    
    parameter param;
    GenSVMParamter(param, CC);
    std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;
    
    model* cur_model = train(&train_prob, &param);
    std::cerr << cur_model->bias << " " << cur_model->nr_feature << std::endl;
    save_model((in_path + "/svm_model.model").c_str(), cur_model);
    
    int total_corr;
    // Computing traininig and testing accuracies
    if ( tacc_flag == true ){
    	int corr_t[OBJ_INST_MAX] = {0}, count_t[OBJ_INST_MAX] = {0};
    	for(int j = 0 ; j < train_prob.l ; j++ )
    	{
        	int true_label = floor(train_prob.y[j]+0.001-1);
        	count_t[true_label]++;
        	double pred_tmp = predict(cur_model, train_prob.x[j]);
        	int pred_label = floor(pred_tmp+0.001-1);
        	if( pred_label == true_label )
            		corr_t[true_label]++;
    	}
    	total_corr = 0;
    	for( int b1 = c1 ; b1 <= c2 ; b1++ )
    	{
        	total_corr += corr_t[b1];
        	std::cerr<<b1<<" "<<count_t[b1]<<" "<<corr_t[b1]/(count_t[b1]+0.0)<<std::endl;
    	}
    	std::cerr<<"Final Training Accuracy: "<<(total_corr+0.0)/train_prob.l<<std::endl;
    }
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    train_prob_set.clear();
    
    for( int id = 0 ; id <= 2 ; id++ )
    {
        std::stringstream id_str;
        id_str << id;
    
    std::vector<problem> test_prob_set;
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
	if(exists_test(in_path + "test_"+ss.str()+"_L"+id_str.str()+".smat")==false )
		continue;
        std::cerr << "Reading: " << in_path + "test_"+ss.str()+"_L"+id_str.str()+".smat" << std::endl;
        std::vector<SparseDataOneClass> cur_test(1);
        int fea_dim = readSoluSparse_piecewise(in_path + "test_"+ss.str()+"_L"+id_str.str()+".smat", cur_test[0].fea_vec, piece_inds);
        cur_test[0].label = i+1;
        
	problem tmp_problem;
        FormFeaSparseMat(cur_test, tmp_problem, cur_test[0].fea_vec.size(), fea_dim);
	test_prob_set.push_back(tmp_problem);
    }
    if( test_prob_set.empty() == true )
	continue;
    problem test_prob;
    test_prob.l = 0;
    mergeProbs(test_prob_set, test_prob);
    
    total_test = test_prob.l;
    int conf_mat[OBJ_INST_MAX][OBJ_INST_MAX] = {0};
    int corr[OBJ_INST_MAX] = {0}, count[OBJ_INST_MAX] = {0};
    for(int j = 0 ; j < test_prob.l ; j++ )
    {   
        int true_label = floor(test_prob.y[j]+0.001-1);
        count[true_label]++;
        
        double tmp_label = predict(cur_model, test_prob.x[j]);
        
        //double tmp_label = predict(cur_model, test_prob.x[j]);
        int pred_label =  floor(tmp_label +0.001 - 1);
        conf_mat[true_label][pred_label]++;

        if( pred_label == true_label )
            corr[true_label]++;
        
    }
    
    if( exists_dir(in_path+"test_result_inst") == false )
        boost::filesystem::create_directories(in_path+"test_result_inst");
    std::ofstream acc_file((in_path+"test_result_inst/acc"+id_str.str()+".txt").c_str()); 
    std::ofstream conf_file((in_path+"test_result_inst/conf"+id_str.str()+".txt").c_str());
    total_corr = 0;
    for( int b1 = c1 ; b1 <= c2 ; b1++ )
    {
        total_corr += corr[b1];
        std::cerr<<b1<<" "<<count[b1]<<" "<<corr[b1]/(count[b1]+0.0)<<std::endl;
        acc_file << b1<<" "<<count[b1]<<" "<<corr[b1]/(count[b1]+0.0)<<std::endl;
    }
    std::cerr << "Total Train: "<< train_prob.l << std::endl;
    std::cerr << "Total Test: "<< test_prob.l << std::endl; 
    std::cerr<<"Final Testing Accuracy: "<<(total_corr+0.0)/test_prob.l<<std::endl;
    acc_file << (total_corr+0.0)/test_prob.l << std::endl;
    acc_file.close();

    //saving confusion matrix
    for( int i = c1 ; i <= c2 ; i++ ){
        for( int j = c1 ; j <= c2 ; j++ )
            conf_file << conf_mat[i][j] << " ";
        conf_file << std::endl;
    }
    conf_file.close();

    //Deallocation
    free(test_prob.y);
    for( int i = 0 ; i < test_prob.l ; i++ )
        free(test_prob.x[i]);
    free(test_prob.x);
    
    }
    destroy_param(&param);
    free_and_destroy_model(&cur_model);

    return 1;
}


    /*
    std::string in_path("/home/chi/UW_RGBD/filtered_pcd");
    std::ifstream inst_list((in_path + "/models_insts.txt").c_str(), std::ios::in);
    std::ifstream category_list((in_path + "/models.txt").c_str(), std::ios::in);
    std::ofstream category_id("inst_category_id.txt", std::ios::out);
    
    std::string category_name, inst_name;
    category_list >> category_name;
    inst_list >> inst_name;
    int cur_id = 1;
    int count = 1;
    for( ;; )
    {
        if( category_name == inst_name.substr(0, category_name.size()) )
        {
            std::cerr << category_name << " " << inst_name << std::endl;
            category_id << cur_id << " ";
            inst_list >> inst_name;
            count++;
            if( count >= 300 )
                break;
        }
        else
        {
            category_list >> category_name;
            cur_id++;
        }
    }
    inst_list.close();
    category_list.close();
    category_id.close();
    return 0;
     * */
