#include "sp_segmenter/utility/utility.h"

float sparseScore(cv::Mat M)
{
    int count = 0;
    int total = M.rows * M.cols;
    float *ptr = (float *)M.data;
    for( int i = 0 ; i < total ; i++, ptr++ )
        if(*ptr != 0 ) count++;
    return (count+0.0)/total;
}

int main(int argc, char** argv)
{   
    int c1 = 0, c2 = BB_INST_MAX-1;
    float CC = 0.001;
    int trial_id = 1;
    
    std::string in_path("UW_cshot_pool");
    pcl::console::parse_argument(argc, argv, "--C", CC);
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    if( exists_dir(in_path) == false )
        return 0;    

    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--t", trial_id);
    
    int fs = 0, fe = -1;
    pcl::console::parse_argument(argc, argv, "--fs", fs);
    pcl::console::parse_argument(argc, argv, "--fe", fe);
    
    std::vector< std::pair<int, int> > piece_inds;
    if( fs < fe )
    	piece_inds.push_back(std::pair<int, int> (fs, fe));
    
    bool tacc_flag = false;
    if( pcl::console::find_switch(argc, argv, "-tacc") == true )
        tacc_flag = true;    
    
    //int cate_id[UW_INST_MAX] = {0};
    int test_inst[UW_CLASS_MAX] = {0};
    std::ifstream inst_cate_id("UW_category_eval/inst_category_id.txt", std::ios::in);
    std::ifstream test_id("UW_category_eval/eval_set.txt", std::ios::in);

    if(inst_cate_id.is_open() == false || test_id.is_open() == false )
    {
        std::cerr << "No category information!" << std::endl;
        return 0;
    }
    std::vector< std::vector<int> > cate_id(UW_CLASS_MAX);
    for(int i = 0 ; i < UW_INST_MAX ; i++)
    {
        int idx;
        inst_cate_id >> idx;
        cate_id[idx-1].push_back(i);
    }
    for(int i = 0 ; i < trial_id ; i++ )
    {
        float tmp;
        for(int j = 0 ; j < UW_CLASS_MAX ; j++)
        {
            if(i == trial_id -1)
            {
                test_id >> test_inst[j];
                test_inst[j]--;
            }
            else
                test_id >> tmp;
        }
        
    }
    
    std::vector<problem> train_prob_set;
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        std::cerr << "Reading Training Class: " << ss.str() << std::endl;
        for( size_t j = 0 ; j < cate_id[i].size() ; j++ )
        {
            if( j == test_inst[i] )
                continue;
            std::stringstream ss1;
            ss1 << cate_id[i][j];
            
            std::vector<SparseDataOneClass> cur_train(1), cur_test(1);
            std::cerr << "Reading: " << in_path + "/train_"+ss1.str()+"_L0.smat" << std::endl;
            //std::cerr << "Reading: " << in_path + "/test_"+ss1.str()+"_L0.smat" << std::endl;
            int fea_dim = readSoluSparse_piecewise(in_path + "/train_"+ss1.str()+"_L0.smat", cur_train[0].fea_vec, piece_inds);
            fea_dim = readSoluSparse_piecewise(in_path + "/test_"+ss1.str()+"_L0.smat", cur_test[0].fea_vec, piece_inds);
            
            cur_train[0].label = i+1;
            cur_test[0].label = i+1;
            
            problem p1, p2;
            FormFeaSparseMat(cur_train, p1, cur_train[0].fea_vec.size(), fea_dim);
            FormFeaSparseMat(cur_test, p2, cur_test[0].fea_vec.size(), fea_dim);
            train_prob_set.push_back(p1);
            train_prob_set.push_back(p2);
        }
    }
    
    problem train_prob;
    train_prob.l = 0;
    mergeProbs(train_prob_set, train_prob);
    
    size_t total_train = train_prob.l;
    std::cerr<< "Fea Dim: " << train_prob.n << std::endl;
    
    parameter param;
    GenSVMParamter(param, CC);
    std::cerr<<std::endl<<"Starting Liblinear Training..."<<std::endl;
    model* cur_model = train(&train_prob, &param);
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
    
    std::vector<problem> test_prob_set;
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        std::cerr << "Reading Test Class: " << ss.str() << std::endl;
        
        std::stringstream ss1;
        ss1 << cate_id[i][test_inst[i]];
        
        std::vector<SparseDataOneClass> cur_train(1), cur_test(1);
        //std::cerr << "Reading: " << in_path + "/train_"+ss1.str()+"_L0.smat" << std::endl;
        std::cerr << "Reading: " << in_path + "/test_"+ss1.str()+"_L0.smat" << std::endl;
        int fea_dim = readSoluSparse_piecewise(in_path + "/train_"+ss1.str()+"_L0.smat", cur_train[0].fea_vec, piece_inds);
        fea_dim = readSoluSparse_piecewise(in_path + "/test_"+ss1.str()+"_L0.smat", cur_test[0].fea_vec, piece_inds);

        cur_train[0].label = i+1;
        cur_test[0].label = i+1;

        problem p1, p2;
        FormFeaSparseMat(cur_train, p1, cur_train[0].fea_vec.size(), fea_dim);
        FormFeaSparseMat(cur_test, p2, cur_test[0].fea_vec.size(), fea_dim);
        test_prob_set.push_back(p1);
        test_prob_set.push_back(p2);
    }
    problem test_prob;
    test_prob.l = 0;
    mergeProbs(test_prob_set, test_prob);
    
    size_t total_test = test_prob.l;
    int conf_mat[OBJ_INST_MAX][OBJ_INST_MAX] = {0};
    int corr[OBJ_INST_MAX] = {0}, count[OBJ_INST_MAX] = {0};
    for(int j = 0 ; j < test_prob.l ; j++ )
    {   
        int true_label = floor(test_prob.y[j]+0.001-1);
        count[true_label]++;
        double tmp_label = predict(cur_model, test_prob.x[j]);
        int pred_label =  floor(tmp_label +0.001 - 1);
        conf_mat[true_label][pred_label]++;

        if( pred_label == true_label )
            corr[true_label]++;
    }
    
    if( exists_dir(in_path+"/test_result_category") == false )
        boost::filesystem::create_directories(in_path+"/test_result_category");
    std::ofstream acc_file((in_path+"/test_result_category/acc.txt").c_str()); 
    std::ofstream conf_file((in_path+"/test_result_category/conf.txt").c_str());
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

    free_and_destroy_model(&cur_model);

    //Deallocation
    destroy_param(&param);
    free(test_prob.y);
    for( int i = 0 ; i < test_prob.l ; i++ )
        free(test_prob.x[i]);
    free(test_prob.x);
    
    return 1;
}

