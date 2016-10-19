#include "sp_segmenter/utility/utility.h"
#include "sp_segmenter/features.h"
#include "sp_segmenter/BBDataParser.h"
#include "sp_segmenter/UWDataParser.h"
#include "sp_segmenter/JHUDataParser.h"

void mergeProbs(const std::vector<problem> &problem_set, problem &final_prob)
{
    if( problem_set.empty() == true )
        return;
    if(final_prob.l > 0)
    {
        for( int i = 0 ; i < final_prob.l ; i++ )
            free(final_prob.x[i]);
        free(final_prob.x);
        free(final_prob.y);
    }
    
    size_t total = 0;
    int fea_dim = problem_set[0].n;
    float bias = problem_set[0].bias;
    
    for( std::vector<problem>::const_iterator it = problem_set.begin() ; it < problem_set.end() ; it++ )
    {    
        total += (*it).l;
        if( (*it).n != fea_dim || (*it).bias != bias )
        {
            std::cerr << (*it).n << " " << fea_dim << std::endl;
            std::cerr << "Merging Dimensions Mismatched!" << std::endl;
            exit(0);
        }
    }
    
    final_prob.l = total;
    final_prob.n = fea_dim;	//include bias term
    final_prob.bias = bias;
    
    final_prob.y = new float[final_prob.l];
    final_prob.x = new feature_node *[final_prob.l];
    int base = 0;
    for( std::vector<problem>::const_iterator it = problem_set.begin() ; it < problem_set.end() ; it++ )
    {
        for( int j = 0 ; j < (*it).l ; j++, base++ )
        {
            if( base >= final_prob.l )
            {
                std::cerr << "base >= final_prob.l!" << std::endl;
                exit(0);
            }
            final_prob.y[base] = (*it).y[j];
            final_prob.x[base] = (*it).x[j];
        }
        if( (*it).l > 0 )
            free((*it).y);
    }
}

double StatOnePool(cv::Mat pool_fea, cv::Mat &mean, cv::Mat &var)
{
    int num = pool_fea.rows;
    int dim = pool_fea.cols;
    mean.release();
    var.release();
    mean = cv::Mat::zeros(1, dim, CV_32FC1);
    var = cv::Mat::zeros(1, dim, CV_32FC1);
    
    for( int c = 0 ; c < dim ; c++ )
    {
        cv::Scalar cur_mean, cur_stddev;
        cv::Mat cur_fea = pool_fea.col(c);
        cv::meanStdDev(cur_fea, cur_mean, cur_stddev);
        mean.at<float>(0, c) = cur_mean.val[0];
        var.at<float>(0, c) = cur_stddev.val[0]*cur_stddev.val[0];
    }
    
    return cv::norm(var, cv::NORM_L1);
}

void onlineStat(cv::Mat one_data, cv::Mat &EX, cv::Mat &EX2, int N)
{
    if( EX.empty() == true )
        EX = cv::Mat::zeros(1, one_data.cols, CV_32FC1);
    if( EX2.empty() == true )
        EX2 = cv::Mat::zeros(1, one_data.cols, CV_32FC1);
    
    cv::Mat one_square = one_data.mul(one_data);
    EX = (EX*N+one_data)/(N+1);
    EX2 = (EX2*N+one_square)/(N+1);
}

double distDistri(cv::Mat E1, cv::Mat E2, cv::Mat V1, cv::Mat V2, float gamma)
{
    double diff_E = cv::norm(E1-E2, cv::NORM_L2);
    diff_E = pow(diff_E, 2.5);
    double diff_V = cv::norm(V1, cv::NORM_L1)+cv::norm(V2, cv::NORM_L1);
    
    double dist = diff_E / diff_V;
    std::cerr << diff_E << " " << diff_V << " " << dist << std::endl;
    
    //cv::Mat diff_E = E1-E2;
    //diff_E = diff_E.mul(diff_E);
    
    //cv::Mat sum_Var = V1 + V2;
    //cv::Mat dist_vec = diff_E / sum_Var;

    //double dist = cv::norm(dist_vec, cv::NORM_L1);
    
    return dist;
}

/*
double HellingerDist(cv::Mat E1, cv::Mat E2, cv::Mat V1, cv::Mat V2)
{
    cv::Mat diff_E = E1-E2;
    diff_E = diff_E.mul(diff_E);
    
    cv::Mat sum_Var = V1 + V2;
    cv::Mat dist_vec = diff_E / sum_Var;

    //double tmp1 = cv::norm(dist_vec, cv::NORM_L1);
    double tmp1 = -(1.0/4.0) * diff_E / (cv::norm(V1, cv::NORM_L1)+cv::norm(V2, cv::NORM_L1));
    //std::cerr <<"TMP1: "<< tmp1 << std::endl;
      
    cv::Mat bar_V = (V1+V2)/2;
    double prod = 1.0;
    for(int i = 0 ; i < V1.cols; i++)
    {
        if(V1.at<float>(0, i)>0 && V2.at<float>(0, i)>0)
            prod *= (sqrt(V1.at<float>(0, i))*sqrt(V2.at<float>(0, i)))/bar_V.at<float>(0, i);
        //std::cerr << prod1 << " " << prod2 << " " << prod3 << std::endl;
    }
    double tmp2 = sqrt(prod);
    std::cerr <<"TMP2: "<< tmp2 << std::endl;
    
    return 1 - tmp2*exp(tmp1);
}
//*/
void getDistri(std::vector<MulInfoT> &data, std::vector<Pooler_L0> &pooler_vec, Hier_Pooler &hie_producer, std::vector<cv::Mat> &mean_vec, std::vector<cv::Mat> &var_vec, std::string id, int type = 0)
{
    int len = pooler_vec.size();
    int num = data.size();
   
    mean_vec.clear();
    var_vec.clear();
    mean_vec.resize(len);
    var_vec.resize(len);
    
    //*
    std::vector<cv::Mat> mean_2(len);
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        std::vector<cv::Mat> cur_fea= hie_producer.getHierFea(data[j], 0);
        std::vector<cv::Mat> fea_vec(len);
        for( int i = 0 ; i < len ; i++ )
        {
            switch(type)
            {
               case 0:
                {
                    cv::Mat depth_fea = pooler_vec[i].PoolOneDomain(data[j].rgb, cur_fea[0], 1, true);
                    cv::Mat color_fea = pooler_vec[i].PoolOneDomain(data[j].rgb, cur_fea[1], 1, true);
                    cv::hconcat(depth_fea, color_fea, fea_vec[i]);
                    break;
                }
                case 1:
                {
                    cv::Mat depth_fea = pooler_vec[i].PoolOneDomain(data[j].uv, cur_fea[0], 1, true);
                    cv::Mat color_fea = pooler_vec[i].PoolOneDomain(data[j].uv, cur_fea[1], 1, true);
                    cv::hconcat(depth_fea, color_fea, fea_vec[i]);
                    break;
                }
                default:exit(1);    
            }
            onlineStat(fea_vec[i], mean_vec[i], mean_2[i], j);
            //std::cerr << mean_vec[i] << std::endl;
            //std::cerr << mean_2[i] << std::endl;
            //std::cin.get();
            
        }
        
    }
    for( int i = 0 ; i < len ; i++ )
    {
        cv::Mat E2 = mean_vec[i].mul(mean_vec[i]);
        var_vec[i] = mean_2[i] - E2;
    }
    //*/
    
    /*
    std::vector< std::vector<cv::Mat> > fea_vec(len);
    for( int i = 0 ; i < len ; i++ )
        fea_vec[i].resize(num);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        std::vector<cv::Mat> cur_fea= hie_producer.getHierFea(data[j], 0);
        for( int i = 0 ; i < len ; i++ )
        {
            switch(type)
            {
                case 0:
                {
                    cv::Mat depth_fea = pooler_vec[i].PoolOneDomain(data[j].rgb, cur_fea[0], 1, true);
                    cv::Mat color_fea = pooler_vec[i].PoolOneDomain(data[j].rgb, cur_fea[1], 1, true);
                    cv::hconcat(depth_fea, color_fea, fea_vec[i][j]);
                    break;
                }
                case 1:
                {
                    cv::Mat depth_fea = pooler_vec[i].PoolOneDomain(data[j].uv, cur_fea[0], 1, true);
                    cv::Mat color_fea = pooler_vec[i].PoolOneDomain(data[j].uv, cur_fea[1], 1, true);
                    cv::hconcat(depth_fea, color_fea, fea_vec[i][j]);
                }
                default:exit(1);    
            }
        }
        
    }
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int i = 0 ; i < len ; i++ )
    {
        std::ostringstream ss; 
        ss << i;
        
        cv::Mat cur_fea;
        cv::vconcat(fea_vec[i], cur_fea);
        double cur_var_total = StatOnePool(cur_fea, mean_vec[i], var_vec[i]);
        
        //std::cerr << "Var-"<<i+1<<": "<<cur_var_total<<std::endl;
        if( exists_dir("BB_stat/"+ss.str()) == false )
            boost::filesystem::create_directories("BB_stat/"+ss.str());
        saveCvMatSparse( "BB_stat/"+ss.str() + "/" + id + "_L0.smat", cur_fea);
        cur_fea.release();
    }
    //*/
}

/*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = BB_INST_MAX-1;
    
    std::string in_path("BB_fea_pool");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    if( exists_dir(in_path) == false )
        return 0;    
    
    bool xyz_flag = false;
    if( pcl::console::find_switch(argc, argv, "-xyz") == true )
        xyz_flag = true;

    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);

    std::vector< std::vector<cv::Mat> > mean_vec(c2-c1+1);
    std::vector< std::vector<cv::Mat> > var_vec(c2-c1+1);
    int num_layer = 8;
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        std::cerr << "Reading: " << in_path + "/train_"+ss.str()+"_L0.smat" << std::endl;
        
        cv::Mat cur_fea = readCvMatSparse(in_path + "/train_"+ss.str()+"_L0.smat");
        
        mean_vec[i].resize(num_layer);
        var_vec[i].resize(num_layer);
        for( int j = 0 ; j < num_layer ; j++ )
        {
            cv::Mat this_fea;
            switch(j)
            {
                case 0:
                    this_fea = xyz_flag == false ? cur_fea.colRange(0,204800) : cur_fea.colRange(204800, 409600);
                    break;
                case 1:
                    this_fea = xyz_flag == false ? cur_fea.colRange(409600, 546800) : cur_fea.colRange(546800, 684000);
                    break;
                case 2:
                    this_fea = xyz_flag == false ? cur_fea.colRange(684000, 770400) : cur_fea.colRange(770400, 856800);
                    break;
                case 3:
                    this_fea = xyz_flag == false ? cur_fea.colRange(856800, 906800) : cur_fea.colRange(906800, 956800);
                    break;
                case 4:
                    this_fea = xyz_flag == false ? cur_fea.colRange(956800, 982400) : cur_fea.colRange(982400, 1008000);
                    break;
                case 5:
                    this_fea = xyz_flag == false ? cur_fea.colRange(1008000, 1018800): cur_fea.colRange(1018800, 1029600);
                    break;
                case 6:
                    this_fea = xyz_flag == false ? cur_fea.colRange(1029600, 1032800) : cur_fea.colRange(1032800, 1036000);
                    break;
                case 7:
                    this_fea = xyz_flag == false ? cur_fea.colRange(1036000, 1036400) : cur_fea.colRange(1036400, 1036800);
                    break;
                default:exit(0);
            }
            double cur_var_total = StatOnePool(this_fea, mean_vec[i][j], var_vec[i][j]);
            std::cerr << "Var-"<<i+1<<"-"<<8-j<<": "<<cur_var_total<<std::endl;
        }
        
    }
    
    for( int j = 0 ; j < num_layer ; j++ ){
        double sum_dist = 0;
        for( int p = 0 ; p <= c2-c1  ; p++ ){
            for( int q = p+1 ; q <= c2-c1  ; q++ ){
                double dist = distDistri(mean_vec[p][j], mean_vec[q][j], var_vec[p][j], var_vec[q][j], 1.0);
                sum_dist += dist;
                //std::cerr << dist << " ";
            }
        }
        std::cerr << std::endl;
        std::cerr << "Layer-"<<num_layer-j<<": "<<sum_dist<<std::endl;
    }
    return 0;
}
//*/
//*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = BB_INST_MAX-1;
    
    std::string in_path("/home/chi/BigBIRD/processed");
    std::string dict_path("BB_new_dict");
    std::string out_path("BB_stat");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    //if( exists_dir(out_path) == false )
    //    return 0;    
    
    float radius = 0.03;
    int type = 0;
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--r", radius);
    pcl::console::parse_argument(argc, argv, "--type", type);

    std::ostringstream rr;
    rr << (int)(radius*100);
    std::cerr << rr.str() << std::endl;
    
    std::vector< std::vector<cv::Mat> > mean_vec(c2-c1+1);
    std::vector< std::vector<cv::Mat> > var_vec(c2-c1+1);
    int num_layer = 20;
    
    Hier_Pooler hie_producer(radius);
    hie_producer.LoadDict_L0(dict_path, "200", "200");
    
    std::vector<Pooler_L0> pooler_vec(num_layer);
    for( int i = 0 ; i < num_layer ; i++ )
        pooler_vec[i].setHSIPoolingParams(i+1);
    
    int dataset_id = -1;
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        ObjectSet train_objects, test_objects;
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i, true);
                break;
            case 1:
                readBB(in_path, train_objects, test_objects, i, i, true);
                break;
            case 2:
                readJHUInst(in_path, train_objects, test_objects, i, i, true);
                break;
            default:
                std::cerr << "No Corresponding Dataset!" << std::endl;
                exit(0);
        }
        getDistri(train_objects[0], pooler_vec, hie_producer, mean_vec[i-c1], var_vec[i-c1], "train_"+ss.str(), type);
        //std::vector<cv::Mat> temp_E, temp_V;
        //getDistri(test_objects[0], pooler_vec, hie_producer, temp_E, temp_V, "test_"+ss.str(), type);
        
        //mean_vec[i].resize(num_layer);
        //var_vec[i].resize(num_layer);
        std::ofstream fp_var;
        if( type == 0 )
            fp_var.open((out_path+"/var_lab_"+ss.str()+"_"+rr.str()+"_.txt").c_str(), std::ios::out);
        else
            fp_var.open((out_path+"/var_xyz_"+ss.str()+"_"+rr.str()+"_.txt").c_str(), std::ios::out);
        for( int j = 0 ; j < var_vec[i-c1].size() ; j++ )
        {
            //int sum_non_zero = 0;
            //for(int k = 0 ; k < var_vec[i-c1][j].cols; k++ )
            //{
            //    if( var_vec[i-c1][j].at<float>(0, k) > 1e-6 )
            //        sum_non_zero++;
            //}
            //double var_score = cv::norm(var_vec[i-c1][j], cv::NORM_L1) / sum_non_zero;
            fp_var << j << " " << cv::norm(var_vec[i-c1][j], cv::NORM_L1) << std::endl;
            std::cerr << "Var-"<<i+1<<"-"<<j<<": "<<cv::norm(var_vec[i-c1][j], cv::NORM_L1) << std::endl;
        }
        fp_var.close();
        /*
        for( int k = 0 ; k < mean_vec[i-c1].size() ; k++ )
        {
            std::ostringstream kk; 
            kk << k;

            if( exists_dir("BB_stat/"+ss.str()) == false )
                boost::filesystem::create_directories("BB_stat/"+ss.str());
            saveMat( "BB_stat/"+kk.str() + "/mean_"+ss.str()+"_L0.smat", mean_vec[i-c1][k]);
            saveMat( "BB_stat/"+kk.str() + "/var_" +ss.str()+"_L0.smat", var_vec[i-c1][k]);
        }
         * */
    }
    
    //for( int j = 0 ; j < num_layer ; j++ ){
    //    double sum_dist = 0;
    //    for( int p = 0 ; p <= c2-c1  ; p++ ){
    //        for( int q = p+1 ; q <= c2-c1  ; q++ ){
    //            double dist = distDistri(mean_vec[p][j], mean_vec[q][j], var_vec[p][j], var_vec[q][j], 1.0);
    //            sum_dist += dist;
    //        }
    //    }
    //    std::cerr << "Layer-"<<j<<": "<<sum_dist<<std::endl;
    //}
    
    return 0;
}
//*/
/*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = BB_INST_MAX-1;
    float CC = 1;
    
    std::string in_path("BB_stat");
    pcl::console::parse_argument(argc, argv, "--C", CC);
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    if( exists_dir(in_path) == false )
        return 0;    

    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    int layer = 0;
    pcl::console::parse_argument(argc, argv, "--l", layer);
    
    size_t total_train, total_test;
    std::vector<problem> train_prob_set(c2-c1+1);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        std::vector<cv::Mat> cur_train_vec(layer);
        for( int j = 0 ; j < layer ; j++ )
        {
            std::stringstream ll;
            ll << j;
            std::cerr << "Reading: " << in_path + "/" + ll.str() + "/train_"+ss.str()+"_L0.smat" << std::endl;
            cur_train_vec[j] = readCvMatSparse(in_path + "/" + ll.str() + "/train_"+ss.str()+"_L0.smat");
        }
        cv::Mat final_train;
       
        cv::hconcat(cur_train_vec, final_train);
        cur_train_vec.clear();
        
        int fea_dim = final_train.cols;
        std::vector<SparseDataOneClass> cur_train(1);
        
        sparseCvMat(final_train, cur_train[0].fea_vec);
        final_train.release();
        cur_train[0].label = i+1;
        
        FormFeaSparseMat(cur_train, train_prob_set[i-c1], cur_train[0].fea_vec.size(), fea_dim);
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
    
    int total_corr;
    // Computing traininig and testing accuracies
    
    free(train_prob.y);
    for( int i = 0 ; i < train_prob.l ; i++ )
        free(train_prob.x[i]);
    free(train_prob.x);
    train_prob_set.clear();
    
    std::vector<problem> test_prob_set(c2-c1+1);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        std::vector<cv::Mat> cur_test_vec(layer);
        for( int j = 0 ; j < layer ; j++ )
        {
            std::stringstream ll;
            ll << j;
            std::cerr << "Reading: " << in_path + "/" + ll.str() + "/test_"+ss.str()+"_L0.smat" << std::endl;
            cur_test_vec[j] = readCvMatSparse(in_path + "/" + ll.str() + "/test_"+ss.str()+"_L0.smat");
        }
        cv::Mat final_test;
       
        cv::hconcat(cur_test_vec, final_test);
        cur_test_vec.clear();
        
        int fea_dim = final_test.cols;
        std::vector<SparseDataOneClass> cur_test(1);
        
        sparseCvMat(final_test, cur_test[0].fea_vec);
        final_test.release();
        cur_test[0].label = i+1;
        
        FormFeaSparseMat(cur_test, test_prob_set[i-c1], cur_test[0].fea_vec.size(), fea_dim);
        
    }
    problem test_prob;
    test_prob.l = 0;
    mergeProbs(test_prob_set, test_prob);
    
    total_test = test_prob.l;
    int corr[OBJ_INST_MAX] = {0}, count[OBJ_INST_MAX] = {0};
    for(int j = 0 ; j < test_prob.l ; j++ )
    {   
        int true_label = floor(test_prob.y[j]+0.001-1);
        count[true_label]++;
        double tmp_label = predict(cur_model, test_prob.x[j]);
        int pred_label =  floor(tmp_label +0.001 - 1);
        
        if( pred_label == true_label )
            corr[true_label]++;
    }
    total_corr = 0;
    for( int b1 = c1 ; b1 <= c2 ; b1++ )
        total_corr += corr[b1];
    
    std::cerr << "Total Train: "<< train_prob.l << std::endl;
    std::cerr << "Total Test: "<< test_prob.l << std::endl; 
    std::cerr<<"Final Testing Accuracy: "<<(total_corr+0.0)/test_prob.l<<std::endl;
    
    
    free_and_destroy_model(&cur_model);

    //Deallocation
    destroy_param(&param);
    free(test_prob.y);
    for( int i = 0 ; i < test_prob.l ; i++ )
        free(test_prob.x[i]);
    free(test_prob.x);
    
    cv::Mat final_E_1, final_E_2, final_V_1, final_V_2;
   
    std::vector<cv::Mat> cur_E_1(layer);
    std::vector<cv::Mat> cur_E_2(layer);
    std::vector<cv::Mat> cur_V_1(layer);
    std::vector<cv::Mat> cur_V_2(layer);
    
    std::ostringstream cc1, cc2;
    cc1 << c1;
    cc2 << c2;
    for( int j = 0 ; j < layer ; j++ )
    {
        std::stringstream ll;
        ll << j;
        std::cerr << "Reading: " << in_path + "/" + ll.str() + "/mean_"+cc1.str()+"_L0.smat" << std::endl;
        readMat(in_path + "/" + ll.str() + "/mean_"+cc1.str()+"_L0.smat", cur_E_1[j]);
        std::cerr << "Reading: " << in_path + "/" + ll.str() + "/mean_"+cc2.str()+"_L0.smat" << std::endl;
        readMat(in_path + "/" + ll.str() + "/mean_"+cc2.str()+"_L0.smat", cur_E_2[j]);
        
        std::cerr << "Reading: " << in_path + "/" + ll.str() + "/var_"+cc1.str()+"_L0.smat" << std::endl;
        readMat(in_path + "/" + ll.str() + "/mean_"+cc1.str()+"_L0.smat", cur_V_1[j]);
        std::cerr << "Reading: " << in_path + "/" + ll.str() + "/var_"+cc2.str()+"_L0.smat" << std::endl;
        readMat(in_path + "/" + ll.str() + "/var_"+cc2.str()+"_L0.smat", cur_V_2[j]);
    }
    cv::hconcat(cur_E_1,final_E_1);
    cv::hconcat(cur_E_2,final_E_2);
    cv::hconcat(cur_V_1,final_V_1);
    cv::hconcat(cur_V_2,final_V_2);
    
    double dist = distDistri(final_E_1, final_E_2, final_V_1, final_V_2, 1.0);
    
    
    return 1;
}
//*/
/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/BigBIRD/processed");
    std::string out_path("BB_fea_pool");
    std::string dict_path("BB_new_dict");
    //std::string dict_path("dict");
    
    int c1 = 0, c2 = 1;
    float radius = 0.03;
    int dataset_id = -1;
    int type = 0;
    float gamma = 1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    pcl::console::parse_argument(argc, argv, "--r", radius);
    pcl::console::parse_argument(argc, argv, "--type", type);
    pcl::console::parse_argument(argc, argv, "--g", gamma);
    
    std::ostringstream tt;
    tt << type;
    
    int len = 20;
    pcl::console::parse_argument(argc, argv, "--l", len);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    Hier_Pooler hie_producer(radius);
    hie_producer.LoadDict_L0(dict_path, "200", "200");
    
    std::vector<Pooler_L0> pooler_vec(len);
    for( int i = 0 ; i < len ; i++ )
        pooler_vec[i].setHSIPoolingParams(i+1);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    ObjectSet train_1, test_1;
    ObjectSet train_2, test_2;    
    switch(dataset_id)
    {
        case 0:
            readUWInst(in_path, train_1, test_1, c1, c1);
            readUWInst(in_path, train_2, test_2, c2, c2);
            break;
        case 1:
            readBB(in_path, train_1, test_1, c1, c1);
            readBB(in_path, train_2, test_2, c2, c2);
            break;
        case 2:
            readJHUInst(in_path, train_1, test_1, c1, c1);
            readJHUInst(in_path, train_2, test_2, c2, c2);
            break;
        default:
            std::cerr << "No Corresponding Dataset!" << std::endl;
            exit(0);
    }
    std::cerr << "Loading Completed... " << std::endl;
    
    std::vector<cv::Mat> mean_1_vec, mean_2_vec, mean_3_vec, mean_4_vec;
    std::vector<cv::Mat> var_1_vec, var_2_vec, var_3_vec, var_4_vec;
    
    //*
    std::ostringstream ss1 , ss2;
    ss1 << c1;
    ss2 << c2;
    getDistri(train_1[0], pooler_vec, hie_producer, mean_1_vec, var_1_vec, "train_"+ss1.str(), type);
    train_1[0].clear();
    std::cerr << "Train 1 Done!" << std::endl;
    getDistri(train_2[0], pooler_vec, hie_producer, mean_2_vec, var_2_vec, "train_"+ss2.str(), type);
    train_2[0].clear();
    std::cerr << "Train 2 Done!" << std::endl;
    getDistri(test_1[0], pooler_vec, hie_producer, mean_3_vec, var_3_vec, "test_"+ss1.str(), type);
    test_1[0].clear();
    std::cerr << "Test 1 Done!" << std::endl;
    getDistri(test_2[0], pooler_vec, hie_producer, mean_4_vec, var_4_vec, "test_"+ss2.str(), type);
    test_2[0].clear();
    std::cerr << "Test 2 Done!" << std::endl;
    
    //std::ofstream fp_diff;
    //fp_diff.open((out_path+"/diff_t"+tt.str()+".txt").c_str(), std::ios::out);
    for( int i = 0 ; i < len ; i++ )
    {
        std::cerr << "Pool-" << i+1 << std::endl;
        double dist = distDistri(mean_1_vec[i], mean_2_vec[i], var_1_vec[i], var_2_vec[i], gamma);
        //<<": "<<dist<<std::endl;
        
        //double dist1 = distDistri(mean_1_vec[i], mean_3_vec[i], var_1_vec[i], var_3_vec[i]);
        //double dist2 = distDistri(mean_2_vec[i], mean_3_vec[i], var_2_vec[i], var_3_vec[i]);
        
        //double dist3 = distDistri(mean_1_vec[i], mean_4_vec[i], var_1_vec[i], var_4_vec[i]);
        //double dist4 = distDistri(mean_2_vec[i], mean_4_vec[i], var_2_vec[i], var_4_vec[i]);
        
        //std::cerr << "Pool-"<<i+1<<": "<<dist2/dist1<<" "<<dist3/dist4<<std::endl;
        //fp_diff<<i+1<<" "<<dist2/dist1<<" "<<dist3/dist4<<std::endl;
    }
    //fp_diff.close();
    
   
    return 1;
}
//*/

 /*
    std::ofstream fp_var_2;
    fp_var_2.open((out_path+"/var_2.txt").c_str(), std::ios::out);
    std::ofstream fp_bound_diff;
    fp_bound_diff.open((out_path+"/bound_diff.txt").c_str(), std::ios::out);
    std::ofstream fp_diff;
    fp_diff.open((out_path+"/diff.txt").c_str(), std::ios::out);
    for( int i = 0 ; i < len ; i++ )
    {
        cv::Mat cur_fea_2;
        
        cv::vconcat(fea_vec_2[i], cur_fea_2);
        double cur_var_total_2 = StatOnePool(cur_fea_2, mean_2_vec[i], var_2_vec[i]);
        cur_fea_2.release();
        
        fp_var_2 << cur_var_total_2 << " ";
        std::cerr << "Var-"<<i+1<<": "<<cur_var_total_2<<std::endl;
        
        double cur_dist = distDistri(mean_1_vec[i], mean_2_vec[i], var_1_vec[i], var_2_vec[i]);
        fp_bound_diff << cur_dist << " ";
        
        std::cerr << "Pool-"<<i+1<<": "<<cur_dist<<std::endl;
    }
    
    fp_var_1.close();
    fp_var_2.close();
    fp_bound_diff.close();
    fp_diff.close();
    //*/