#include "sp_segmenter/utility/utility.h"
#include "sp_segmenter/features.h"
#include "sp_segmenter/WKmeans.h"

cv::Mat getCorrelation(cv::Mat fea)
{
    cv::Mat covar, mean;
    cv::calcCovarMatrix(fea, covar, mean, CV_COVAR_NORMAL|CV_COVAR_ROWS|CV_COVAR_SCALE);
    
    for( int i = 0 ; i < covar.rows ; i++ ){
        double std_dev = sqrt(covar.at<double>(i,i));
        for( int j = 0 ; j < covar.cols ; j++){
            if( i == j )
                continue;
            covar.at<double>(i, j) /= std_dev;
        }
        for( int j = 0 ; j < covar.cols ; j++){
            if( i == j )
                continue;
            covar.at<double>(j, i) /= std_dev;
        }
    }
    
    covar.convertTo(covar, CV_32FC1);
    return covar;
}

void Max_Avg(cv::Mat corr, float &avg_val, float &max_val)
{
    max_val = 0;
    avg_val = 0;
    int count=0;
    for( int i = 0 ; i < corr.rows ; i++ ){
        for( int j = i+1 ; j < corr.cols ; j++, count++ )
        {
            float cur_val = fabs(corr.at<float>(i, j));
            avg_val += cur_val;
            if( max_val < cur_val )
                max_val = cur_val;
        }
    }
    avg_val /= count;
    std::cerr <<"Average Value: "<< avg_val << std::endl;
    std::cerr <<"Max Value: "<< max_val << std::endl;
}

std::vector< std::vector<int> > getPatterns(cv::Mat corr, float T)
{
    Eigen::MatrixXi adj_mat = Eigen::MatrixXi::Identity(corr.rows, corr.cols);
    for(int i = 0 ; i < corr.rows; i++ ){
        for(int j = i ; j < corr.cols; j++ ){
            if( fabs(corr.at<float>(i, j)) >= T )
            {
                adj_mat(i, j) = 1;
                adj_mat(j, i) = 1;
            }
        }
    }
    
    std::vector< std::vector<int> > patterns_idx = maximalClique(adj_mat);
    //for( size_t i = 0 ; i < patterns_idx.size() ; i++ )
    //{
    //    std::cerr << "Cluster " << i << std::endl;
    //    for( size_t j = 0 ; j < patterns_idx[i].size() ; j++ )
    //        std::cerr << patterns_idx[i][j] << " ";
    //    std::cerr << std::endl;
    //}
    return patterns_idx;
}

cv::Mat ext_pattern(cv::Mat fea, cv::Mat xyz_lab, const std::vector<int> &pattern_idx, float ratio)
{
    int total_num = fea.rows;
    std::vector<cv::Mat> key_dim_vec(pattern_idx.size());
    for( int j = 0 ; j < pattern_idx.size() ; j++)
        key_dim_vec[j] = fea.col(pattern_idx[j]);
    
    cv::Mat key_dim;
    cv::hconcat(key_dim_vec, key_dim);
    
    std::vector<cv::Mat> pattern_vec;
    for( int i = 0 ; i < total_num ; i++ ){
        if( cv::norm(key_dim.row(i), cv::NORM_INF) >= ratio )
            pattern_vec.push_back(xyz_lab.row(i));
    }
    std::cerr << "Active Idx: " << pattern_idx.size() << std::endl;
    std::cerr << "Pattern Number: " << pattern_vec.size() << std::endl;
    cv::Mat pattern_xyz_lab;
    cv::vconcat(pattern_vec, pattern_xyz_lab);
    
    return pattern_xyz_lab;
}

//*
void ext_pooling(cv::Mat pool_fea, cv::Mat &center, cv::Mat &range, int K)
{
    int num = pool_fea.rows;
    //cv::flann::KDTreeIndexParams indexParams;
    //cv::flann::Index fea_tree;
    //fea_tree.build(pool_fea, cv::noArray(), indexParams);
    
    cv::Mat labels;
    cv::kmeans(pool_fea, K, labels, cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 1000, 1e-6), 5, cv::KMEANS_PP_CENTERS, center);
        
    range = cv::Mat::zeros(center.rows, 1, CV_32FC1);
    std::vector<size_t> count(center.rows, 0);
    int *idx = (int *)labels.data;
    for( int i = 0 ; i < pool_fea.rows ; i++, idx++ )
    {
        float cur_dist = cv::norm(pool_fea.row(i), center.row(*idx), cv::NORM_L2);
        //if( range.at<float>(*idx, 0) < cur_dist )
            range.at<float>(*idx, 0) += cur_dist;
            count[*idx]++;
    }    
    
    float *ptr = (float *)range.data;
    for( int i = 0 ; i < range.rows ; i++, ptr++ )
        *ptr /= count[i];
    
}
//*/

void getPoolingOpt(cv::Mat fea, cv::Mat corr, cv::Mat xyz_lab, cv::Mat &centers, cv::Mat &ranges, float tt, float ratio)
{
    std::vector< std::vector<int> > pattern_idx = getPatterns(corr, tt);
    int len = pattern_idx.size();
    std::vector<cv::Mat> centers_vec(len);
    std::vector<cv::Mat> ranges_vec(len);
    
    #pragma omp parallel for schedule(dynamic, 1) 
    for( int i = 0 ; i < len ; i++ )
    {
        cv::Mat cur_xyz_lab = ext_pattern(fea, xyz_lab, pattern_idx[i], ratio);
        //int cur_K = 20 - pattern_idx[i].size();
        //if( cur_K <= 1 )
        //    cur_K = 2;
        int cur_K = 300;
        
        cv::Mat cur_center, cur_range;
        ext_pooling(cur_xyz_lab, cur_center, cur_range, cur_K);
        
        centers_vec[i] = cur_center;
        ranges_vec[i] = cur_range;
        //std::cerr << cur_range << std::endl;
    }
    
    cv::vconcat(centers_vec, centers);
    cv::vconcat(ranges_vec, ranges);
}

void getPoolingSep(cv::Mat fea, cv::Mat corr, cv::Mat xyz_lab, cv::Mat &xyz_center, cv::Mat &xyz_ranges, cv::Mat &lab_center, cv::Mat &lab_ranges, float tt, float ratio)
{
    int KK = 200;
    
    std::vector< std::vector<int> > pattern_idx = getPatterns(corr, tt);
    int len = pattern_idx.size();
    std::vector<cv::Mat> xyz_centers_vec(len);
    std::vector<cv::Mat> xyz_ranges_vec(len);
    std::vector<cv::Mat> lab_centers_vec(len);
    std::vector<cv::Mat> lab_ranges_vec(len);
    
    #pragma omp parallel for schedule(dynamic, 1) 
    for( int i = 0 ; i < len ; i++ )
    {
        cv::Mat cur_xyz_lab = ext_pattern(fea, xyz_lab, pattern_idx[i], ratio);
        
        cv::Mat cur_xyz = cur_xyz_lab.colRange(0, 3);
        cv::Mat cur_lab = cur_xyz_lab.colRange(3, 6);
        
        cv::Mat cur_xyz_center, cur_xyz_range, cur_lab_center, cur_lab_range;
        ext_pooling(cur_xyz, cur_xyz_center, cur_xyz_range, KK);
        ext_pooling(cur_lab, cur_lab_center, cur_lab_range, KK);
        
        xyz_centers_vec[i] = cur_xyz_center;
        xyz_ranges_vec[i] = cur_xyz_range;
        
        lab_centers_vec[i] = cur_lab_center;
        lab_ranges_vec[i] = cur_lab_range;
    }
    
    cv::vconcat(xyz_centers_vec, xyz_center);
    cv::vconcat(xyz_ranges_vec, xyz_ranges);
    
    cv::vconcat(lab_centers_vec, lab_center);
    cv::vconcat(lab_ranges_vec, lab_ranges);
}


void refineCenters(cv::Mat center, cv::Mat range, cv::Mat &new_center, cv::Mat &new_range)
{
    int num = center.rows;
    cv::flann::LinearIndexParams indexParams;
    cv::flann::Index fea_tree;
    fea_tree.build(center, cv::noArray(), indexParams);
    
    std::vector< std::pair<int, int> > inds(num);
    std::vector< std::vector<int> > neigh_vec(num);
 
    #pragma omp parallel for schedule(dynamic, 1) 
    for( int i = 0 ; i < num ; i++ )
    {
        std::vector<float> dists;
        std::vector<int> tmp;
        int found = fea_tree.radiusSearch(center.row(i), tmp, dists, range.at<float>(i, 0), num, cv::flann::SearchParams());
        
        inds[i].first = i;
        inds[i].second = found;
        
        tmp.erase(tmp.begin()+found, tmp.end());
        neigh_vec[i] = tmp;
        //neigh_vec[i].clear();
        //neigh_vec[i].erase(neigh_vec[i].begin()+found, neigh_vec[i].end());
        //std::cerr << tmp.size() << std::endl;
        //std::cerr << found << std::endl;
        //std::cin.get();
        //std::cerr << i << std::endl;
    }
    
    std::vector<cv::Mat> new_center_vec;
    std::vector<cv::Mat> new_range_vec;
    std::vector<bool> flag(num, 0);
    std::sort(inds.begin(), inds.end(), comp2);
    for( int i = 0 ; i < inds.size() ; i++ )
    {
        int cur_idx = inds[i].first;
        if( flag[cur_idx] == false && range.at<float>(cur_idx, 0) > 0.01 )
        {
            new_center_vec.push_back(center.row(cur_idx));
            new_range_vec.push_back(range.row(cur_idx));
        }
        for( std::vector<int>::iterator p = neigh_vec[cur_idx].begin() ; p < neigh_vec[cur_idx].end() ; p++ )
        {
            //std::cerr << *p << " ";
            flag[*p] = true;
        }
        //std::cerr << neigh_vec[]
        //std::cin.get();
    }
    cv::vconcat(new_center_vec, new_center);
    cv::vconcat(new_range_vec, new_range);
}
//*
void refineCenterFixed(cv::Mat center, cv::Mat &new_center, float radius)
{
    //cv::Mat buf = cv::norm(range, cv::NORM_L1) / range.rows;
    //float radius = buf.at<float>(0, 0);
    //std::cerr << "Radius: " << radius << std::endl;
    
    int num = center.rows;
    cv::flann::LinearIndexParams indexParams;
    cv::flann::Index fea_tree;
    fea_tree.build(center, cv::noArray(), indexParams);
    
    std::vector< std::pair<int, int> > inds(num);
    std::vector< std::vector<int> > neigh_vec(num);
 
    #pragma omp parallel for schedule(dynamic, 1) 
    for( int i = 0 ; i < num ; i++ )
    {
        std::vector<float> dists;
        std::vector<int> tmp;
        int found = fea_tree.radiusSearch(center.row(i), tmp, dists, radius, num, cv::flann::SearchParams());
        
        inds[i].first = i;
        inds[i].second = found;
        
        tmp.erase(tmp.begin()+found, tmp.end());
        neigh_vec[i] = tmp;
        //neigh_vec[i].clear();
        //neigh_vec[i].erase(neigh_vec[i].begin()+found, neigh_vec[i].end());
        //std::cerr << tmp.size() << std::endl;
        //std::cerr << found << std::endl;
        //std::cin.get();
        //std::cerr << i << std::endl;
    }
    
    std::vector<cv::Mat> new_center_vec;
    std::vector<bool> flag(num, 0);
    std::sort(inds.begin(), inds.end(), comp2);
    for( int i = 0 ; i < inds.size() ; i++ )
    {
        int cur_idx = inds[i].first;
        if( flag[cur_idx] == false )
            new_center_vec.push_back(center.row(cur_idx));
        
        for( std::vector<int>::iterator p = neigh_vec[cur_idx].begin() ; p < neigh_vec[cur_idx].end() ; p++ )
        {
            //std::cerr << *p << " ";
            flag[*p] = true;
        }
        //std::cerr << neigh_vec[]
        //std::cin.get();
    }
    cv::vconcat(new_center_vec, new_center);
}
//*/

/*
// Get Covariance Matrix of L0 feature
int main(int argc, char** argv)
{   
    float tt = 0.5, ratio = 0.3;
    pcl::console::parse_argument(argc, argv, "--t", tt);
    pcl::console::parse_argument(argc, argv, "--r", ratio);
    
    std::string in_path("UW_new_dict");
    //std::string in_path("BB_new_dict");
    
    cv::Mat depth_fea, color_fea, xyz_lab;
    readMat(in_path+"/depth_raw_L0.cvmat", depth_fea);
    readMat(in_path+"/color_raw_L0.cvmat", color_fea);
    readMat(in_path+"/xyz_lab_L0.cvmat", xyz_lab);
    
    bool compute_corr = false;
    int num = depth_fea.rows;
    if( num != color_fea.rows || num != xyz_lab.rows )
    {
        std::cerr<<"Dimension Mismatched!" << std::endl;
        return 0;
    }
    //Normalize raw features
    for(int i = 0 ; i < num; i++ )
    {
        cv::normalize(depth_fea.row(i),depth_fea.row(i));
        cv::normalize(color_fea.row(i),color_fea.row(i));
    }
    
    
    cv::Mat depth_corr, color_corr;
    if( pcl::console::find_switch(argc, argv, "-corr") == true )
        compute_corr = true;
    if( compute_corr == true )
    {
        depth_corr = getCorrelation(depth_fea);
        saveMat(in_path+"/depth_corr.cvmat", depth_corr);
        std::cerr << "**** Depth ****" << std::endl;
        
        color_corr = getCorrelation(color_fea);
        saveMat(in_path+"/color_corr.cvmat", color_corr);
        std::cerr << "**** Color ****" << std::endl;
    }
    else
    {
        readMat(in_path+"/depth_corr.cvmat", depth_corr);
        readMat(in_path+"/color_corr.cvmat", color_corr);
    }
    //cv::Mat depth_center, depth_range;
    //getPoolingOpt(depth_fea, depth_corr, xyz_lab, depth_center, depth_range, tt, ratio);
    //saveMat(in_path+"/depth_center.cvmat", depth_center);
    //saveMat(in_path+"/depth_range.cvmat", depth_range);
    
    //cv::Mat color_center, color_range;
    //getPoolingOpt(color_fea, color_corr, xyz_lab, color_center, color_range, tt, ratio);
    //saveMat(in_path+"/color_center.cvmat", color_center);
    //saveMat(in_path+"/color_range.cvmat", color_range);
    
    cv::Mat depth_xyz_center, depth_xyz_range, depth_lab_center, depth_lab_range;
    getPoolingSep(depth_fea, depth_corr, xyz_lab, depth_xyz_center, depth_xyz_range, depth_lab_center, depth_lab_range, tt, ratio);
    saveMat(in_path+"/depth_xyz_center.cvmat", depth_xyz_center);
    saveMat(in_path+"/depth_xyz_range.cvmat", depth_xyz_range);
    saveMat(in_path+"/depth_lab_center.cvmat", depth_lab_center);
    saveMat(in_path+"/depth_lab_range.cvmat", depth_lab_range);
    
    cv::Mat color_xyz_center, color_xyz_range, color_lab_center, color_lab_range;
    getPoolingSep(color_fea, color_corr, xyz_lab, color_xyz_center, color_xyz_range, color_lab_center, color_lab_range, tt, ratio);
    saveMat(in_path+"/color_xyz_center.cvmat", color_xyz_center);
    saveMat(in_path+"/color_xyz_range.cvmat", color_xyz_range);
    saveMat(in_path+"/color_lab_center.cvmat", color_lab_center);
    saveMat(in_path+"/color_lab_range.cvmat", depth_lab_range);   
    
    return 1;
}
//*/

/*
// Get Covariance Matrix of L0 feature
int main(int argc, char** argv)
{   
    float tt = 1.0, ratio = 0.3;
    pcl::console::parse_argument(argc, argv, "--t", tt);
    std::string id("1");
    pcl::console::parse_argument(argc, argv, "--i", id);
    
    std::string in_path("UW_new_dict");
    
    {
        cv::Mat depth_xyz_center, depth_xyz_range;
        readMat(in_path+"/depth_xyz_center.cvmat", depth_xyz_center);
        readMat(in_path+"/depth_xyz_range.cvmat", depth_xyz_range);
        std::cerr << depth_xyz_center.rows << std::endl;
        depth_xyz_range *= tt;

        cv::Mat refined_depth_xyz_center, refined_depth_xyz_range;
        //refineCenters(depth_xyz_center, depth_xyz_range, refined_depth_xyz_center, refined_depth_xyz_range);
        refineCenterFixed(depth_xyz_center, refined_depth_xyz_center, tt);
        std::cerr << refined_depth_xyz_center.rows << std::endl;
        saveMat(in_path+"/filter_depth_xyz_center_" + id + ".cvmat", refined_depth_xyz_center);
    }
    {
        cv::Mat depth_lab_center, depth_lab_range;
        readMat(in_path+"/depth_lab_center.cvmat", depth_lab_center);
        readMat(in_path+"/depth_lab_range.cvmat", depth_lab_range);
        std::cerr << depth_lab_center.rows << std::endl;
        depth_lab_range *= tt;

        cv::Mat refined_depth_lab_center, refined_depth_lab_range;
        //refineCenters(depth_lab_center, depth_lab_range, refined_depth_lab_center, refined_depth_lab_range);
        refineCenterFixed(depth_lab_center, refined_depth_lab_center, tt);
        std::cerr << refined_depth_lab_center.rows << std::endl;
        saveMat(in_path+"/filter_depth_lab_center_" + id + ".cvmat", refined_depth_lab_center);
    }
    
    {
        cv::Mat color_xyz_center, color_xyz_range;
        readMat(in_path+"/color_xyz_center.cvmat", color_xyz_center);
        readMat(in_path+"/color_xyz_range.cvmat", color_xyz_range);
        std::cerr << color_xyz_center.rows << std::endl;
        color_xyz_range *= tt;

        cv::Mat refined_color_xyz_center, refined_color_xyz_range;
        //refineCenters(color_xyz_center, color_xyz_range, refined_color_xyz_center, refined_color_xyz_range);
        refineCenterFixed(color_xyz_center, refined_color_xyz_center, tt);
        std::cerr << refined_color_xyz_center.rows << std::endl;
        saveMat(in_path+"/filter_color_xyz_center_" + id + ".cvmat", refined_color_xyz_center);
    }
    {
        cv::Mat color_lab_center, color_lab_range;
        readMat(in_path+"/color_lab_center.cvmat", color_lab_center);
        readMat(in_path+"/color_lab_range.cvmat", color_lab_range);
        std::cerr << color_lab_center.rows << std::endl;
        color_lab_range *= tt;

        cv::Mat refined_color_lab_center, refined_color_lab_range;
        //refineCenters(color_lab_center, color_lab_range, refined_color_lab_center, refined_color_lab_range);
        refineCenterFixed(color_lab_center, refined_color_lab_center, tt);
        std::cerr << refined_color_lab_center.rows << std::endl;
        saveMat(in_path+"/filter_color_lab_center_" + id + ".cvmat", refined_color_lab_center);
    }
    
    return 1;
}
//*/

//*
// Get Covariance Matrix of L0 feature
int main(int argc, char** argv)
{   
    std::string in_path("UW_new_dict");
    //std::string in_path("BB_new_dict");
    
    int KK = 128;
    int T = 10;
    pcl::console::parse_argument(argc, argv, "--K", KK);
    pcl::console::parse_argument(argc, argv, "--t", T);
    
    std::ostringstream ss;
    ss << KK;
    
    {
        cv::Mat depth_xyz_center, depth_xyz_range;
        readMat(in_path+"/depth_xyz_center.cvmat", depth_xyz_center);
        readMat(in_path+"/depth_xyz_range.cvmat", depth_xyz_range);
        std::cerr << depth_xyz_center.rows << std::endl;
        depth_xyz_range = depth_xyz_range.mul(depth_xyz_range);

        WKmeans depth_clusterer;
        depth_clusterer.AddData(depth_xyz_center, depth_xyz_range);
        cv::Mat refined_depth_xyz_center;
        depth_clusterer.W_Cluster(refined_depth_xyz_center, KK, T);

        saveMat(in_path+"/refined_depth_xyz_center_"+ss.str()+".cvmat", refined_depth_xyz_center);
    }
    {
        cv::Mat depth_lab_center, depth_lab_range;
        readMat(in_path+"/depth_lab_center.cvmat", depth_lab_center);
        readMat(in_path+"/depth_lab_range.cvmat", depth_lab_range);
        std::cerr << depth_lab_center.rows << std::endl;
        depth_lab_range = depth_lab_range.mul(depth_lab_range);

        WKmeans depth_clusterer;
        depth_clusterer.AddData(depth_lab_center, depth_lab_range);
        cv::Mat refined_depth_lab_center;
        depth_clusterer.W_Cluster(refined_depth_lab_center, KK, T);

        saveMat(in_path+"/refined_depth_lab_center_"+ss.str()+".cvmat", refined_depth_lab_center);
    }
    {
        cv::Mat color_xyz_center, color_xyz_range;
        readMat(in_path+"/color_xyz_center.cvmat", color_xyz_center);
        readMat(in_path+"/color_xyz_range.cvmat", color_xyz_range);
        std::cerr << color_xyz_center.rows << std::endl;
        color_xyz_range = color_xyz_range.mul(color_xyz_range);

        WKmeans color_clusterer;
        color_clusterer.AddData(color_xyz_center, color_xyz_range);
        cv::Mat refined_color_xyz_center;
        color_clusterer.W_Cluster(refined_color_xyz_center, KK, T);

        saveMat(in_path+"/refined_color_xyz_center_"+ss.str()+".cvmat", refined_color_xyz_center);
    }
    {
        cv::Mat color_lab_center, color_lab_range;
        readMat(in_path+"/color_lab_center.cvmat", color_lab_center);
        readMat(in_path+"/color_lab_range.cvmat", color_lab_range);
        std::cerr << color_lab_center.rows << std::endl;
        color_lab_range = color_lab_range.mul(color_lab_range);

        WKmeans color_clusterer;
        color_clusterer.AddData(color_lab_center, color_lab_range);
        cv::Mat refined_color_lab_center;
        color_clusterer.W_Cluster(refined_color_lab_center, KK, T);

        saveMat(in_path+"/refined_color_lab_center_"+ss.str()+".cvmat", refined_color_lab_center);
    }
    
    return 1;
}
//*/


/*
    cv::Mat tmp;
    readMat(in_path+"/refined_color_lab_center_40.cvmat", tmp);
    std::ofstream fp;
    fp.open("/home/chi/visualize/refined_color_lab_center_40.txt", std::ios::out);
    for( int i = 0 ; i < tmp.rows; i++ )
    {
        for(int j = 0 ; j < tmp.cols ; j++)
            fp << tmp.at<float>(i, j) << " ";
        fp << std::endl;
    }
    fp.close();
    return 1;
    */

/*
    cv::Mat depth_center, depth_range;
    readMat(in_path+"/depth_center.cvmat", depth_center);
    readMat(in_path+"/depth_range.cvmat", depth_range);
    
    cv::Mat color_center, color_range;
    readMat(in_path+"/color_center.cvmat", color_center);
    readMat(in_path+"/color_range.cvmat", color_range);
    
    std::cerr << depth_center.rows << std::endl;
    std::cerr << color_center.rows << std::endl;
    
    depth_range = depth_range.mul(depth_range);
    //std::cerr << depth_range << std::endl;
    //std::cin.get();
    color_range = color_range.mul(color_range);
    
    WKmeans depth_clusterer;
    depth_clusterer.AddData(depth_center, depth_range);
    cv::Mat new_depth_center;
    depth_clusterer.W_Cluster(new_depth_center, KK, T);
    
    saveMat(in_path+"/new_depth_center_"+ss.str()+".cvmat", new_depth_center);
    
    WKmeans color_clusterer;
    color_clusterer.AddData(color_center, color_range);
    cv::Mat new_color_center;
    color_clusterer.W_Cluster(new_color_center, KK, T);
    
    saveMat(in_path+"/new_color_center_"+ss.str()+".cvmat", new_color_center);
    
    
    std::cerr << new_depth_center.rows << std::endl;
    std::cerr << new_color_center.rows << std::endl;
    
    //std::cerr << refined_depth_center << std::endl;
    //std::cerr << refined_depth_range << std::endl;
    */

/*
    cv::Mat depth_center, depth_range;
    readMat(in_path+"/depth_center.cvmat", depth_center);
    readMat(in_path+"/depth_range.cvmat", depth_range);
    
    cv::Mat color_center, color_range;
    readMat(in_path+"/color_center.cvmat", color_center);
    readMat(in_path+"/color_range.cvmat", color_range);
    
    std::cerr << depth_center.rows << std::endl;
    std::cerr << color_center.rows << std::endl;
    
    depth_range *= tt;
    color_range *= tt;
    
    cv::Mat refined_depth_center, refined_depth_range;
    cv::Mat refined_color_center, refined_color_range;
    refineCenters(depth_center, depth_range, refined_depth_center, refined_depth_range);
    refineCenters(color_center, color_range, refined_color_center, refined_color_range);
    
    std::cerr << refined_depth_center.rows << std::endl;
    std::cerr << refined_color_center.rows << std::endl;
    
    saveMat(in_path+"/refined_depth_center_L0.cvmat", refined_depth_center);
    saveMat(in_path+"/refined_color_center_L0.cvmat", refined_color_center);
    saveMat(in_path+"/refined_depth_range_L0.cvmat", refined_depth_range);
    saveMat(in_path+"/refined_color_range_L0.cvmat", refined_color_range);
    */