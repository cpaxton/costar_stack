#include "sp_segmenter/features.h"

Pooler_L0::Pooler_L0(int hsi_len)
{
    hs_cell_scale = 1.00000001/hsi_len;
    len_h = hsi_len;
    len_s = hsi_len;
    len_i = hsi_len;
    
    total_hs_len = len_h*len_s*len_i;
    total_xy_len = len_h*len_s;
}

Pooler_L0::Pooler_L0()
{
    int hsi_len = 1;
    
    hs_cell_scale = 1.00000001/hsi_len;
    len_h = hsi_len;
    len_s = hsi_len;
    len_i = hsi_len;
    
    total_hs_len = len_h*len_s*len_i;
    total_xy_len = len_h*len_s;
}

void Pooler_L0::setHSIPoolingParams(int hsi_len)
{
    hs_cell_scale = 1.00000001/hsi_len;
    len_h = hsi_len;
    len_s = hsi_len;
    len_i = hsi_len;
    
    std::cerr<<"Setting HSI: "<<len_h<<" "<<len_s<<" "<<len_i<<std::endl;
    total_hs_len = len_h*len_s*len_i;
    total_xy_len = len_h*len_s;
}

/*
int Pooler_L0::getXYZPoolIdx(const cv::Mat &xyz)
{
    float x = xyz.at<float>(0, 0);
    float y = xyz.at<float>(0, 1);
    float z = xyz.at<float>(0, 2);
    
    if( max_x <= x ) x = max_x-0.001;
    if( max_y <= y ) y = max_y-0.001;
    if( max_z <= z ) z = max_z-0.001;

    if( min_x > x ) x = min_x;
    if( min_y > y ) y = min_y;
    if( min_z > z ) z = min_z;
    
    int idx_x = floor((x-min_x)/xyz_cell_scale);
    int idx_y = floor((y-min_y)/xyz_cell_scale);
    int idx_z = floor((z-min_z)/xyz_cell_scale);
    
    return idx_x*(len_y*len_z)+idx_y*len_z+idx_z;
}
//*/

int Pooler_L0::getXYPoolIdx(const cv::Mat &xyz)
{
    float x = xyz.at<float>(0, 0);
    float y = xyz.at<float>(0, 1);
    
    int idx_x = floor( x / hs_cell_scale );
    int idx_y = floor( y / hs_cell_scale );
    
    if( idx_x >= len_h )  idx_x = len_h - 1;
    if( idx_y >= len_s )  idx_y = len_s - 1;
    
    return idx_x*len_s + idx_y;
}

//*
int Pooler_L0::getHSIPoolIdx(const cv::Mat &hsi)
{
    float h = hsi.at<float>(0, 0);
    float s = hsi.at<float>(0, 1);
    float i = hsi.at<float>(0, 2);
    
    int idx_h = floor( h / hs_cell_scale );
    int idx_s = floor( s / hs_cell_scale );
    int idx_i = floor( i / hs_cell_scale);
    
    if( idx_h >= len_h )  idx_h = len_h - 1;
    if( idx_s >= len_s )  idx_s = len_s - 1;
    if( idx_i >= len_i )  idx_i = len_i - 1;
    
    return idx_h*len_s*len_i + idx_s*len_i+idx_i;
}
//*/

int Pooler_L0::getGenericPoolIdx(const cv::Mat &pool_fea)
{
    std::vector<int> index(1);
    std::vector<float> dist(1);

    pool_tree.knnSearch(pool_fea, index, dist, 1, cv::flann::SearchParams());
    
    return index[0];
}

cv::Mat Pooler_L0::getGenericPoolMat(const cv::Mat &domain)
{
    int data_num = domain.rows;
    cv::Mat pool_idx = cv::Mat::zeros(domain.rows, 1, CV_32SC1);
    int *ptr = (int *)pool_idx.data;
    for( int i = 0 ; i < data_num; i++, ptr++ )
        *ptr = getGenericPoolIdx(domain.row(i));
    return pool_idx;
}

cv::Mat Pooler_L0::PoolOneDomain(const cv::Mat &domain, const cv::Mat &fea_code, int pool_type, bool max_pool)
{
    if( domain.rows != fea_code.rows )
    {
        std::cerr << "domain.rows != fea_code.rows" << std::endl;
        exit(0);
    }
    int len;
    switch(pool_type)
    {
        case 0:     //xyz-pooling domain
            len = total_xy_len;
            break;
        case 1:     //hs-pooling domain
            len = total_hs_len;
            //len = rgb_seeds.rows;
            break;
        case 2:
            len = pool_len;
            break;
        case 3:
            len = pool_len;
            break;
        default:break;
    }
    
    std::vector<cv::Mat> fea_vec(len);
    for( int i = 0 ; i < len ; i++ )
        fea_vec[i] = cv::Mat::zeros(1, fea_code.cols, CV_32FC1);
    
    std::vector<int> count(len);
    for( int i = 0 ; i < domain.rows; i++ )
    {
        std::vector<int> idxs;
        std::vector<float> w;
        int idx;
        switch(pool_type)
        {
            case 0:     //xyz-pooling domain
                idx = getXYPoolIdx(domain.row(i));
                break;
            case 1:     //hs-pooling domain
                idx = getHSIPoolIdx(domain.row(i));
                break;
            case 2:
                idx = getGenericPoolIdx(domain.row(i));
                break;
            case 3:
                idx = domain.at<int>(i,0);
                break;
            default:break;
            
        } 
        if( pool_type < 10 )
        {
            count[idx]++;
            if( max_pool == false )
                fea_vec[idx] += fea_code.row(i);
            else 
                MaxOP(fea_vec[idx], fea_code.row(i));
        }
        else
        {
            for( size_t k = 0 ; k < idxs.size() ; k++ )
            {
                if( max_pool == false )
                    fea_vec[idxs[k]] += fea_code.row(i)*w[k];
                else 
                    MaxOP(fea_vec[idxs[k]], fea_code.row(i)*w[k]);
            }
        }
    }
    for( size_t i = 0 ; i < fea_vec.size() ; i++ )
        cv::normalize(fea_vec[i], fea_vec[i], 1.0, 0.0, cv::NORM_L2);
    
    cv::Mat final_fea;
    cv::hconcat(fea_vec, final_fea);
    
    //cv::normalize(final_fea, final_fea, 1.0, 0.0, cv::NORM_L2);
    
    return final_fea;
}

std::vector<cv::Mat> Pooler_L0::PoolOneDomain_Raw(const cv::Mat &domain, const cv::Mat &fea_code, int pool_type, bool max_pool)
{
    if( domain.rows != fea_code.rows )
    {
        std::cerr << "domain.rows != fea_code.rows" << std::endl;
        exit(0);
    }
    int len;
    switch(pool_type)
    {
        case 0:     //xyz-pooling domain
            len = total_xy_len;
            break;
        case 1:     //hs-pooling domain
            len = total_hs_len;
            //len = rgb_seeds.rows;
            break;
        case 2:
            len = pool_len;
            break;
        case 3:
            len = pool_len;
            break;
        default:break;
    }
    
    std::vector<cv::Mat> fea_vec(len);
    for( int i = 0 ; i < len ; i++ )
        fea_vec[i] = cv::Mat::zeros(1, fea_code.cols, CV_32FC1);
    
    std::vector<int> count(len);
    for( int i = 0 ; i < domain.rows; i++ )
    {
        std::vector<int> idxs;
        std::vector<float> w;
        int idx;
        switch(pool_type)
        {
            case 0:     //xyz-pooling domain
                idx = getXYPoolIdx(domain.row(i));
                break;
            case 1:     //hs-pooling domain
                idx = getHSIPoolIdx(domain.row(i));
                break;
            case 2:
                idx = getGenericPoolIdx(domain.row(i));
                break;
            case 3:
                idx = domain.at<int>(i,0);
                break;
            default:break;
            
        } 
        if( pool_type < 10 )
        {
            count[idx]++;
            if( max_pool == false )
                fea_vec[idx] += fea_code.row(i);
            else 
                MaxOP(fea_vec[idx], fea_code.row(i));
        }
        else
        {
            for( size_t k = 0 ; k < idxs.size() ; k++ )
            {
                if( max_pool == false )
                    fea_vec[idxs[k]] += fea_code.row(i)*w[k];
                else 
                    MaxOP(fea_vec[idxs[k]], fea_code.row(i)*w[k]);
            }
        }
    }
    
    return fea_vec;
}

int Pooler_L0::LoadSeedsPool(std::string pool_seed_file)
{
    readMat(pool_seed_file, pool_seeds);
    
    //for( int i = 0 ; i < pool_seeds.rows ; i++ )
    //    cv::normalize(pool_seeds.row(i), pool_seeds.row(i), 1.0, 0.0, cv::NORM_L2);
    //std::cerr << cv::norm(pool_seeds.row(i)) << std::endl;  
    
    cv::flann::LinearIndexParams indexParams;
    // cv::flann::KDTreeIndexParams indexParams;
#ifdef opencv_miniflann_build_h
    pool_tree = extFlannIndexBuild(pool_seeds, indexParams);
#else
    pool_tree.build(pool_seeds, indexParams);
#endif

    

    pool_len = pool_seeds.rows;

    return pool_len;
}

int Pooler_L0::LoadHybridPool(std::string hybrid_center_file, float radius)
{
    readMat(hybrid_center_file, hybrid_centers);
    //readMat(hybrid_range_file, hybrid_range);
    hybrid_range = radius * cv::Mat::ones(hybrid_centers.rows, 1, CV_32FC1);
    
    //hybrid_range = hybrid_range * scale;
    
    if( hybrid_centers.rows != hybrid_range.rows )
    {
        std::cerr << "hybrid_centers.rows != hybrid_range.rows" << std::endl;
        exit(0);
    }
    hybrid_len = hybrid_centers.rows;
    return hybrid_len;
}

cv::Mat Pooler_L0::PoolHybridDomain(const cv::Mat &mixed_domain, const cv::Mat &fea_code, bool max_pool)
{
    cv::flann::LinearIndexParams indexParams;
    //cv::flann::KDTreeIndexParams indexParams;
    cv::flann::Index mixed_tree;
#ifdef opencv_miniflann_build_h
    mixed_tree = extFlannIndexBuild(mixed_domain, indexParams);
#else
    mixed_tree.build(mixed_domain, indexParams);
#endif
    
    std::vector<cv::Mat> fea_vec(hybrid_len);
    for( int i = 0 ; i < hybrid_len ; i++ )
        fea_vec[i] = cv::Mat::zeros(1, fea_code.cols, CV_32FC1);
    
    int num = mixed_domain.rows;
    for(int i = 0 ; i < hybrid_len ; i++ )
    {
        std::vector<float> dists;
        std::vector<int> idxs;
        int found = mixed_tree.radiusSearch(hybrid_centers.row(i), idxs, dists, hybrid_range.at<float>(i, 0), num, cv::flann::SearchParams());
        
        for( int k = 0 ; k < found ; k++ )
            MaxOP(fea_vec[i], fea_code.row(idxs[k]));
    }
    for( size_t i = 0 ; i < fea_vec.size() ; i++ )
        cv::normalize(fea_vec[i], fea_vec[i], 1.0, 0.0, cv::NORM_L2);
    
    cv::Mat final_fea;
    cv::hconcat(fea_vec, final_fea);
    
    return final_fea;
}

