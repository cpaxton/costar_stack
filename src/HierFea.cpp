#include <opencv2/core/core.hpp>

#include "sp_segmenter/features.h"

Hier_Pooler::Hier_Pooler(float rad)
{
    pool_flag[0] = true;
    pool_flag[1] = true;
    pool_flag[2] = true;
    pool_flag[3] = true;
    
    pool_type_num = 4;
    
    pool_radius_L0 = rad;
    ratio = 0;  //ratio = 0.05;
}

Hier_Pooler::~Hier_Pooler(){}

std::vector<int> Hier_Pooler::LoadDict_L0(std::string dict_path, std::string colorK, std::string depthK, std::string jointK)
{
    int tmp;
    std::cerr << "Loading Dictionary L0-0: " << dict_path + "dict_color_L0_"+colorK+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L0-1: " << dict_path + "dict_depth_L0_"+depthK+".cvmat" << std::endl;
    
    LoadSeedsHigh(dict_path + "dict_color_L0_"+colorK+".cvmat", tree_color_L0, dict_color_L0, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_depth_L0_"+depthK+".cvmat", tree_depth_L0, dict_depth_L0, tmp, ratio);
    
    if( exists_test(dict_path + "dict_joint_L0_"+jointK+".cvmat") == true )
    {
        readMat(dict_path + "dict_joint_L0_"+jointK+".cvmat", dict_joint_L0);
    
        //cv::flann::LinearIndexParams indexParams;
        cv::flann::KDTreeIndexParams indexParams;
#ifdef opencv_miniflann_build_h
        extFlannIndexBuild(tree_joint_L0,dict_joint_L0, indexParams);
#else
        tree_joint_L0.build(dict_joint_L0, indexParams);
#endif
//        LoadSeedsHigh(dict_path + "dict_joint_L0_"+jointK+".cvmat", tree_joint_L0, dict_joint_L0, tmp, ratio);
        std::cerr << "Loading Dictionary L0-2: " << dict_path + "dict_joint_L0_"+jointK+".cvmat" << std::endl;
    }
    std::vector<int> fea_dim;
    fea_dim.push_back(dict_color_L0.rows);
    fea_dim.push_back(dict_depth_L0.rows);
    
    return fea_dim;
}

std::vector<int> Hier_Pooler::LoadDict_L1(std::string dict_path, std::vector<std::string> dictK)
{
    int tmp;
    std::cerr << "Loading Dictionary L1-0: " << dict_path + "dict_colorInLAB_L1_"+dictK[0]+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L1-1: " << dict_path + "dict_depthInLAB_L1_"+dictK[1]+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L1-2: " << dict_path + "dict_colorInXYZ_L1_"+dictK[2]+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L1-3: " << dict_path + "dict_depthInXYZ_L1_"+dictK[3]+".cvmat" << std::endl;
    
    
    LoadSeedsHigh(dict_path + "dict_colorInLAB_L1_"+dictK[0]+".cvmat", tree_colorInLAB_L1, dict_colorInLAB_L1, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_depthInLAB_L1_"+dictK[1]+".cvmat", tree_depthInLAB_L1, dict_depthInLAB_L1, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_colorInXYZ_L1_"+dictK[2]+".cvmat", tree_colorInXYZ_L1, dict_colorInXYZ_L1, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_depthInXYZ_L1_"+dictK[3]+".cvmat", tree_depthInXYZ_L1, dict_depthInXYZ_L1, tmp, ratio);
    
    std::vector<int> fea_dim;
    fea_dim.push_back(dict_colorInLAB_L1.rows);
    fea_dim.push_back(dict_depthInLAB_L1.rows);
    fea_dim.push_back(dict_colorInXYZ_L1.rows);
    fea_dim.push_back(dict_depthInXYZ_L1.rows);
    
    return fea_dim;
}

std::vector<int> Hier_Pooler::LoadDict_L2(std::string dict_path, std::vector<std::string> dictK)
{
    int tmp;
    std::cerr << "Loading Dictionary L2-0: " << dict_path + "dict_colorInLAB_L2_"+dictK[0]+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L2-1: " << dict_path + "dict_depthInLAB_L2_"+dictK[1]+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L2-2: " << dict_path + "dict_colorInXYZ_L2_"+dictK[2]+".cvmat" << std::endl;
    std::cerr << "Loading Dictionary L2-3: " << dict_path + "dict_depthInXYZ_L2_"+dictK[3]+".cvmat" << std::endl;
    
    LoadSeedsHigh(dict_path + "dict_colorInLAB_L2_"+dictK[0]+".cvmat", tree_colorInLAB_L2, dict_colorInLAB_L2, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_depthInLAB_L2_"+dictK[1]+".cvmat", tree_depthInLAB_L2, dict_depthInLAB_L2, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_colorInXYZ_L2_"+dictK[2]+".cvmat", tree_colorInXYZ_L2, dict_colorInXYZ_L2, tmp, ratio);
    LoadSeedsHigh(dict_path + "dict_depthInXYZ_L2_"+dictK[3]+".cvmat", tree_depthInXYZ_L2, dict_depthInXYZ_L2, tmp, ratio);
    
    std::vector<int> fea_dim;
    fea_dim.push_back(dict_colorInLAB_L2.rows);
    fea_dim.push_back(dict_depthInLAB_L2.rows);
    fea_dim.push_back(dict_colorInXYZ_L2.rows);
    fea_dim.push_back(dict_depthInXYZ_L2.rows);
    
    return fea_dim;
}

void Hier_Pooler::computeRaw_L0(MulInfoT &data, cv::Mat& depth_fea, cv::Mat& color_fea, float rad)
{
    cv::Mat high_fea = cshot_cloud_ss(data.cloud, data.cloud_normals, data.down_lrf, data.down_cloud, rad, -1);
    
    depth_fea = cv::Mat::zeros(high_fea.rows, 352, CV_32FC1);
    high_fea.colRange(0, 352).copyTo(depth_fea);
    color_fea = cv::Mat::zeros(high_fea.rows, 992, CV_32FC1);
    high_fea.colRange(352, 1344).copyTo(color_fea);
    
    int num = data.down_cloud->size();
    #pragma omp parallel for
    for( int i = 0 ; i < num ; i++ )
    {
        cv::normalize(depth_fea.row(i),depth_fea.row(i));
        cv::normalize(color_fea.row(i),color_fea.row(i));
    }
    
}

std::vector<int> Hier_Pooler::sampleRaw_L0(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr cloud_normal,
                    cv::Mat &depth_fea, cv::Mat &color_fea, int max_num, float rad)
{
    std::vector<int> idxs;
    cv::Mat high_fea = cshot_cloud_uni(cloud, cloud_normal, idxs, rad, max_num);
    
    depth_fea = cv::Mat::zeros(high_fea.rows, 352, CV_32FC1);
    high_fea.colRange(0, 352).copyTo(depth_fea);
    color_fea = cv::Mat::zeros(high_fea.rows, 992, CV_32FC1);
    high_fea.colRange(352, 1344).copyTo(color_fea);
    
    int num = idxs.size();
    for( int i = 0 ; i < num ; i++ )
    {
        cv::normalize(depth_fea.row(i),depth_fea.row(i));
        cv::normalize(color_fea.row(i),color_fea.row(i));
    }
    
    return idxs;
}

std::vector<cv::Mat> Hier_Pooler::EncodeLayer_L0(const cv::Mat &depth_fea, const cv::Mat &color_fea)
{
    int depth_len = dict_depth_L0.rows;
    int color_len = dict_color_L0.rows;
    int depthK = depth_len * ratio;
    int colorK = color_len * ratio;

    int num = depth_fea.rows <= color_fea.rows ? color_fea.rows : depth_fea.rows;
    
    std::vector<cv::Mat> depth_code_vec(num);  
    std::vector<cv::Mat> color_code_vec(num);
    #pragma omp parallel for
    for( int i = 0 ; i < num ; i++ )
    {
        depth_code_vec[i] = KNNEncoder(depth_fea.row(i), tree_depth_L0, depth_len, depthK);
        color_code_vec[i] = KNNEncoder(color_fea.row(i), tree_color_L0, color_len, colorK);
    }
    std::vector<cv::Mat> fea_codes(2);  //color, depth
    cv::vconcat(depth_code_vec, fea_codes[0]);
    cv::vconcat(color_code_vec, fea_codes[1]);
    
    return fea_codes;
}

std::vector<cv::Mat> Hier_Pooler::PoolLayer_L1(const MulInfoT &data, const std::vector<cv::Mat> code_L0, std::vector<size_t> idxs, bool max_pool)
{
    int num;
    bool subsampling = false;
    if( idxs.empty() == true )
        num = data.cloud->size();
    else
    {
        subsampling = true;
        num = idxs.size();
    }
    pcl::PointCloud<PointT>::ConstPtr temp_ptr = data.xyz_tree->getInputCloud();
    int total_num = temp_ptr->size();
    
    // Build Neighborhood Index
    std::vector< std::vector<int> > lab_neighs(num);
    std::vector< std::vector<int> > xyz_neighs(num);
    
    cv::Mat lab_ptr = data.rgb;
    for( int i = 0 ; i < num ; i++ )
    {
        int cur_idx = subsampling == true ? idxs[i] : i;
        
        myPointXYZ cur_lab_elem;
        cur_lab_elem.x = lab_ptr.at<float>(cur_idx, 0);
        cur_lab_elem.y = lab_ptr.at<float>(cur_idx, 1);
        cur_lab_elem.z = lab_ptr.at<float>(cur_idx, 2);
        
        std::vector<float> cur_lab_dists;
        std::vector<int> cur_lab_neighs;
        data.lab_tree->radiusSearch(cur_lab_elem, pool_radius_L1[0], cur_lab_neighs, cur_lab_dists, total_num);
        lab_neighs[i] = cur_lab_neighs;    
        
        std::vector<float> cur_xyz_dists;
        std::vector<int> cur_xyz_neighs;
        data.xyz_tree->radiusSearch(data.cloud->at(cur_idx), pool_radius_L1[1], cur_xyz_neighs, cur_xyz_dists, total_num);
        xyz_neighs[i] = cur_xyz_neighs;
        
        //std::cerr << cur_lab_neighs.size() << " " << cur_xyz_neighs.size() << " " << total_num << std::endl;
        //std::cin.get();
    }
    
    std::vector<cv::Mat> raw_fea_L1(pool_type_num);
    for( int j = 0 ; j < pool_type_num ; j++ )
    {
        cv::Mat feas;
        std::vector< std::vector<int> > *neighs;
        if( pool_flag[j] == true)
        {
            switch(j)
            {
                case 0:
                    feas = code_L0[1];  //color_fea -> lab
                    neighs = &lab_neighs;
                    break;
                case 1:
                    feas = code_L0[0];  //depth_fea -> lab
                    neighs = &lab_neighs;
                    break;
                case 2:
                    feas = code_L0[1];  //color_fea -> xyz
                    neighs = &xyz_neighs;
                    break;
                case 3:
                    feas = code_L0[0];  //depth_fea -> xyz
                    neighs = &xyz_neighs;
                    break;
                default:break;
            }
            
            // Pool L0 Features
            std::vector<cv::Mat> raw_fea_L1_vec(num);
            for( int i = 0 ; i < num ; i++ )
            {
                cv::Mat temp_fea = cv::Mat::zeros(1, feas.cols, CV_32FC1);
               
                for(std::vector<int>::iterator it_n = neighs->at(i).begin(); it_n < neighs->at(i).end() ; it_n++ )
                {
                    if( max_pool == false )
                        temp_fea += feas.row(*it_n);
                    else 
                        MaxOP(temp_fea, feas.row(*it_n));
                }  
                // Normalize Pool Raw Vector
                cv::normalize(temp_fea, temp_fea);
                raw_fea_L1_vec[i] = temp_fea;
            }
            
            cv::vconcat(raw_fea_L1_vec, raw_fea_L1[j]);
        }
    }
    
    return raw_fea_L1;
}

std::vector<cv::Mat> Hier_Pooler::PoolLayer_L2(const MulInfoT &data, const std::vector<cv::Mat> code_L1, std::vector<size_t> idxs, bool max_pool)
{
    int num;
    bool subsampling = false;
    if( idxs.empty() == true )
        num = data.cloud->size();
    else
    {
        subsampling = true;
        num = idxs.size();
    }
    pcl::PointCloud<PointT>::ConstPtr temp_ptr = data.xyz_tree->getInputCloud();
    int total_num = temp_ptr->size();
    
    // Build Neighborhood Index
    std::vector< std::vector<int> > lab_neighs(num);
    std::vector< std::vector<int> > xyz_neighs(num);
    
    cv::Mat lab_ptr = data.rgb;
    for( int i = 0 ; i < num ; i++ )
    {
        int cur_idx = subsampling == true ? idxs[i] : i;
        
        myPointXYZ cur_lab_elem;
        cur_lab_elem.x = lab_ptr.at<float>(cur_idx, 0);
        cur_lab_elem.y = lab_ptr.at<float>(cur_idx, 1);
        cur_lab_elem.z = lab_ptr.at<float>(cur_idx, 2);
        
        std::vector<float> cur_lab_dists;
        std::vector<int> cur_lab_neighs;
        data.lab_tree->radiusSearch(cur_lab_elem, pool_radius_L2[0], cur_lab_neighs, cur_lab_dists, total_num);
        lab_neighs[i] = cur_lab_neighs;    
        
        std::vector<float> cur_xyz_dists;
        std::vector<int> cur_xyz_neighs;
        data.xyz_tree->radiusSearch(data.cloud->at(cur_idx), pool_radius_L2[1], cur_xyz_neighs, cur_xyz_dists, total_num);
        xyz_neighs[i] = cur_xyz_neighs;
        
        //std::cerr << cur_lab_neighs.size() << " " << cur_xyz_neighs.size() << " " << total_num << std::endl;
        //std::cin.get();
    }
    
    std::vector<cv::Mat> raw_fea_L2(pool_type_num);
    for( int j = 0 ; j < pool_type_num ; j++ )
    {
        cv::Mat feas;
        std::vector< std::vector<int> > *neighs;
        if( pool_flag[j] == true)
        {
            switch(j)
            {
                case 0:
                    feas = code_L1[0];  //color_fea
                    neighs = &lab_neighs;
                    break;
                case 1:
                    feas = code_L1[1];  //depth_fea
                    neighs = &lab_neighs;
                    break;
                case 2:
                    feas = code_L1[2];  //color_fea
                    neighs = &xyz_neighs;
                    break;
                case 3:
                    feas = code_L1[3];  //depth_fea
                    neighs = &xyz_neighs;
                    break;
                default:break;
            }
            
            // Pool L0 Features
            std::vector<cv::Mat> raw_fea_L2_vec(num);
            for( int i = 0 ; i < num ; i++ )
            {
                cv::Mat temp_fea = cv::Mat::zeros(1, feas.cols, CV_32FC1);
               
                for(std::vector<int>::iterator it_n = neighs->at(i).begin(); it_n < neighs->at(i).end() ; it_n++ )
                {
                    if( max_pool == false )
                        temp_fea += feas.row(*it_n);
                    else 
                        MaxOP(temp_fea, feas.row(*it_n));
                }  
                // Normalize Pool Raw Vector
                cv::normalize(temp_fea, temp_fea);
                raw_fea_L2_vec[i] = temp_fea;
            }
            
            cv::vconcat(raw_fea_L2_vec, raw_fea_L2[j]);
        }
    }
    
    return raw_fea_L2;
}

std::vector<cv::Mat> Hier_Pooler::EncodeLayer_L1(const std::vector<cv::Mat> rawfea_L1)
{
    std::vector<cv::Mat> fea_L1(pool_type_num);
    for( int j = 0 ; j < pool_type_num ; j++ )
    {
        cv::flann::Index *fea_tree;
        int fea_len = 0, feaK;
        if( pool_flag[j] == true)
        {
            switch(j)
            {
                case 0:
                    fea_tree = &tree_colorInLAB_L1;
                    fea_len = dict_colorInLAB_L1.rows;
                    break;
               case 1:
                    fea_tree = &tree_depthInLAB_L1;
                    fea_len = dict_depthInLAB_L1.rows;
                    break;
                case 2:
                    fea_tree = &tree_colorInXYZ_L1;
                    fea_len = dict_colorInXYZ_L1.rows;
                    break;
                case 3:
                    fea_tree = &tree_depthInXYZ_L1;
                    fea_len = dict_depthInXYZ_L1.rows;
                    break;
                default:break;
            }
            feaK = fea_len * ratio;
            // Pool L0 Features
            int num = rawfea_L1[j].rows;
            std::vector<cv::Mat> fea_L1_vec(num);
            for( int i = 0 ; i < num ; i++ )
                fea_L1_vec[i] = KNNEncoder(rawfea_L1[j].row(i), *fea_tree, fea_len, feaK);
            
            cv::vconcat(fea_L1_vec, fea_L1[j]);
        }
    }
    
    return fea_L1;
}

std::vector<cv::Mat> Hier_Pooler::EncodeLayer_L2(const std::vector<cv::Mat> rawfea_L2)
{
    std::vector<cv::Mat> fea_L2(pool_type_num);
    for( int j = 0 ; j < pool_type_num ; j++ )
    {
        cv::flann::Index *fea_tree;
        int fea_len = 0, feaK;
        if( pool_flag[j] == true)
        {
            switch(j)
            {
                case 0:
                    fea_tree = &tree_colorInLAB_L2;
                    fea_len = dict_colorInLAB_L2.rows;
                    break;
                case 1:
                    fea_tree = &tree_depthInLAB_L2;
                    fea_len = dict_depthInLAB_L2.rows;
                    break;
                case 2:
                    fea_tree = &tree_colorInXYZ_L2;
                    fea_len = dict_colorInXYZ_L2.rows;
                    break;
                case 3:
                    fea_tree = &tree_depthInXYZ_L2;
                    fea_len = dict_depthInXYZ_L2.rows;
                    break;
                default:break;
            }
            feaK = fea_len * ratio;
            // Pool L0 Features
            int num = rawfea_L2[j].rows;
            std::vector<cv::Mat> fea_L2_vec(num);
            for( int i = 0 ; i < num ; i++ )
                fea_L2_vec[i] = KNNEncoder(rawfea_L2[j].row(i), *fea_tree, fea_len, feaK);
            
            cv::vconcat(fea_L2_vec, fea_L2[j]);
        }
    }
    
    return fea_L2;
}

std::vector<cv::Mat> Hier_Pooler::getHierFea( MulInfoT &data, int layer)
{
    std::vector<cv::Mat> final_fea;
    if( layer < 0 )
        exit(0);
    
    cv::Mat depth_fea, color_fea;
    //double t1, t2;
    //t1 = get_wall_time();
    computeRaw_L0(data, depth_fea, color_fea, pool_radius_L0);
    //t2 = get_wall_time();
    //std::cerr << "CSHOT Time-" << t2 - t1 << std::endl;
    
    //t1 = get_wall_time();
    std::vector<cv::Mat> fea_L0 = EncodeLayer_L0(depth_fea, color_fea);
    //t2 = get_wall_time();
    //std::cerr << "Encoding Time-" << t2 - t1 << std::endl;
    if( dict_joint_L0.empty() == true )
    {
        for( size_t i = 0 ; i < fea_L0.size() ; i++ )
            final_fea.push_back(fea_L0[i]);
        final_fea.push_back(depth_fea);
        final_fea.push_back(color_fea);
    }
    else
    {
        int num = fea_L0[0].rows;
        std::vector<cv::Mat> joint_code_vec(num);  
        
        int joint_len = dict_joint_L0.rows;
        int jointK = joint_len * ratio;
    
        for( int i = 0 ; i < num ; i++ )
        {
            cv::Mat tmp_fea;
            cv::hconcat(fea_L0[0].row(i), fea_L0[1].row(i), tmp_fea);
            joint_code_vec[i] = KNNEncoder(tmp_fea, tree_joint_L0, joint_len, jointK);
//            depth_code_vec[i] = KNNEncoder(depth_fea.row(i), tree_depth_L0, depth_len, depthK);
//            color_code_vec[i] = KNNEncoder(color_fea.row(i), tree_color_L0, color_len, colorK);
        }
        cv::Mat joint_fea;
        cv::vconcat(joint_code_vec, joint_fea);

        final_fea.push_back(joint_fea);
        
    }
    if( layer >= 1 )
    {    
        //buildIndex(data);
        std::vector<size_t> void_idx_L1;
        std::vector<cv::Mat> raw_fea_L1 = PoolLayer_L1(data, fea_L0, void_idx_L1);
        std::vector<cv::Mat> fea_L1 = EncodeLayer_L1(raw_fea_L1);
    
        for( size_t i = 0 ; i < fea_L1.size() ; i++ )
            final_fea.push_back(fea_L1[i]);
        
        if( layer >= 2 )
        {
            std::vector<size_t> void_idx_L2;
            std::vector<cv::Mat> raw_fea_L2 = PoolLayer_L2(data, fea_L1, void_idx_L2);
            std::vector<cv::Mat> fea_L2 = EncodeLayer_L2(raw_fea_L2);

            for( size_t i = 0 ; i < fea_L2.size() ; i++ )
                final_fea.push_back(fea_L2[i]);
        }
    }
    
    return final_fea;
}

std::vector<cv::Mat> Hier_Pooler::getRawFea( MulInfoT &data, int layer, size_t max_num)
{
    std::vector<cv::Mat> raw_fea;
    cv::Mat depth_fea, color_fea;
        
    if( layer <= 0 )
    {
        std::vector<int> active_idx = sampleRaw_L0(data.cloud, data.cloud_normals, depth_fea, color_fea, max_num, pool_radius_L0);
        
        raw_fea.push_back(depth_fea);
        raw_fea.push_back(color_fea);
        
        if( layer == 0 )
        {
            cv::Mat xyz_lab  = cv::Mat::zeros(active_idx.size(), 6, CV_32FC1);
            for( size_t i = 0 ; i < active_idx.size() ; i++ )
            {
                data.xyz.row(active_idx[i]).copyTo(xyz_lab.row(i).colRange(0,3));
                data.rgb.row(active_idx[i]).copyTo(xyz_lab.row(i).colRange(3,6));
            }
            std::vector<cv::Mat> code_L0 = EncodeLayer_L0(depth_fea, color_fea);
            raw_fea.push_back(code_L0[0]);  //depth_code
            raw_fea.push_back(code_L0[1]);  //color_code
            raw_fea.push_back(xyz_lab);     //xyz_lab
        }
        return raw_fea;
    }
    else
        computeRaw_L0(data, depth_fea, color_fea, pool_radius_L0);
    
    if( layer >= 1 )
    {
        //buildIndex(data);
        std::vector<cv::Mat> code_L0 = EncodeLayer_L0(depth_fea, color_fea);
        
        if( layer == 1 )
        {
            std::vector<size_t> rand_idx_L1;
            if( data.cloud->size() > max_num ) 
            {
                GenRandSeq(rand_idx_L1, data.cloud->size());
                rand_idx_L1.erase(rand_idx_L1.begin()+max_num, rand_idx_L1.end());
            }

            std::vector<cv::Mat> raw_fea_L1 = PoolLayer_L1(data, code_L0, rand_idx_L1);
            
            for( size_t i = 0 ; i < raw_fea_L1.size() ; i++ )
                raw_fea.push_back(raw_fea_L1[i]);
            return raw_fea;
        }
        else if( layer == 2 )
        {
            std::vector<size_t> void_idx_L1;
            std::vector<cv::Mat> raw_fea_L1 = PoolLayer_L1(data, code_L0, void_idx_L1);
            std::vector<cv::Mat> code_L1 = EncodeLayer_L1(raw_fea_L1);
            
            std::vector<size_t> rand_idx_L2;
            if( data.cloud->size() > max_num ) 
            {
                GenRandSeq(rand_idx_L2, data.cloud->size());
                rand_idx_L2.erase(rand_idx_L2.begin()+max_num, rand_idx_L2.end());
            }

            std::vector<cv::Mat> raw_fea_L2 = PoolLayer_L1(data, code_L1, rand_idx_L2);
            
            for( size_t i = 0 ; i < raw_fea_L2.size() ; i++ )
                raw_fea.push_back(raw_fea_L2[i]);
            return raw_fea;
        }
    }
        

    return raw_fea;
}
