#include "sp_segmenter/features.h"

#ifdef opencv_miniflann_build_h

//#include <opencv2/opencv.hpp>
cv::flann::Index extFlannIndexBuild(cv::InputArray _data, const cv::flann::IndexParams& params, const cvflann::flann_distance_t _distType)
{
    //    cv::InputArray * _wholedata = &(_data);
    cv::flann::Index result;
    result.build( _data, _data, params, _distType);
    return result;
}
#endif

void setObjID(std::map<std::string, int> &model_name_map)
{
    model_name_map["drill"] = 1;
    model_name_map["driller_small"] = 2;
    model_name_map["drill_flat"] = 3;
    model_name_map["drill_point"] = 4;
    model_name_map["mallet_ball_pein"] = 5;
    model_name_map["mallet_black_white"] = 6;
    model_name_map["mallet_drilling"] = 7;
    model_name_map["mallet_fiber"] = 8;
    model_name_map["old_hammer"] = 9;
    model_name_map["sander"] = 10;
}

bool overlap(const Hypo &hp1, const Hypo &hp2, float ratio)
{
    cv::Rect isect = hp1.box & hp2.box;
    return (isect.area()+0.0)/(hp1.box.area() + hp2.box.area() - isect.area()) > ratio;
}

bool tmp_overlap(const Hypo &hypo, const Hypo &gt, float ratio)
{
    cv::Rect isect = hypo.box & gt.box;
    
    return (isect.area()+0.0) / hypo.box.area() > ratio;
}

void poolRGBLayer(Pooler_L0 &pooler, const MulInfoT &inst, const cv::Mat &local_fea, std::vector<cv::Mat> &pool_fea_vec)
{
    bool max_flag = false;
    cv::Mat temp = pooler.PoolOneDomain(inst.rgb, local_fea, 1, max_flag);   //lab -> depth
    pool_fea_vec.push_back(temp);
}

void poolXYZLayer(Pooler_L0 &pooler, const MulInfoT &inst, const cv::Mat &local_fea, std::vector<cv::Mat> &pool_fea_vec)
{
    bool max_flag = false;
    cv::Mat temp = pooler.PoolOneDomain(inst.xyz, local_fea, 1, max_flag);   //lab -> depth
    pool_fea_vec.push_back(temp);
}

cv::Mat multiFPFHPool(const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const MulInfoT &inst, const std::vector<cv::Mat> &local_fea, float radius)
{
    cv::Mat fpfh = fpfh_cloud(inst.cloud, inst.down_cloud, inst.cloud_normals, radius, true);
    
    std::vector<cv::Mat> pool_fea_vec;
    for( int j = 1 ; j < pooler_set.size() ; j++ )
    {
        cv::Mat cur_final1 = pooler_set[j]->PoolOneDomain(fpfh, local_fea[0], 2, false);
        cv::Mat cur_final2 = pooler_set[j]->PoolOneDomain(fpfh, local_fea[1], 2, false);
        pool_fea_vec.push_back(cur_final1);
        pool_fea_vec.push_back(cur_final2);
    }
    cv::Mat final_fea;
    cv::hconcat(pool_fea_vec, final_fea);
    
    return final_fea;
}

cv::Mat multiPool(const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const MulInfoT &inst, const std::vector<cv::Mat> &local_fea)
{
    std::vector<cv::Mat> pool_fea_vec;
    int pooler_num = pooler_set.size();

    for( int k = 1 ; k < pooler_num ; k++ )
    {
        poolRGBLayer(*pooler_set[k], inst, local_fea[0], pool_fea_vec);
        poolRGBLayer(*pooler_set[k], inst, local_fea[1], pool_fea_vec);
    }

    //for( int k = 1 ; k < pooler_num ; k++ )
    //{
        //poolXYZLayer(*pooler_set[k], inst, local_fea[0], pool_fea_vec);
    //for( int k = 1 ; k < pooler_num ; k++ )
        //poolXYZLayer(*pooler_set[k], inst, local_fea[1], pool_fea_vec);
    //}
    cv::Mat final_fea;
    cv::hconcat(pool_fea_vec, final_fea);

    return final_fea;
}

std::vector<cv::Mat> multiPool_raw(const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const MulInfoT &inst, const std::vector<cv::Mat> &local_fea)
{
    std::vector<cv::Mat> pool_fea_vec;
    int pooler_num = pooler_set.size();
    
    for( int k = 1 ; k < pooler_num ; k++ )
    {
        std::vector<cv::Mat> temp_fea = pooler_set[k]->PoolOneDomain_Raw(inst.rgb, local_fea[0], 1, false);
        pool_fea_vec.insert(pool_fea_vec.end(), temp_fea.begin(), temp_fea.end());
    }
    for( int k = 1 ; k < pooler_num ; k++ )
    {
        std::vector<cv::Mat> temp_fea = pooler_set[k]->PoolOneDomain_Raw(inst.rgb, local_fea[1], 1, false);
        pool_fea_vec.insert(pool_fea_vec.end(), temp_fea.begin(), temp_fea.end());
    }
    
//    cv::Mat final_fea;
//    cv::hconcat(pool_fea_vec, final_fea);
    
    return pool_fea_vec;
}

void CvMatToFeatureNode(cv::Mat one_fea, sparseVec &fea_vec)
{
    fea_vec.clear();
    
    for( int c = 0 ; c < one_fea.cols ; c++ ){
        float cur_val = one_fea.at<float>(0, c);
        if( fabs(cur_val) >= 1e-6 && cur_val == cur_val) 
        {
            feature_node cur_node;
            cur_node.index = c+1;  
            cur_node.value = cur_val;
            fea_vec.push_back(cur_node);
        }
    }
    
}

std::pair<float, float> readBoxFile(std::string filename)
{
    std::ifstream fp(filename.c_str());
    if( fp.is_open() == false )
    {
        std::cerr << filename << " does not exist!" << std::endl;
        exit(0);
    }
    std::pair<float, float> box;
    fp >> box.first >> box.second;
    
    fp.close();
    return box;
}

int readGround(std::string filename, std::vector< std::vector<Hypo> > &hypo_set)
{
    std::cerr << "Loading Ground Truth: " << filename << std::endl;
    std::ifstream fp;
    fp.open(filename.c_str(), std::ios::in);
    
    int n, frame_num;
    fp >> n >> frame_num;
    //std::cerr << n << " " << frame_num << std::endl;
    hypo_set.clear();
    hypo_set.resize(frame_num);
    for(int i = 0 ; i < n ; i++ )
    {
        Hypo cur_hypo;
        int frame_id, label, tl_x, tl_y, br_x, br_y;
        fp >> frame_id >> label >> tl_x >> tl_y >> br_x >> br_y;
        //std::cerr << frame_id << " " << label<< " " << tl_x<< " " << tl_y<< " " << br_x<< " " << br_y << std::endl;
        cur_hypo.ap_ratio = 1.0;
        cur_hypo.label = label;
        cur_hypo.box.x = tl_x;
        cur_hypo.box.y = tl_y;
        cur_hypo.box.width = br_x - tl_x;
        cur_hypo.box.height = br_y - tl_y;
        
        int fid = frame_id - 1; //frame index starting at 1
        hypo_set[fid].push_back(cur_hypo);
    }
    fp.close();
    return frame_num;
}

void saveGround(std::string filename, const std::vector< std::vector<Hypo> > &hypo_set)
{
    std::cerr << "Saving Ground Truth: " << filename << std::endl;
    std::ofstream fp;
    fp.open(filename.c_str(), std::ios::out);
    
    int n = 0, frame_num;
    frame_num = hypo_set.size();
    for( size_t i = 0 ; i < hypo_set.size() ; i++ )
        n += hypo_set[i].size();
    
    fp << n << " " << frame_num << std::endl;
    
    for( int i = 0 ; i < hypo_set.size() ; i++ )
    {
        for( int j = 0 ; j < hypo_set[i].size() ; j++ )
        {
            //frame index starting at 1
            fp << i+1 << " " << hypo_set[i][j].label 
                    << " " << hypo_set[i][j].box.tl().x
                    << " " << hypo_set[i][j].box.tl().y
                    << " " << hypo_set[i][j].box.br().x
                    << " " << hypo_set[i][j].box.br().y
                    << std::endl;
        }
    }
    fp.close();
}

std::vector< std::pair<float, float> > PRScore(const std::vector< std::vector<Hypo> > &hypo_set, const std::vector< std::vector<Hypo> > &gt_set, int class_num)
{
    if( hypo_set.size() != gt_set.size() )
    {
        std::cerr << "hypo_set.size() != gt_set.size()" << std::endl;
        exit(0);
    }
    
    int frame_num = hypo_set.size();
    std::vector<size_t> true_pos(class_num+1, 0);  //start from index 1, the same with the label from liblinear
    std::vector<size_t> false_pos(class_num+1, 0);
    std::vector<size_t> total_pos(class_num+1, 0);
    
    for( int i = 0 ; i < frame_num ; i++ )
        for( std::vector<Hypo>::const_iterator it = gt_set[i].begin() ; it < gt_set[i].end() ; it++ )
            total_pos[it->label]++;
    
    for( int i = 0 ; i < frame_num ; i++ ){
        std::vector<bool> gt_flags(gt_set[i].size(), false);
        for( std::vector<Hypo>::const_iterator it = hypo_set[i].begin() ; it < hypo_set[i].end() ; it++ )
        {
            bool true_pos_flag = false;
            bool dup_flag = false;
            for( size_t j = 0 ; j < gt_set[i].size() ; j++ )
            {
                if( gt_set[i][j].label == it->label && tmp_overlap(*it, gt_set[i][j], 0.5) == true )//overlap(*it, gt_set[i][j], 0.5) == true )
                {
                    true_pos_flag = true;
                    if( gt_flags[j] == false )
                        gt_flags[j] = true;
                    else
                        dup_flag = true;
                    break;
                }
            }
            if( true_pos_flag ==true && dup_flag == false )
                true_pos[it->label]++;
            else if (true_pos_flag == false )
                false_pos[it->label]++;
        }
    }
    std::vector< std::pair<float, float> > pr_set(class_num+1, std::pair<float, float> (-1.0, -1.0));
    
    for( size_t i = 1 ; i < pr_set.size() ; i++ )
    {
        if( total_pos[i] > 0 )
        {
            pr_set[i].first = (true_pos[i] + 0.0) / (true_pos[i] + false_pos[i]);
            pr_set[i].second = (true_pos[i] + 0.0) / total_pos[i];
            
            std::cerr << "Class---" << i << "\tPrecision - "<< pr_set[i].first << "\tRecall - "<< pr_set[i].second << std::endl;
        }
    }
    
    return pr_set;
}

void SceneOn2D(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &uv, cv::Mat &map2d, float ap_ratio, float fx, float fy, float center_x, float center_y)
{
    size_t num = cloud->size();
    
    int img_h = 480 * ap_ratio;
    int img_w = 640 * ap_ratio;
    
    map2d = cv::Mat::ones(img_h, img_w, CV_32SC1)*-1;
    uv = cv::Mat::zeros(num, 2, CV_32SC1);
    int *ptr_uv = (int *)uv.data;
    
    for( size_t i = 0 ; i < num ; i++ )
    {
        PointT *pt_ptr = &cloud->at(i);
        int img_x = round(((*pt_ptr).x / (*pt_ptr).z * fx + center_x)*ap_ratio);
        int img_y = round(((*pt_ptr).y / (*pt_ptr).z * fy + center_y)*ap_ratio);
        if( img_x < 0 ) img_x = 0; 
        if( img_y < 0 ) img_y = 0;
        if( img_x >= img_w ) img_x = img_w-1;
        if( img_y >= img_h ) img_y = img_h-1;
            
        map2d.at<int>(img_y, img_x) = i;
        *ptr_uv = img_y; ptr_uv++;
        *ptr_uv = img_x; ptr_uv++;
               
    } 
}

std::vector<cv::Rect> UniformCubing(const pcl::PointCloud<PointT>::Ptr cloud, const cv::Mat &map2d, const std::vector< std::pair<float, float> > &box_size, int step, 
        float ap_ratio, float fx, float fy, float center_x, float center_y)
{
    int img_w = 640 * ap_ratio;
    int img_h = 480 * ap_ratio;
    
    std::vector<cv::Rect> regs;
            
    if( cloud->empty() || map2d.rows == 0 )
    {       
        std::cerr << "No data for region proposal!" << std::endl;
        return regs;
    }       
    
    for( std::vector< std::pair<float, float> >::const_iterator it_b = box_size.begin() ; it_b < box_size.end() ; it_b++ )
    {
        float box_w = it_b->first;
        float box_h = it_b->second;
        
        for( int r = step ; r < img_h - step; r+=step ){
            for( int c = step ; c < img_w - step; c+=step ){

                int idx = map2d.at<int>(r, c);
                if( idx < 0 )
                    continue;
                PointT *pt_ptr = &cloud->at(idx);

                cv::Rect cur_rect;

                int tl_x = ((pt_ptr->x - box_w) / pt_ptr->z * fx + center_x) * ap_ratio;
                int tl_y = ((pt_ptr->y - box_h) / pt_ptr->z * fy + center_y) * ap_ratio;

                int br_x = ((pt_ptr->x + box_w) / pt_ptr->z * fx + center_x) * ap_ratio;
                int br_y = ((pt_ptr->y + box_h) / pt_ptr->z * fy + center_y) * ap_ratio;

                if( tl_x < 0 ) tl_x = 0; 
                if( tl_y < 0 ) tl_y = 0;
                if( br_x >= img_w ) br_x = img_w - 1;
                if( br_y >= img_h ) br_y = img_h - 1;

                cur_rect.x = tl_x;
                cur_rect.y = tl_y;

                cur_rect.width = br_x - tl_x + 1;
                cur_rect.height = br_y - tl_y + 1;

                regs.push_back(cur_rect);
            }
        }
    }
    return regs;
}

cv::Mat getImage(const pcl::PointCloud<PointT>::Ptr cloud, float ap_ratio, float fx, float fy, float center_x, float center_y)
{
    int img_h = 480 * ap_ratio;
    int img_w = 640 * ap_ratio;
    
    cv::Mat img = cv::Mat::zeros(img_h, img_w, CV_8UC3);
    for( size_t i = 0 ; i < cloud->size() ; i++ )
    {
        PointT *pt_ptr = &cloud->at(i);
        uint32_t rgb = pt_ptr->rgba;
        
        int img_x = round(((*pt_ptr).x / (*pt_ptr).z * fx + center_x)*ap_ratio);
        int img_y = round(((*pt_ptr).y / (*pt_ptr).z * fy + center_y)*ap_ratio);
        if( img_x < 0 ) img_x = 0; 
        if( img_y < 0 ) img_y = 0;
        if( img_x >= img_w ) img_x = img_w-1;
        if( img_y >= img_h ) img_y = img_h-1;
        
        img.at<uchar>(img_y, img_x*3+2) = (rgb >> 16) & 0x0000ff;
        img.at<uchar>(img_y, img_x*3+1) = (rgb >> 8) & 0x0000ff;
        img.at<uchar>(img_y, img_x*3+0) = (rgb) & 0x0000ff;
    }
    
    return img;
}

pcl::PointIndices::Ptr getSegIdx(const cv::Mat &map2d, const cv::Rect &reg)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    int tl_x = reg.tl().x;
    int tl_y = reg.tl().y;
    int br_x = reg.br().x;
    int br_y = reg.br().y;
    
    for( int r = tl_y ; r < br_y ; r++ ){
        for( int c = tl_x ; c < br_x ; c++ )
        {
            int idx = map2d.at<int>(r, c);
            if( idx >= 0  )
                inliers->indices.push_back(idx);
        }
    }
    //if( (inliers->indices.size() + 0.0) / reg.area() < IN_TO_BOX_RATIO )
    //    inliers->indices.clear();
    return inliers;
}

std::vector<pcl::PointIndices::Ptr> CropSegs(const MulInfoT &data, const std::vector< std::pair<float, float> > &box_size, 
        int max_rand_sample, float in_box_ratio, float fx, float fy, float center_x_, float center_y_)
{
    // Random Sampling Segments
    size_t num = data.down_cloud->size();
    
    int img_h = 480;
    int img_w = 640;
    int center_x = center_x_;
    int center_y = center_y_;
    
    std::vector<pcl::PointIndices::Ptr> inlier_set;
    
    cv::Mat map2d = cv::Mat::ones(img_h, img_w, CV_32SC1)*-1;
    int max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;
    for( size_t i = 0 ; i < num ; i++ )
    {
        PointT *pt_ptr = &data.down_cloud->at(i);
        int img_x = round(pt_ptr->x / pt_ptr->z * fx + center_x);
        int img_y = round(pt_ptr->y / pt_ptr->z * fy + center_y);
        
        if( img_x < 0 ) img_x = 0; 
        if( img_y < 0 ) img_y = 0;
        if( img_x >= img_w ) img_x = img_w-1;
        if( img_y >= img_h ) img_y = img_h-1;
            
        map2d.at<int>(img_y, img_x) = i;
        
        if( img_x > max_x ) max_x = img_x;
        if( img_x < min_x ) min_x = img_x;
        if( img_y > max_y ) max_y = img_y;
        if( img_y < min_y ) min_y = img_y;
    } 
//    std::cerr << min_x << " " << max_x << " " << min_y << " " << max_y << std::endl;
    cv::Mat dense2d = cv::Mat::ones(img_h, img_w, CV_32SC1)*-1;
    size_t dense_num = data.cloud->size();
    for( size_t i = 0 ; i < dense_num ; i++ )
    {
        PointT *pt_ptr = &data.cloud->at(i);
        int img_x = round(pt_ptr->x / pt_ptr->z * fx + center_x);
        int img_y = round(pt_ptr->y / pt_ptr->z * fy + center_y);
        
        if( img_x < 0 ) img_x = 0; 
        if( img_y < 0 ) img_y = 0;
        if( img_x >= img_w ) img_x = img_w-1;
        if( img_y >= img_h ) img_y = img_h-1;
            
        dense2d.at<int>(img_y, img_x) = i;
    } 
    
    for( std::vector< std::pair<float, float> >::const_iterator it = box_size.begin() ; it < box_size.end() ; it++ )
    { 
        std::vector<size_t> rand_idx;
        GenRandSeq(rand_idx, num);
    
        int sample_count = 0;
        for( size_t i = 0 ; i < num ; i++ )
        {
            PointT *pt_ptr = &data.down_cloud->at(rand_idx[i]);
            
            cv::Rect cur_rect;
            
            int tl_x = (pt_ptr->x - it->first)  / pt_ptr->z * fx + center_x;
            int tl_y = (pt_ptr->y - it->second) / pt_ptr->z * fy + center_y;
            
            int br_x = (pt_ptr->x + it->first)  / pt_ptr->z * fx + center_x;
            int br_y = (pt_ptr->y + it->second) / pt_ptr->z * fy + center_y;
            
            if( tl_x < min_x ) tl_x = min_x; 
            if( tl_y < min_y ) tl_y = min_y;
            if( br_x > max_x ) br_x = max_x;
            if( br_y > max_y ) br_y = max_y;
            
            cur_rect.x = tl_x;
            cur_rect.y = tl_y;

            cur_rect.width = br_x - tl_x + 1;
            cur_rect.height = br_y - tl_y + 1;
            if( cur_rect.area() < 200 )
                continue;
            
            pcl::PointIndices::Ptr cur_inlier = getSegIdx(map2d, cur_rect);
            pcl::PointIndices::Ptr dummy_inlier = getSegIdx(dense2d, cur_rect);
            
            float score = (dummy_inlier->indices.size() + 0.0) / cur_rect.area();
            if( score > in_box_ratio )
            {
                inlier_set.push_back(cur_inlier);
                sample_count++;
                if( sample_count >= max_rand_sample )
                    break;
            }
        }
    }
    return inlier_set;
}

std::vector<cv::Mat> extFea(const std::vector<cv::Mat> &main_fea, const std::vector<int> &idx)
{
    std::vector<cv::Mat> new_fea_set;
    for(size_t i = 0 ; i < main_fea.size() ; i++ )
    {
        cv::Mat cur_fea = cv::Mat::zeros(idx.size(), main_fea[i].cols, CV_32FC1);
        for( size_t j = 0 ; j < idx.size() ; j++ )
            main_fea[i].row(idx[j]).copyTo(cur_fea.row(j));
        new_fea_set.push_back(cur_fea);
    }
    
    return new_fea_set;
}

cv::Mat clOn2D(const pcl::PointCloud<PointT>::Ptr cloud, float fx, float fy, float center_x_, float center_y_)
{
    // Random Sampling Segments
    size_t num = cloud->size();
    
    int img_h = 480;
    int img_w = 640;
    int center_x = center_x_;
    int center_y = center_y_;
    
    cv::Mat map2d = cv::Mat::ones(img_h, img_w, CV_32SC1)*-1;
    for( size_t i = 0 ; i < num ; i++ )
    {
        PointT *pt_ptr = &cloud->at(i);
        int img_x = round(pt_ptr->x / pt_ptr->z * fx + center_x);
        int img_y = round(pt_ptr->y / pt_ptr->z * fy + center_y);
        
        if( img_x < 0 ) img_x = 0; 
        if( img_y < 0 ) img_y = 0;
        if( img_x >= img_w ) img_x = img_w-1;
        if( img_y >= img_h ) img_y = img_h-1;
            
        map2d.at<int>(img_y, img_x) = i;
    } 
    return map2d;
}

std::vector<pcl::PointIndices::Ptr> CropSegs(const MulInfoT &data, int min_num, int max_num, 
        int max_rand_sample, float fx, float fy, float center_x, float center_y)
{
    size_t num = data.down_cloud->size();
    std::vector<pcl::PointIndices::Ptr> inlier_set;
    
    if( min_num > num )
        return inlier_set;
    else if( min_num + max_rand_sample*1.5 > num )
    {
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        for(int j = 0 ; j < data.down_cloud->size() ;j++)
            inliers->indices.push_back(j);
        inlier_set.push_back(inliers);
        return inlier_set;
    }
    
    cv::Mat map2d = clOn2D(data.down_cloud, fx, fy, center_x, center_y);
    
    int w = map2d.cols;
    int h = map2d.rows;
    
//    std::vector<size_t> rand_idx;
//    GenRandSeq(rand_idx, num);
    srand(time(NULL));
    int sample_count = 0;
//    for( size_t i = 0 ; i < num ; i++ )
    while(true)
    {
        int cur_idx = rand()%num;
        PointT *pt_ptr = &data.down_cloud->at(cur_idx);

        float cur_w = 0.015 + rand()%70 / 100.0;
        float cur_h = 0.015 + rand()%70 / 100.0;
        
        cv::Rect cur_rect;
        int tl_x = (pt_ptr->x - cur_w) / pt_ptr->z * fx + center_x;
        int tl_y = (pt_ptr->y - cur_h) / pt_ptr->z * fy + center_y;

        int br_x = (pt_ptr->x + cur_w) / pt_ptr->z * fx + center_x;
        int br_y = (pt_ptr->y + cur_h) / pt_ptr->z * fy + center_y;

        if( tl_x < 0 )  tl_x = 0; 
        if( tl_y < 0 )  tl_y = 0;
        if( br_x >= w ) br_x = w-1;
        if( br_y >= h ) br_y = h-1;

        cur_rect.x = tl_x;
        cur_rect.y = tl_y;

        cur_rect.width =  br_x - tl_x;
        cur_rect.height = br_y - tl_y;
        
        pcl::PointIndices::Ptr cur_inlier = getSegIdx(map2d, cur_rect);
        
        if( cur_inlier->indices.size() >= min_num && cur_inlier->indices.size() <= max_num )
        {
            inlier_set.push_back(cur_inlier);
            sample_count++;
            if( sample_count >= max_rand_sample )
                break;
        }
    }
    
    return inlier_set;
}


void get_SHOT_PCSHOT_Code_ss(const MulInfoT &data, 
        const cv::Mat shot_dict, cv::flann::Index &shot_tree, int shot_len, int shotK, cv::Mat &shot_code, 
        const cv::Mat pcshot_dict, cv::flann::Index &pcshot_tree, int pcshot_len, int pcshotK, cv::Mat &pcshot_code, 
        MulInfoT &high_data, float rad, float ss)
{
    
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
    cv::Mat high_fea = cshot_cloud_ss(data.cloud, data.cloud_normals, data.down_lrf, down_cloud, rad, ss);
    
    cv::Mat temp_shot = cv::Mat::zeros(high_fea.rows, 352, CV_32FC1);
    high_fea.colRange(0, 352).copyTo(temp_shot);
    cv::Mat temp_pcshot = cv::Mat::zeros(high_fea.rows, 992, CV_32FC1);
    high_fea.colRange(352, 1344).copyTo(temp_pcshot);
    
    int num = down_cloud->size();
    
    std::vector<cv::Mat> shot_code_vec(num);  
    std::vector<cv::Mat> pcshot_code_vec(num);      
    for( int i = 0 ; i < num ; i++ )
    {
        cv::normalize(temp_shot.row(i),temp_shot.row(i));
        cv::normalize(temp_pcshot.row(i),temp_pcshot.row(i));
        
        shot_code_vec[i] = KNNEncoder(temp_shot.row(i), shot_tree, shot_len, shotK);
        pcshot_code_vec[i] = KNNEncoder(temp_pcshot.row(i), pcshot_tree, pcshot_len, pcshotK);
    }
    cv::vconcat(shot_code_vec, shot_code);
    cv::vconcat(pcshot_code_vec, pcshot_code);
    
    if( ss < 0 )
        high_data = data;
    else
    {
        pcl::PointCloud<NormalT>::Ptr down_normal(new pcl::PointCloud<NormalT>());
        high_data = convertPCD(down_cloud, down_normal);
    }
}


void getSHOTCode_ss(const MulInfoT &data, const cv::Mat dict, cv::flann::Index &fea_tree, cv::Mat &high_code, MulInfoT &high_data, int fea_len, int feaK, float rad, int type, float ss)
{
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
    cv::Mat high_fea;
    
    switch(type)
    {
        case 0:
            high_fea = shot_cloud_ss(data.cloud, data.cloud_normals, data.down_lrf, down_cloud, rad, ss);
            break;
        case 1:
            high_fea = cshot_cloud_ss(data.cloud, data.cloud_normals, data.down_lrf, down_cloud, rad, ss);
            break;
        case 2:
            //high_fea = usc_cloud_ss(data.cloud, data.cloud_normals, data.lrf, down_cloud, rad, ss);
            break;
        default:break;
    }
    
    int num = down_cloud->size();
    
    std::vector<cv::Mat> high_code_vec(num);
        
    for( int i = 0 ; i < num ; i++ )
        high_code_vec[i] = KNNEncoder(high_fea.row(i), fea_tree, fea_len, feaK);
        
    cv::vconcat(high_code_vec, high_code);
    
    if( ss < 0 )
        high_data = data;
    else
    {
        pcl::PointCloud<NormalT>::Ptr down_normal(new pcl::PointCloud<NormalT>());
        high_data = convertPCD(down_cloud, down_normal);
    }
}

cv::Mat getNormalCode(const MulInfoT &data, cv::flann::Index &fea_tree, int fea_len, int feaK)
{
    int num = data.normal.rows;
    std::vector<cv::Mat> high_code_vec(num);
    for( int i = 0 ; i < num ; i++ )
    {
        cv::Mat no = data.normal.row(i);
        if( no.at<float>(0, 2) > 0 )
            no = -no;
        else if( no.at<float>(0, 2) == 0 )
            return cv::Mat::zeros(1, fea_len, CV_32FC1);

        high_code_vec[i] = KNNEncoder(no, fea_tree, fea_len, feaK); 
    }   
    cv::Mat high_code;
    cv::vconcat(high_code_vec, high_code);
    return high_code;
}

void LoadSeedsNormal(std::string normal_seed_file, cv::flann::Index &fea_tree, cv::Mat &fea_seeds, int &feaK, float ratio)
{
    fea_seeds = read3d(normal_seed_file);
    for( int i = 0 ; i < fea_seeds.rows ; i++ )
        cv::normalize(fea_seeds.row(i), fea_seeds.row(i), 1.0, 0.0, cv::NORM_L2);
    
    cv::flann::LinearIndexParams indexParams;
    //cv::flann::KDTreeIndexParams indexParams(fea_seeds, indexParams);
#ifdef opencv_miniflann_build_h
    fea_tree = extFlannIndexBuild(fea_seeds, indexParams);
#else
    fea_tree.build(fea_seeds, indexParams);
#endif

    int len = fea_seeds.rows;

    feaK = len * ratio;

    
}

void LoadSeedsHigh(std::string high_seed_file, cv::flann::Index &fea_tree, cv::Mat &fea_seeds, int &feaK, float ratio)
{
    readMat(high_seed_file, fea_seeds);
    for( int i = 0 ; i < fea_seeds.rows ; i++ )
        cv::normalize(fea_seeds.row(i), fea_seeds.row(i), 1.0, 0.0, cv::NORM_L2);
    
    //cv::flann::LinearIndexParams indexParams;
    cv::flann::KDTreeIndexParams indexParams;
#ifdef opencv_miniflann_build_h
    fea_tree = extFlannIndexBuild(fea_seeds, indexParams);
#else
    fea_tree.build(fea_seeds, indexParams);
#endif

    int len = fea_seeds.rows;
    
    feaK = len * ratio;
}

cv::Mat getColorDiff(const pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud);
    
    cv::Mat diff = cv::Mat::zeros(cloud->size(), 1, CV_32FC1);
    for( size_t i = 0 ; i < cloud->size() ; i++ )
    {
        std::vector<int> ind;
        std::vector<float> dist;
        kdtree->radiusSearch(cloud->at(i), 0.01, ind, dist, cloud->size());
        uint32_t buf = *reinterpret_cast<int*>(&(cloud->at(i).rgba));
        cv::Mat rgb = cv::Mat::zeros(1, 3, CV_32FC1);
        rgb.at<float>(0, 0) = float((buf >> 16) & 0x0000ff);
        rgb.at<float>(0, 1) = float((buf >> 8)  & 0x0000ff);
        rgb.at<float>(0, 2) = float((buf)       & 0x0000ff);
        rgb = rgb / 255.0;
        //rgb = rgb + EPS;
        //cv::normalize(rgb, rgb, 1.0, 0.0, cv::NORM_L2);
        
        float max_dist = 0;
        for( size_t j = 1 ; j < ind.size() ; j++ )
        {
            uint32_t tmp = *reinterpret_cast<int*>(&(cloud->at(ind[j]).rgba));
            cv::Mat cur_rgb = cv::Mat::zeros(1, 3, CV_32FC1);
            
            cur_rgb.at<float>(0, 0) = float((tmp >> 16) & 0x0000ff);
            cur_rgb.at<float>(0, 1) = float((tmp >> 8) & 0x0000ff);
            cur_rgb.at<float>(0, 2) = float(tmp & 0x0000ff);
            cur_rgb = cur_rgb / 255.0;
            
            float cur_dist = cv::norm(cur_rgb-rgb, cv::NORM_L2);
            
            if( cur_dist > max_dist )
                max_dist = cur_dist;
        }
        diff.at<float>(i, 0) = max_dist;
    }
    return diff;
} 

/*
std::vector<cv::KeyPoint> extSIFTKeys(const cv::Mat &cur_gray, const std::vector<cv::SiftFeatureDetector*> &sift_det_vec)
{
    cv::Mat cur_response = cv::Mat::ones(cur_gray.size(), CV_32FC1)*-1;
    std::vector< std::pair<int, int> > active_idx;
    cv::Mat cur_idx = cv::Mat::ones(cur_gray.size(), CV_32SC1)*-1;
    
    std::vector<cv::KeyPoint> sift_keys;
    for( size_t i = 0 ; i < sift_det_vec.size() ; i++ )
    {
        std::vector<cv::KeyPoint> tmp_keys;
        sift_det_vec[i]->detect(cur_gray, tmp_keys);
        sift_keys.insert(sift_keys.end(), tmp_keys.begin(), tmp_keys.end());
    }  
    int count = 0;
    for(std::vector<cv::KeyPoint>::iterator it = sift_keys.begin() ; it < sift_keys.end() ; it++, count++ )
    {
        int row = round(it->pt.y);
        int col = round(it->pt.x);
        if( it->response > cur_response.at<float>(row, col) )
        {
            cur_response.at<float>(row, col) = it->response;
            if( cur_idx.at<int>(row, col) < 0 )
                active_idx.push_back(std::pair<int, int> (row, col));
            cur_idx.at<int>(row, col) = count;
        }
    }
    std::vector<cv::KeyPoint> final_keys(active_idx.size());
    count = 0;
    for(std::vector< std::pair<int, int> >::iterator it = active_idx.begin() ; it < active_idx.end() ; it++, count++ )
        final_keys[count] = sift_keys[cur_idx.at<int>(it->first, it->second)];

    return final_keys;
}
*/

