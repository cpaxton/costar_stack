#include <opencv2/core/core.hpp>
#include "../include/features.h"

float this_fx = FOCAL_X;
float this_fy = FOCAL_Y;
float center_x = CENTER_X;
float center_y = CENTER_Y;

uchar color_label[11][3] = 
{ {0, 0, 0}, 
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
  {255, 128, 0},
  {255, 0, 128},
  {0, 128, 255},
  {128, 0, 255},
};    

struct ONE_PRED
{
    int label = -1;
    float score = -1000;
};

void batch_CvMatToFeatureNode(const std::vector<cv::Mat> &cv_set, const model* cur_model, std::vector<sparseVec> &sparse_set)
{
    feature_node bias_term;
    bias_term.index = cur_model->nr_feature;
    bias_term.value = cur_model->bias;
    feature_node end_node;
    end_node.index = -1;
    end_node.value = 0;
    
    int num = cv_set.size();
    sparse_set.clear();
    sparse_set.resize(num);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        CvMatToFeatureNode(cv_set[j], sparse_set[j]);
        sparse_set[j].push_back(bias_term);
        sparse_set[j].push_back(end_node);
    }
}

void extSPFea(const std::vector<pcl::PointCloud<PointT>::Ptr> segs, const MulInfoT &ori_cloud, const model* cur_model, 
                        Hier_Pooler &cshoter, const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, std::vector<sparseVec> &fea_set)
{
    int seg_num = segs.size();
    fea_set.clear();
    fea_set.resize(seg_num);   //release fea_set outside the function
    
    feature_node bias_term;
    bias_term.index = cur_model->nr_feature;
    bias_term.value = cur_model->bias;
    feature_node end_node;
    end_node.index = -1;
    end_node.value = 0;
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < seg_num ; j++ )
    {
        if( segs[j]->empty() == true )
            continue;
        
        MulInfoT cur_seg = convertPCD(ori_cloud.cloud, ori_cloud.cloud_normals);
        cur_seg.down_cloud = segs[j];//down_seg;
        PreCloud(cur_seg, -1, false);

        std::vector<cv::Mat> local_fea = cshoter.getHierFea(cur_seg, 0);
        cv::Mat final_temp = multiPool(pooler_set, cur_seg, local_fea);
        local_fea.clear();

        CvMatToFeatureNode(final_temp, fea_set[j]);
        fea_set[j].push_back(bias_term);
        fea_set[j].push_back(end_node);
    
    }
}

void extSPFea_Raw(const std::vector<pcl::PointCloud<PointT>::Ptr> segs, const MulInfoT &ori_cloud,
                        Hier_Pooler &cshoter, const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, std::vector< std::vector<cv::Mat> > &raw_set)
{
    int seg_num = segs.size();
    raw_set.clear();
    raw_set.resize(seg_num);   //release fea_set outside the function
    
    pcl::PointCloud<PointT>::Ptr all_segs_cloud(new pcl::PointCloud<PointT>());
    std::vector< std::pair<int, int> > bound_vec;
    for( int j = 0 ; j < seg_num ; j++ )
    {
        int s_idx = all_segs_cloud->size();
        all_segs_cloud->insert(all_segs_cloud->end(), segs[j]->begin(), segs[j]->end());
        int e_idx = all_segs_cloud->size();
        bound_vec.push_back(std::pair<int, int> (s_idx, e_idx) );
    }
    all_segs_cloud->height = 1;
    all_segs_cloud->width = all_segs_cloud->size();
    
    MulInfoT all_segs = convertPCD(ori_cloud.cloud, ori_cloud.cloud_normals);
    all_segs.down_cloud = all_segs_cloud;
    PreCloud(all_segs, -1, false);
    
    std::vector<cv::Mat> main_fea = cshoter.getHierFea(all_segs, 0);
    size_t temp = main_fea.size();
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < seg_num ; j++ )
    {
        if( segs[j]->empty() == true )
            continue;
        
        MulInfoT cur_seg = convertPCD(ori_cloud.cloud, ori_cloud.cloud_normals);
        cur_seg.down_cloud = segs[j];//down_seg;
        PreCloud(cur_seg, -1, false);

        std::vector<cv::Mat> local_fea(temp);
        for(size_t i = 0 ; i < temp ; i++ )
        {
            cv::Mat cur_fea = cv::Mat::zeros(bound_vec[j].second - bound_vec[j].first, main_fea[i].cols, CV_32FC1);
            main_fea[i].rowRange(bound_vec[j].first, bound_vec[j].second).copyTo(cur_fea);
            local_fea[i].push_back(cur_fea);
        }
        
        raw_set[j] = multiPool_raw(pooler_set, cur_seg, local_fea);
        
        local_fea.clear();
    }
}


std::vector<cv::Mat> combineRaw(const std::vector< std::vector<cv::Mat> > &raw_set, const IDXSET &idx_set, bool max_pool = true)
{
    std::vector<cv::Mat> fea_set(idx_set.size());
    
    int pool_num = -1, dim_per_pool = -1;
    for( size_t i = 0 ; i < raw_set.size() ; i++ )
    {
        if( raw_set[i].size() > 0 )
        {
            pool_num = raw_set[i].size();
            dim_per_pool = raw_set[i][0].cols;
            break;
        }
    }
    
    if( pool_num < 0 || dim_per_pool <= 0)
    {
        std::cerr << "Error in combineRaw()!" << std::endl;
        return fea_set;
    }
    
    for( size_t i = 0 ; i < idx_set.size() ; i++ ){
        std::vector<cv::Mat> cur_fea;
        for( std::vector<int>::const_iterator it = idx_set[i].begin() ; it < idx_set[i].end() ; it++ )
        {
            if( raw_set[*it].empty() == true )
                continue;
            if( cur_fea.empty() == true )
            {
                cur_fea.resize(pool_num);
                for( int j = 0 ; j < pool_num ; j++ )
                    cur_fea[j] = raw_set[*it][j].clone();
            }
            else
            {
                for( int j = 0 ; j < pool_num ; j++ )
                {
                    if( max_pool == false )
                        cur_fea[j] += raw_set[*it][j];
                    else 
                        MaxOP(cur_fea[j], raw_set[*it][j]);
                }
            }
        }
        cv::Mat final_fea;
        if( cur_fea.empty() == false )
        {
            for( size_t j = 0 ; j < pool_num ; j++ )
                cv::normalize(cur_fea[j], cur_fea[j], 1.0, 0.0, cv::NORM_L2);
            cv::hconcat(cur_fea, final_fea);
        }
        else
            final_fea = cv::Mat::zeros(1, pool_num*dim_per_pool, CV_32FC1);
        
        fea_set[i] = final_fea;
    }
    return fea_set;
}

std::vector<ONE_PRED> classifyFeas(const model* cur_model, const std::vector<sparseVec> &fea_set)
{
    int num = fea_set.size();
    std::vector<ONE_PRED> pred_label_set(num);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        if( fea_set[j].empty() == false )
        {
            feature_node *cur_fea = new feature_node[fea_set[j].size()];
            std::copy(fea_set[j].begin(), fea_set[j].end(), cur_fea);

            double *dec_values = new double[cur_model->nr_class];
            double cur_label = predict_values(cur_model, cur_fea, dec_values);

            int tmp_label = round(cur_label) - 1;
            pred_label_set[j].label = tmp_label;
            pred_label_set[j].score = dec_values[tmp_label];
            
            delete[] cur_fea;
            delete[] dec_values;
        }
    }

    return pred_label_set;
}

std::vector<ONE_PRED> hard_classify(const model* low_model, const model* med_model, const model* high_model, const std::vector<sparseVec> &fea_set, const std::vector<pcl::PointCloud<PointT>::Ptr> &segs)
{
    int num = fea_set.size();
    if( fea_set.size() != segs.size() )
    {
        std::cerr << "fea_set.size() != fea_size.size()" << std::endl;
        exit(0);
    }
    std::vector<ONE_PRED> pred_label_set(num);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        int cur_fea_len = fea_set[j].size();
        if( cur_fea_len > 0 )
        {
            feature_node *cur_fea = new feature_node[cur_fea_len];
            std::copy(fea_set[j].begin(), fea_set[j].end(), cur_fea);
            
            //std::cerr << cur_fea_len << " ";
            double *dec_values;
            double cur_label;
            if( segs[j]->size() <= 250 )
            {
                dec_values = new double[low_model->nr_class];
                cur_label = predict_values(low_model, cur_fea, dec_values);
            }
            else if( segs[j]->size() <= 500 )
            {
                dec_values = new double[med_model->nr_class];
                cur_label = predict_values(med_model, cur_fea, dec_values);
            }
            else
            {
                dec_values = new double[high_model->nr_class];
                cur_label = predict_values(high_model, cur_fea, dec_values);
            }
            
            int tmp_label = round(cur_label) - 1;
            pred_label_set[j].label = tmp_label;
            if( low_model->nr_class <= 2 )
                pred_label_set[j].score = tmp_label == 1 ? -dec_values[0] : dec_values[0];
            else
                pred_label_set[j].score = dec_values[tmp_label];
            
            //std::cerr << tmp_label << " " << pred_label_set[j].score << std::endl;
            
            delete[] cur_fea;
            delete[] dec_values;
        }
    }
    //std::cerr << std::endl;
    //std::cin.get();
    return pred_label_set;
}

std::vector<ONE_PRED> multi_classify(const model* multi_model, const std::vector<sparseVec> &fea_set, const std::vector<ONE_PRED> &init_labels)
{
    int num = fea_set.size();
    if( fea_set.size() != init_labels.size() )
    {
        std::cerr << "fea_set.size() != init_labels.size()" << std::endl;
        exit(0);
    }
    std::vector<ONE_PRED> pred_label_set(num);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for( int j = 0 ; j < num ; j++ )
    {
        int cur_fea_len = fea_set[j].size();
        if( cur_fea_len > 0 && init_labels[j].label > 0 )
        {
            feature_node *cur_fea = new feature_node[cur_fea_len];
            std::copy(fea_set[j].begin(), fea_set[j].end(), cur_fea);
            
            double *dec_values;
            double cur_label;
            dec_values = new double[multi_model->nr_class];
            cur_label = predict_values(multi_model, cur_fea, dec_values);
            
            int tmp_label = round(cur_label) - 1;
            pred_label_set[j].label = tmp_label + 1; /****************/
            if( multi_model->nr_class <= 2 )
                pred_label_set[j].score = tmp_label == 1 ? -dec_values[0] : dec_values[0];
            else
                pred_label_set[j].score = dec_values[tmp_label];
            
            //std::cerr << tmp_label << " " << pred_label_set[j].score << std::endl;
            
            delete[] cur_fea;
            delete[] dec_values;
        }
        else
        {
            pred_label_set[j].label = -1;
            pred_label_set[j].score = -1000;
        }
    }
    return pred_label_set;
}

std::vector<ONE_PRED> mappingLabels(const std::vector<ONE_PRED> &high_labels, const std::vector< std::vector<int> > &ori_idx, int num)
{
    std::vector<ONE_PRED> low_labels(num);
    if( high_labels.size() != ori_idx.size() )
    {
        std::cerr << "Labels and Idx size don't match!" << std::endl;
        exit(0);
    }
    
    int seg_num = ori_idx.size();
    for( int j = 0 ; j < seg_num ; j++ )
    {
        for( std::vector<int>::const_iterator it = ori_idx[j].begin() ; it < ori_idx[j].end() ; it++ ){
            int tmp_label = high_labels[j].label;
            int tmp_score = high_labels[j].score;
            if (tmp_label > 0 && tmp_score > low_labels[*it].score)//( tmp_score > low_labels[*it].score ) //( tmp_label > 0 && tmp_score > low_labels[*it].score )//( dec_values[tmp_label] > pred_score_set[*it])
            {
                low_labels[*it].label = tmp_label;
                low_labels[*it].score = tmp_score;
            }
        }
    }
    return low_labels;
}

//std::vector< std::vector<int> > addOneSeg(const std::multimap<uint32_t, uint32_t> &graph, int num, const std::vector< std::vector<int> > &ori_idxs)
//{
//    std::set< std::vector<int> > new_idxs_set;
//    std::vector< std::vector<int> > new_idxs;
//    
//    for( size_t i = 0 ; i < ori_idxs.size() ; i++ )
//    {
//        for(std::vector<int>::const_iterator it = ori_idxs[i].begin() ; it < ori_idxs[i].end() ; it++ )
//        {
//            std::multimap<uint32_t,uint32_t>::const_iterator adj_it;
//            for( adj_it = graph.equal_range(*it).first ; adj_it != graph.equal_range(*it).second ; ++adj_it )
//            {
//                int adj_label = adj_it->second;
//                std::vector<int> tmp_idx = ori_idxs[i];
//                tmp_idx.push_back(adj_label);
//                std::sort(tmp_idx.begin(), tmp_idx.end());
//                
//                std::set< std::vector<int> >::iterator set_it = new_idxs_set.find(tmp_idx);
//                if( set_it == new_idxs_set.end() )
//                {
//                    new_idxs_set.insert(tmp_idx);
//                    new_idxs.push_back(tmp_idx);
//                }
//            }
//        }
//        
//    }
//    return new_idxs;
//}
//
////std::vector< std::vector<int> > getSegPairIdx(const std::multimap<uint32_t, uint32_t> &graph, int num, const  std::vector<int>  &ori_idxs)
////{
////    std::vector< std::vector<bool> > adj_mat;
////    adj_mat.clear();
////    adj_mat.resize(num);
////    for( int i = 0 ; i < num ; i++ )
////        adj_mat[i].resize(num, 0);
////    
////    std::vector< std::vector<int> > new_idxs;
////    
////    for(std::vector<int>::const_iterator it = ori_idxs.begin() ; it < ori_idxs.end() ; it++ )
////    {
////        std::multimap<uint32_t,uint32_t>::const_iterator adj_it;
////        for( adj_it = graph.equal_range(*it).first ; adj_it != graph.equal_range(*it).second ; ++adj_it )
////        {
////            int adj_label = adj_it->second;
////            if( adj_mat[*it][adj_label] == false && adj_mat[adj_label][*it] == false )
////            {
////                std::vector<int> cur_idx;
////                std::cerr << *it << "---" << adj_label << std::endl;
////                cur_idx.push_back(*it);
////                cur_idx.push_back(adj_label);
////                new_idxs.push_back(cur_idx);
////            }
////            adj_mat[*it][adj_label] = true;
////            adj_mat[adj_label][*it] = true;
////        }
////    }
////    
////    return new_idxs;
////}
//
//std::vector< std::vector<int> > getSegPairIdx(const std::multimap<uint32_t, uint32_t> &graph, int num)
//{
//    std::vector< std::vector<bool> > adj_mat;
//    adj_mat.clear();
//    adj_mat.resize(num);
//    for( int i = 0 ; i < num ; i++ )
//        adj_mat[i].resize(num, 0);
//    
//    std::vector< std::vector<int> > seg_idx;
//    
//    std::multimap<uint32_t,uint32_t>::const_iterator label_itr;
//    for ( label_itr = graph.begin () ; label_itr != graph.end (); )
//    {
//        uint32_t first_label = label_itr->first;
//
//        std::multimap<uint32_t,uint32_t>::const_iterator adjacent_itr = graph.equal_range (first_label).first;
//        for ( ; adjacent_itr!=graph.equal_range (first_label).second; ++adjacent_itr)
//        {
//            if( adj_mat[first_label][adjacent_itr->second] == false && adj_mat[adjacent_itr->second][first_label] == false )
//            {
//                std::vector<int> cur_idx;
//                cur_idx.push_back(first_label);
//                cur_idx.push_back(adjacent_itr->second);
//                seg_idx.push_back(cur_idx);
//            }
//            adj_mat[first_label][adjacent_itr->second] = true;
//            adj_mat[adjacent_itr->second][first_label] = true;
//        }
//        label_itr = graph.upper_bound(first_label);
//    }
//    
//    return seg_idx;
//}
//
//std::vector<pcl::PointCloud<PointT>::Ptr> getSegsCloud(const std::vector<pcl::PointCloud<PointT>::Ptr> segs, const std::vector< std::vector<int> > &idx_set)
//{
//    std::vector<pcl::PointCloud<PointT>::Ptr> high_segs(idx_set.size());
//    for( size_t i = 0 ; i < idx_set.size() ; i++ )
//    {
//        pcl::PointCloud<PointT>::Ptr cur_seg(new pcl::PointCloud<PointT>());
//        
//        for(std::vector<int>::const_iterator it = idx_set[i].begin() ; it < idx_set[i].end() ; it++ )
//            cur_seg->insert(cur_seg->end(), segs[*it]->begin(), segs[*it]->end());
//        
//        cur_seg->height = 1;
//        cur_seg->width = cur_seg->size();    
//        
//        high_segs[i] = cur_seg;
//    }
//    
//    return high_segs;
//}

//*

std::map<std::string, int> model_name_map;

std::vector<std::string> readMesh(std::string mesh_path, std::vector<ModelT> &model_set)
{
    boost::filesystem::path p(mesh_path);
    std::vector< std::string > ret;
    find_files(p, ".obj", ret);
    
    std::vector< std::string > valid_names;
    for(size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-4);
        ModelT cur_model;
        pcl::PolygonMesh::Ptr model_mesh(new pcl::PolygonMesh()); 
        pcl::io::loadPolygonFile(mesh_path + ret[i], *model_mesh); 
        pcl::PointCloud<myPointXYZ>::Ptr model_cloud(new pcl::PointCloud<myPointXYZ>()); 
        pcl::fromPCLPointCloud2(model_mesh->cloud, *model_cloud);
        cur_model.model_mesh = model_mesh;
        cur_model.model_label = model_name;
        cur_model.model_cloud = model_cloud;
            
        model_set.push_back(cur_model);
        valid_names.push_back(model_name);
    }
    return valid_names;
}

std::vector<poseT> readGT(std::string pose_path, std::string file_id)
{
    std::vector<poseT> gt_poses;
    
    boost::filesystem::path p(pose_path);
    std::vector< std::string > ret;
    find_files(p, "_"+file_id+".csv", ret);
    for(size_t i = 0 ; i < ret.size() ; i++ )
    {
        std::string model_name = ret[i].substr(0, ret[i].size()-5-file_id.size());
        readCSV(pose_path + ret[i], model_name, gt_poses);
    }
    return gt_poses;
}

pcl::PointCloud<PointT>::Ptr genSeg(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map)
{
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene, *scene_xyz);
    
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (scene_xyz);
    
    std::vector<int> obj_labels(scene->size(), -1);
    std::vector<float> obj_dist(scene->size(), 1000);
    float T = 0.015;
    
    for(size_t k = 0 ; k < gt_poses.size() ; k++ )
    {
        std::stringstream kk;
        kk << k;

        int model_idx = -1;
        int obj_id = -1;
        for(size_t i = 0 ; i < model_set.size(); i++ ){
            if(model_set[i].model_label == gt_poses[k].model_name)
            {
                model_idx = i;
                obj_id = model_map[model_set[i].model_label];
                //std::cerr << model_set[i].model_label << " " << obj_id << std::endl;
                break;
            }
        }
        if( obj_id <= 0 )
        {
            std::cerr << "No Matching Model!" << std::endl;
            exit(0);
        }
        
        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[k].shift, gt_poses[k].rotation);
        
        for(pcl::PointCloud<myPointXYZ>::iterator it = buf_cloud->begin() ; it < buf_cloud->end() ; it++ )
        {
            std::vector<int> idx;
            std::vector<float> dist;
    
            tree.radiusSearch(*it, T, idx, dist, buf_cloud->size());
            for( size_t j = 0 ; j < idx.size() ; j++ )
            {
                if( obj_dist[idx[j]] > dist[j] )
                {
                    obj_labels[idx[j]] = obj_id;
                    obj_dist[idx[j]] = dist[j];
                } 
            }   
        }
    }
    
    pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < scene->size() ; i++ )
    {
        if( obj_labels[i] > 0 )
        {
            PointT new_pt = scene->at(i);
            new_pt.rgba = obj_labels[i];
            seg_cloud->push_back(new_pt);
        }
    }
    return seg_cloud;
}

pcl::PointCloud<PointT>::Ptr genSeg_all(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map)
{
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene, *scene_xyz);
    
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (scene_xyz);
    
    std::vector<int> obj_labels(scene->size(), -1);
    std::vector<float> obj_dist(scene->size(), 1000);
    float T = 0.015;
    
    for(size_t k = 0 ; k < gt_poses.size() ; k++ )
    {
        std::stringstream kk;
        kk << k;

        int model_idx = -1;
        int obj_id = -1;
        for(size_t i = 0 ; i < model_set.size(); i++ ){
            if(model_set[i].model_label == gt_poses[k].model_name)
            {
                model_idx = i;
                obj_id = model_map[model_set[i].model_label];
                //std::cerr << model_set[i].model_label << " " << obj_id << std::endl;
                break;
            }
        }
        if( obj_id <= 0 )
        {
            std::cerr << "No Matching Model!" << std::endl;
            exit(0);
        }
        
        pcl::PointCloud<myPointXYZ>::Ptr buf_cloud(new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*(model_set[model_idx].model_cloud), *buf_cloud, gt_poses[k].shift, gt_poses[k].rotation);
        
        for(pcl::PointCloud<myPointXYZ>::iterator it = buf_cloud->begin() ; it < buf_cloud->end() ; it++ )
        {
            std::vector<int> idx;
            std::vector<float> dist;
    
            tree.radiusSearch(*it, T, idx, dist, buf_cloud->size());
            for( size_t j = 0 ; j < idx.size() ; j++ )
            {
                if( obj_dist[idx[j]] > dist[j] )
                {
                    obj_labels[idx[j]] = obj_id;
                    obj_dist[idx[j]] = dist[j];
                } 
            }   
        }
    }
    
    pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < scene->size() ; i++ )
    {
        PointT new_pt = scene->at(i);
        if(obj_labels[i] > 0 )
            new_pt.rgba = obj_labels[i];
        else
            new_pt.rgba = 0;
        seg_cloud->push_back(new_pt);
    }
    return seg_cloud;
}

//std::pair<float, float> computePR(const pcl::PointCloud<PointT>::Ptr gt_cloud, const pcl::PointCloud<PointT>::Ptr label_cloud_0)
//{
//    if( gt_cloud->empty() == true || label_cloud_0->empty() == true )
//        return std::pair<float, float> (0.0, 0.0);
//    pcl::search::KdTree<PointT> tree;
//    tree.setInputCloud(gt_cloud);
//    
//    int corr_count = 0;
//    float T = 0.001;
//    float sqrT = T*T;
//    for( pcl::PointCloud<PointT>::const_iterator it = label_cloud_0->begin() ; it < label_cloud_0->end() ; it++ )
//    {
//        std::vector<int> indices (1);
//	std::vector<float> sqr_distances (1);
//        int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
//        if ( nres >= 1 && sqr_distances[0] <= sqrT )
//            corr_count++;
//    }
//    std::pair<float, float> tmp;
//    tmp.first = (corr_count + 0.0) / label_cloud_0->size();   //precision 
//    tmp.second = (corr_count + 0.0) / gt_cloud->size();       //recall
//    return tmp;
//}

std::vector<PR_ELEM> foregroundPR(const pcl::PointCloud<PointT>::Ptr gt_cloud, const pcl::PointCloud<PointT>::Ptr foreground_cloud, int model_num)
{
    std::vector<PR_ELEM> model_pr(model_num + 1);
    
    if( foreground_cloud->empty() == true )
    {
        for(size_t i = 0 ; i < model_pr.size() ; i++ )
        {
            model_pr[i].precision = 0;
            model_pr[i].recall = 0;
            model_pr[i].f_score = 0;
            model_pr[i].valid = true;
        }
        return model_pr;
    }
    
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(foreground_cloud);
    
    std::vector<int> corr_count(model_num + 1, 0);
    std::vector<int> count_in_gt(model_num + 1, 0);
    float T = 0.007;
    float sqrT = T*T;
    for( pcl::PointCloud<PointT>::const_iterator it = gt_cloud->begin() ; it < gt_cloud->end() ; it++ )
    {
        std::vector<int> indices (1);
	std::vector<float> sqr_distances (1);
        int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
        if ( nres >= 1 && sqr_distances[0] <= sqrT )
            corr_count[it->rgba]++;
        count_in_gt[it->rgba]++;
    }
    
    for( size_t i = 1 ; i < corr_count.size() ; i++ )
    {
        if( count_in_gt[i] <= 0 )
        {
            model_pr[i].valid = false;
            model_pr[i].recall = -1.0;
        }
        else
        {
            model_pr[i].valid = true;
            model_pr[i].recall = (corr_count[i] + 0.0) / count_in_gt[i];
        }
    }
    
    return model_pr;
}

std::vector<PR_ELEM> multiPR(const pcl::PointCloud<PointT>::Ptr gt_cloud, const pcl::PointCloud<PointT>::Ptr label_cloud, int model_num)
{
    std::vector<PR_ELEM> model_pr(model_num + 1);
    
    if( label_cloud->empty() == true )
    {
        for(size_t i = 0 ; i < model_pr.size() ; i++ )
        {
            model_pr[i].precision = 0;
            model_pr[i].recall = 0;
            model_pr[i].f_score = 0;
            model_pr[i].valid = true;
        }
        return model_pr;
    }
    
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud(label_cloud);
    
    std::vector<int> corr_count(model_num + 1, 0);
    std::vector<int> count_in_gt(model_num + 1, 0);
    float T = 0.007;
    float sqrT = T*T;
    for( pcl::PointCloud<PointT>::const_iterator it = gt_cloud->begin() ; it < gt_cloud->end() ; it++ )
    {
        std::vector<int> indices (1);
	std::vector<float> sqr_distances (1);
        int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
        if ( nres >= 1 && sqr_distances[0] <= sqrT && it->rgba == label_cloud->at(indices[0]).rgba )
            corr_count[it->rgba]++;
        count_in_gt[it->rgba]++;
    }
    
    for( size_t i = 1 ; i < corr_count.size() ; i++ )
    {
        if( count_in_gt[i] <= 0 )
        {
            model_pr[i].valid = false;
            model_pr[i].recall = -1.0;
        }
        else
        {
            model_pr[i].valid = true;
            model_pr[i].recall = (corr_count[i] + 0.0) / count_in_gt[i];
        }
    }
    
    return model_pr;
}


int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string workspace("../../data_pool/sp_hie_new/");
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string dict_path("BB_new_dict/");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--w", workspace);

    int c1 = 0, c2 = 99;
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);

    std::string model_path = workspace + "SVM_Model/";
    
    bool view_flag = true;
    if( pcl::console::find_switch(argc, argv, "-dv") == true )
        view_flag = false;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( view_flag == true )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
    }
    
    float radius = 0.03;
    float down_ss = 0.005;
    float ratio = 0;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    setObjID(model_name_map);
    std::vector<ModelT> model_set;
    std::vector<std::string> model_names = readMesh(mesh_path, model_set);
    
    std::cerr << "Loading SVM Model..." << std::endl;
    model* model_L0 = load_model((model_path + "SVM_L0.model").c_str());
    model* model_L1 = load_model((model_path + "SVM_L1.model").c_str());
    model* model_L2 = load_model((model_path + "SVM_L2.model").c_str());
    
//    model* multi_model = load_model((model_path + "SVM_multi2.model").c_str());
//    model* obj_model = load_model((model_path + "svm_model.model").c_str());
//    int fea_dim = obj_model->nr_feature - 1;    //subtract the bias term
//    std::cerr << "Feature Dimension: " << fea_dim << std::endl;  
    
    Hier_Pooler hie_producer_3(radius);
    hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    hie_producer_3.setRatio(ratio);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::vector< boost::shared_ptr<Pooler_L0> > pooler_set(FEA_LAYER+1);
    for( size_t i = 1 ; i < pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        pooler_set[i] = cur_pooler;
    }

//    std::pair<float, float> avg_pr_0(0.0, 0.0), avg_pr_1(0.0, 0.0), avg_pr_2(0.0, 0.0);
//    float avg_f_0 = 0, avg_f_1 = 0, avg_f_2 = 0;
    
    std::vector<PR_ELEM> avg_pr_0(model_names.size()+1);
    std::vector<PR_ELEM> avg_pr_1(model_names.size()+1);
    std::vector<PR_ELEM> avg_pr_2(model_names.size()+1);
    std::vector<int> obj_count(model_names.size()+1, 0);
    for( size_t i = 0 ; i < model_names.size()+1 ; i++ )
    {
        avg_pr_0[i].precision = 0;avg_pr_0[i].recall = 0;avg_pr_0[i].f_score = 0;avg_pr_0[i].valid = true;
        avg_pr_1[i].precision = 0;avg_pr_1[i].recall = 0;avg_pr_1[i].f_score = 0;avg_pr_1[i].valid = true;
        avg_pr_2[i].precision = 0;avg_pr_2[i].recall = 0;avg_pr_2[i].f_score = 0;avg_pr_2[i].valid = true;
    }
    int frame_count = 0;
    spExt sp_ext(down_ss);
    
    std::ifstream fp_scenes;
    fp_scenes.open((in_path + "test_scene.txt").c_str());
    std::vector<std::string> scene_names;
    while(true)
    {
        std::string temp;
        if( !(fp_scenes >> temp))
            break;
        if( temp.empty() || temp[0] != '#' )
        {
            scene_names.push_back(temp);
            std::cerr << temp << std::endl;
        }
    }
    fp_scenes.close();
    
    for( std::vector<std::string>::iterator scene_it = scene_names.begin() ; scene_it < scene_names.end() ; scene_it++ )
    {
        std::string out_path = workspace + "result_pool/" + *scene_it + "/" ;
        std::string cur_path(in_path + *scene_it + "/for_recog/");
        std::string gt_path(in_path + *scene_it + "/poses/");
        
        boost::filesystem::create_directories(out_path);
        
        for( int i = c1 ; i <= c2 ; i++ )
        {
            std::stringstream ss;
            ss << i;

            std::string filename(cur_path + *scene_it + "_" + ss.str() + ".pcd");
            std::string filename_n(cur_path + "normal_" + *scene_it + "_" + ss.str() + ".pcd");

            if( exists_test(filename) == false || exists_test(filename_n) == false )
            {
                pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
                continue;
            }
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(filename, *cloud);
            pcl::io::loadPCDFile(filename_n, *cloud_normal);

            std::cerr << filename << "-Loaded" << std::endl;
            sp_ext.clear();
            sp_ext.LoadPointCloud(cloud);
            pcl::PointCloud<PointT>::Ptr down_cloud = sp_ext.getCloud();
            
            MulInfoT cur_cloud = convertPCD(cloud, cloud_normal);
            cur_cloud.down_cloud = down_cloud;
            PreCloud(cur_cloud, -1, false);

            std::vector<poseT> cur_gt = readGT(gt_path, ss.str());
            if(cur_gt.empty() == true )
                continue;
            pcl::PointCloud<PointT>::Ptr gt_cloud = genSeg(down_cloud, model_set, cur_gt, model_name_map);
            
            std::vector<pcl::PointCloud<PointT>::Ptr> segs_low = sp_ext.getSPCloud(0);
//            for( int j = 0 ; j < segs_low.size() ; j++ )
//            {
//                std::stringstream ss;
//                ss << j;
//
//                viewer->addPointCloud(segs_low[j], "seg"+ss.str());
//                        
//                if( segs_low[j]->size() > 0 && segs_low[j]->size() <= 250 )
//                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "seg"+ss.str());
//                else if( segs_low[j]->size() > 250 && segs_low[j]->size() <= 500 )
//                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "seg"+ss.str());
//                else
//                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "seg"+ss.str());
//                //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "seg"+ss.str());
//            }
//            viewer->spin();
//            viewer->removeAllPointClouds();
//            continue;
            IDXSET segs_low_idx = sp_ext.getSPIdx(0);

    //        int count_0[40] = {0};
    //        for( size_t t = 0 ; t < segs_low.size() ; t++ )
    //        {
    //            if( segs_low[t]->size() >= 2000 )
    //                count_0[39]++;
    //            else
    //                count_0[segs_low[t]->size()/50]++;
    //        }
    //        for( int t = 0 ; t < 39 ; t++ )
    //            std::cerr << count_0[t] << " ";
    //        std::cerr << std::endl;

            int low_seg_num = segs_low.size();
    //        std::cerr << "Low Seg Num: "<< low_seg_num << std::endl;

            std::vector< std::vector<cv::Mat> > low_raw_feas;
            //extSPFea(segs_low, cur_cloud, model_L0, hie_producer_3, pooler_set, low_feas);
            extSPFea_Raw(segs_low, cur_cloud, hie_producer_3, pooler_set, low_raw_feas);
            std::vector<cv::Mat> low_cv_feas = combineRaw(low_raw_feas, segs_low_idx, false);
            std::vector<sparseVec> low_feas; 
            batch_CvMatToFeatureNode(low_cv_feas, model_L0, low_feas);
            low_cv_feas.clear();
   
            if( low_seg_num != low_feas.size() )
            {
                std::cerr << "Error: fea num != seg num" << std::endl;
                exit(0);
            }
            //std::vector<ONE_PRED> low_0 = classifyFeas(model_L0, low_feas);
            std::vector<ONE_PRED> low_0 = hard_classify(model_L0, model_L1, model_L2, low_feas, segs_low);
            
            pcl::PointCloud<PointT>::Ptr label_cloud_0(new pcl::PointCloud<PointT>());
            for( int j = 0 ; j < low_seg_num ; j++ )
            {
                int cur_label = low_0[j].label;
                if( cur_label > 0 )
                {
                    label_cloud_0->insert(label_cloud_0->begin(), segs_low[j]->begin(), segs_low[j]->end());
                    
                    if( view_flag == true )
                    {
                        std::stringstream ss;
                        ss << j;

                        viewer->addPointCloud(segs_low[j], "seg"+ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_label[cur_label][0], color_label[cur_label][1], color_label[cur_label][2], "seg"+ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "seg"+ss.str());
                    }
                }
            }
            if( view_flag == true )
            {
                viewer->addPointCloud(cloud, "cloud");
                viewer->spinOnce(1);
                viewer->saveScreenshot(out_path + "det_" + ss.str() + "_L0.png");
                viewer->removeAllPointClouds();
                
            }
            label_cloud_0->height = 1;
            label_cloud_0->width = label_cloud_0->size();    
            //std::pair<float, float> pr_0 = computePR(gt_cloud, label_cloud_0);
            std::vector< PR_ELEM > model_pr_0 = foregroundPR(gt_cloud, label_cloud_0, model_names.size());
            
            std::vector< std::vector<int> > in_seg;
            for( size_t j = 0 ; j < low_0.size() ; j++ )
                if( low_0[j].label > 0 )
                {
                    std::vector<int> tmp;
                    tmp.push_back(j);
                    in_seg.push_back(tmp);
                }
//            sp_ext.updateLevel(0, in_seg);

            IDXSET segs_med_idx = sp_ext.getSPIdx(1);
            std::vector<pcl::PointCloud<PointT>::Ptr> segs_med = sp_ext.getSPCloud(1);

            std::vector<cv::Mat> med_cv_feas = combineRaw(low_raw_feas, segs_med_idx, false);
            std::vector<sparseVec> med_feas; 
            batch_CvMatToFeatureNode(med_cv_feas, model_L0, med_feas);
            med_cv_feas.clear();

    //        int count_1[40] = {0};
    //        for( size_t t = 0 ; t < segs_med.size() ; t++ )
    //        {
    //            if( segs_med[t]->size() >= 2000 )
    //                count_1[39]++;
    //            else
    //                count_1[segs_med[t]->size()/50]++;
    //        }
    //        for( int t = 0 ; t < 39 ; t++ )
    //            std::cerr << count_1[t] << " ";
    //        std::cerr << std::endl;

            //std::vector<sparseVec> med_feas;
            //extSPFea(segs_med, cur_cloud, model_L1, hie_producer_3, pooler_set, med_feas);

            if( med_feas.size() != segs_med.size() )
            {
                std::cerr << "Error: med fea num != med seg num" << std::endl;
                exit(0);
            }

            //std::vector<ONE_PRED> med_labels = classifyFeas(model_L1, med_feas);
            std::vector<ONE_PRED> med_labels = hard_classify(model_L0, model_L1, model_L2, med_feas, segs_med);
            std::vector<ONE_PRED> low_1 = mappingLabels(med_labels, segs_med_idx, low_seg_num);
            
            pcl::PointCloud<PointT>::Ptr label_cloud_1(new pcl::PointCloud<PointT>());
            for( size_t j = 0 ; j < low_1.size() ; j++ )
            {
                int cur_label = low_1[j].label;
                if( cur_label > 0 )
                {
                    label_cloud_1->insert(label_cloud_1->begin(), segs_low[j]->begin(), segs_low[j]->end());
                    
                    if( view_flag == true )
                    {
                        std::stringstream ss;
                        ss << j;

                        viewer->addPointCloud(segs_low[j], "seg"+ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_label[cur_label][0], color_label[cur_label][1], color_label[cur_label][2], "seg"+ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "seg"+ss.str());
                    }
                }
            }
            if( view_flag == true )
            {
                viewer->addPointCloud(cloud, "cloud");
                viewer->spinOnce(1);
                viewer->saveScreenshot(out_path + "det_" + ss.str() + "_L1.png");
                viewer->removeAllPointClouds();
            }
            
            std::vector< PR_ELEM > model_pr_1 = foregroundPR(gt_cloud, label_cloud_1, model_names.size());
            
//            std::pair<float, float> pr_1 = computePR(gt_cloud, label_cloud_1);
//            std::cerr << "SP-1:\tP---" << pr_1.first << ",\tR---" << pr_1.second << ",\tF---" << 2*(pr_1.first*pr_1.second) / (pr_1.first+pr_1.second) << std::endl;
//            avg_pr_1.first += pr_1.first;
//            avg_pr_1.second += pr_1.second;
//            avg_f_1 += pr_1.first+pr_1.second == 0 ? 0 : 2*(pr_1.first*pr_1.second) / (pr_1.first+pr_1.second);

            in_seg.clear();
            for( size_t j = 0 ; j < med_labels.size() ; j++ )
                if( med_labels[j].label > 0 )
                    in_seg.push_back(segs_med_idx[j]);
//            sp_ext.updateLevel(1, in_seg);

            IDXSET segs_high_idx = sp_ext.getSPIdx(2);
            std::vector<pcl::PointCloud<PointT>::Ptr> segs_high = sp_ext.getSPCloud(2);

            std::vector<cv::Mat> high_cv_feas = combineRaw(low_raw_feas, segs_high_idx, false);
            std::vector<sparseVec> high_feas; 
            batch_CvMatToFeatureNode(high_cv_feas, model_L0, high_feas);
            high_cv_feas.clear();

    //        int count_2[40] = {0};
    //        for( size_t t = 0 ; t < segs_high.size() ; t++ )
    //        {
    //            if( segs_high[t]->size() >= 2000 )
    //                count_2[39]++;
    //            else
    //                count_2[segs_high[t]->size()/50]++;
    //        }
    //        for( int t = 0 ; t < 39 ; t++ )
    //            std::cerr << count_2[t] << " ";
    //        std::cerr << std::endl;

    //        std::vector<sparseVec> high_feas;
    //        extSPFea(segs_high, cur_cloud, model_L2, hie_producer_3, pooler_set, high_feas);

            //std::vector<ONE_PRED> high_labels = classifyFeas(model_L2, high_feas);
            std::vector<ONE_PRED> high_labels = hard_classify(model_L0, model_L1, model_L2, high_feas, segs_high);
            std::vector<ONE_PRED> low_2 = mappingLabels(high_labels, segs_high_idx, low_seg_num);

            if( segs_high.size() != high_feas.size() )
            {
                std::cerr << "Error: high fea num != high seg num" << std::endl;
                exit(0);
            }

            pcl::PointCloud<PointT>::Ptr label_cloud_2(new pcl::PointCloud<PointT>());
            for( size_t j = 0 ; j < low_2.size() ; j++ )
            {
                int cur_label = low_2[j].label;
                if( cur_label > 0 )
                {
                    label_cloud_2->insert(label_cloud_2->begin(), segs_low[j]->begin(), segs_low[j]->end());
                    if( view_flag == true )
                    {
                        std::stringstream ss;
                        ss << j;

                        viewer->addPointCloud(segs_low[j], "seg"+ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_label[cur_label][0], color_label[cur_label][1], color_label[cur_label][2], "seg"+ss.str());
                        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "seg"+ss.str());
                    }
                }
            }
            if( view_flag == true )
            {
                viewer->addPointCloud(cloud, "cloud");
                viewer->spinOnce(1);
                viewer->saveScreenshot(out_path + "det_" + ss.str() + "_L2.png");
                viewer->removeAllPointClouds();
            }
            std::vector< PR_ELEM > model_pr_2 = foregroundPR(gt_cloud, label_cloud_2, model_names.size());
            
//            std::pair<float, float> pr_2 = computePR(gt_cloud, label_cloud_2);
//            std::cerr << "SP-2:\tP---" << pr_2.first << ",\tR---" << pr_2.second << ",\tF---" << 2*(pr_2.first*pr_2.second) / (pr_2.first+pr_2.second) << std::endl;
//            avg_pr_2.first += pr_2.first;
//            avg_pr_2.second += pr_2.second;
//            avg_f_2 += pr_2.first+pr_2.second == 0 ? 0 : 2*(pr_2.first*pr_2.second) / (pr_2.first+pr_2.second);
            
            std::cerr << "Foreground Recall**********" << std::endl;
            for( size_t j = 1 ; j < model_names.size() + 1 ; j++ )
            {
                std::cerr << std::setprecision(4) << model_pr_0[j].recall << "\t" << model_pr_1[j].recall << "\t" << model_pr_2[j].recall << "\t" << std::endl;
                if( model_pr_0[j].valid )
                {
                    avg_pr_0[j].recall += model_pr_0[j].recall;
                    avg_pr_1[j].recall += model_pr_1[j].recall;
                    avg_pr_2[j].recall += model_pr_2[j].recall;
                    obj_count[j]++;
                }
            }
            std::cerr << "***************************" << std::endl;    
            
            frame_count++;
        }
    }
    
    std::cerr << "Total Frames: " << frame_count << std::endl;
    
//    std::cerr << "Average-0:\tP---" << avg_pr_0.first/frame_count << ",\tR---" << avg_pr_0.second/frame_count << ",\tF---" << avg_f_0/frame_count << std::endl;
//    std::cerr << "Average-1:\tP---" << avg_pr_1.first/frame_count << ",\tR---" << avg_pr_1.second/frame_count << ",\tF---" << avg_f_1/frame_count << std::endl;
//    std::cerr << "Average-2:\tP---" << avg_pr_2.first/frame_count << ",\tR---" << avg_pr_2.second/frame_count << ",\tF---" << avg_f_2/frame_count << std::endl;
    
    std::ofstream fp;
    fp.open((workspace + "result_pool/pr_score.txt").c_str());
    
    for( size_t j = 1 ; j < model_names.size() + 1 ; j++ )
    {
        std::cerr << obj_count[j] << std::endl;
        std::cerr << std::setprecision(4) << avg_pr_0[j].recall / obj_count[j] << "\t" << avg_pr_1[j].recall / obj_count[j] << "\t" << avg_pr_2[j].recall / obj_count[j] << "\t" << std::endl;
        fp << obj_count[j] << std::endl;
        fp << std::setprecision(4) << avg_pr_0[j].recall / obj_count[j] << " " << avg_pr_1[j].recall / obj_count[j] << " " << avg_pr_2[j].recall / obj_count[j] << " " << std::endl;
    }
//    fp << avg_pr_0.first/frame_count << " " << avg_pr_0.second/frame_count << " " << avg_f_0/frame_count << std::endl;
//    fp << avg_pr_1.first/frame_count << " " << avg_pr_1.second/frame_count << " " << avg_f_1/frame_count << std::endl;
//    fp << avg_pr_2.first/frame_count << " " << avg_pr_2.second/frame_count << " " << avg_f_2/frame_count << std::endl;
//    fp << std::endl;
    fp.close();
    
    free_and_destroy_model(&model_L0);
    free_and_destroy_model(&model_L1);
    free_and_destroy_model(&model_L2);
    return 1;
} 
//*/
/*
int main(int argc, char** argv)
{
//    std::set< std::vector<int> > myset;
//    std::vector<int> aa, bb, cc;
//    aa.push_back(1);
//    bb.push_back(1);
//    bb.push_back(2);
//    cc.push_back(3);
//    
//    myset.insert(aa);
//    myset.insert(bb);
//    std::set< std::vector<int> >::iterator a_it = myset.find(aa);
//    std::set< std::vector<int> >::iterator b_it = myset.find(bb);
//    std::set< std::vector<int> >::iterator c_it = myset.find(cc);
//    
//    std::cerr << (a_it == myset.end()) << " " << (b_it == myset.end()) << " " << (c_it == myset.end()) << std::endl;
//    for( std::vector<int>::const_iterator it = a_it->begin() ; it < a_it->end() ; it++ )
//        std::cerr << *it << " ";
//    std::cerr << std::endl;
//    
//    for( std::vector<int>::const_iterator it = b_it->begin() ; it < b_it->end() ; it++ )
//        std::cerr << *it << " ";
//    std::cerr << std::endl;
//    return 0;
    
    int c1 = 0, c2 = -1;
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string workspace("../../data_pool/ht10_3x3/");
    std::string scene_name("4drills_simple");
    std::string dict_path("BB_new_dict/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--w", workspace);
    pcl::console::parse_argument(argc, argv, "--sn", scene_name);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);

    std::string model_path = workspace + "SVM_Model/";
    std::string out_path =  workspace + "result_pool/" + scene_name + "/" ;
    std::string cur_path(in_path + scene_name + "/for_recog/");
    
//    if( exists_dir(out_path) == false )
    boost::filesystem::create_directories(out_path+"L0/");
    boost::filesystem::create_directories(out_path+"L1/");
    boost::filesystem::create_directories(out_path+"L2/");
    
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    float radius = 0.03;
    float down_ss = 0.005;
    float ratio = 0;
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    std::cerr << "Loading SVM Model..." << std::endl;
    model* obj_model = load_model((model_path + "svm_model.model").c_str());
    int fea_dim = obj_model->nr_feature - 1;    //subtract the bias term
    std::cerr << "Feature Dimension: " << fea_dim << std::endl;  
    
    Hier_Pooler hie_producer_3(radius);
    hie_producer_3.LoadDict_L0(dict_path, "200", "200");
    hie_producer_3.setRatio(ratio);
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::vector< boost::shared_ptr<Pooler_L0> > pooler_set(FEA_LAYER+1);
    for( size_t i = 1 ; i < pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        pooler_set[i] = cur_pooler;
    }
      
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        
        viewer->removeAllPointClouds();
        std::stringstream ss;
        ss << i;
        
        std::string filename(cur_path + scene_name + "_" + ss.str() + ".pcd");
        std::string filename_n(cur_path + "normal_" + scene_name + "_" + ss.str() + ".pcd");
        if( exists_test(filename) == false || exists_test(filename_n) == false )
        {
            pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
            continue;
        }
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(filename, *cloud);
        pcl::io::loadPCDFile(filename_n, *cloud_normal);
        
        std::cerr << filename << "-Loaded" << std::endl;
        
        std::vector<pcl::PointCloud<PointT>::Ptr> segs_low;
        std::multimap<uint32_t, uint32_t> graph;
    
        if( down_ss > 0 )
        {
            pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
            pcl::VoxelGrid<PointT> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(down_ss, down_ss, down_ss);
            sor.filter(*down_cloud);
            
            SPCloud(down_cloud, segs_low, graph);
        }
        else
            SPCloud(cloud, segs_low, graph);
        
        int low_seg_num = segs_low.size();
        std::cerr << "Low Seg Num: "<< low_seg_num << std::endl;
        
        MulInfoT cur_cloud = convertPCD(cloud, cloud_normal);
        std::vector<sparseVec> low_feas;
        extSPFea(segs_low, cur_cloud, obj_model, hie_producer_3, pooler_set, low_feas);
        std::vector<ONE_PRED> low_0 = classifyFeas(obj_model, low_feas);
        
        for( int j = 0 ; j < low_seg_num ; j++ )
        {
            int cur_label = low_0[j].label;
            if( cur_label > 0 )
            {
                std::stringstream ss;
                ss << j;

//                float r = (std::rand()%255)/255.0;
//                float g = (std::rand()%255)/255.0;
//                float b = (std::rand()%255)/255.0;
                
                viewer->addPointCloud(segs_low[j], "seg"+ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_label[cur_label][0], color_label[cur_label][1], color_label[cur_label][2], "seg"+ss.str());
//                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "seg"+ss.str());
            }
        }
        viewer->spinOnce(1);
        viewer->saveScreenshot(out_path + "L0/det_" + ss.str() + ".png");
        viewer->removeAllPointClouds();
        
        std::vector< std::vector<int> > in_seg;
        for( size_t j = 0 ; j < low_0.size() ; j++ )
            if( low_0[j].label > 0 )
            {
                std::vector<int> tmp;
                tmp.push_back(j);
                in_seg.push_back(tmp);
            }
        std::cerr << "Inlier Low Seg Num: " << in_seg.size() << std::endl;
        
        std::vector< std::vector<int> > segs_med_idx = addOneSeg(graph, segs_low.size(), in_seg);
        int med_seg_num = segs_med_idx.size();
        std::cerr << "Med Seg Num: "<< med_seg_num << std::endl;
        
        std::vector<pcl::PointCloud<PointT>::Ptr> segs_med = getSegsCloud(segs_low, segs_med_idx);
        
        std::vector<sparseVec> med_feas;
        extSPFea(segs_med, cur_cloud, obj_model, hie_producer_3, pooler_set, med_feas);
        
        std::vector<ONE_PRED> med_labels = classifyFeas(obj_model, med_feas);
        std::vector<ONE_PRED> low_1 = mappingLabels(med_labels, segs_med_idx, low_seg_num);
        
        for( int j = 0 ; j < low_1.size() ; j++ )
        {
            int cur_label = low_1[j].label;
            if( cur_label > 0 )
            {
                std::stringstream ss;
                ss << j;

                float r = (std::rand()%255)/255.0;
                float g = (std::rand()%255)/255.0;
                float b = (std::rand()%255)/255.0;
                
                viewer->addPointCloud(segs_low[j], "seg"+ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_label[cur_label][0], color_label[cur_label][1], color_label[cur_label][2], "seg"+ss.str());
//                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "seg"+ss.str());
            }
        }
        viewer->spinOnce(1);
        viewer->saveScreenshot(out_path + "L1/det_" + ss.str() + ".png");
        viewer->removeAllPointClouds();
        
        in_seg.clear();
        for( size_t j = 0 ; j < med_labels.size() ; j++ )
            if( med_labels[j].label > 0 )
                in_seg.push_back(segs_med_idx[j]);
        std::cerr << "Inlier Low Seg Num: " << in_seg.size() << std::endl;
        
        std::vector< std::vector<int> > segs_high_idx = addOneSeg(graph, segs_low.size(), in_seg);
        int high_seg_num = segs_high_idx.size();
        std::cerr << "High Seg Num: "<< high_seg_num << std::endl;
        
        std::vector<pcl::PointCloud<PointT>::Ptr> segs_high = getSegsCloud(segs_low, segs_high_idx);
        
        std::vector<sparseVec> high_feas;
        extSPFea(segs_high, cur_cloud, obj_model, hie_producer_3, pooler_set, high_feas);
        
        std::vector<ONE_PRED> high_labels = classifyFeas(obj_model, high_feas);
        std::vector<ONE_PRED> low_2 = mappingLabels(high_labels, segs_high_idx, low_seg_num);
        
        for( int j = 0 ; j < low_2.size() ; j++ )
        {
            int cur_label = low_2[j].label;
            if( cur_label > 0 )
            {
                std::stringstream ss;
                ss << j;

                float r = (std::rand()%255)/255.0;
                float g = (std::rand()%255)/255.0;
                float b = (std::rand()%255)/255.0;
                
                viewer->addPointCloud(segs_low[j], "seg"+ss.str());
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color_label[cur_label][0], color_label[cur_label][1], color_label[cur_label][2], "seg"+ss.str());
//                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, "seg"+ss.str());
            }
        }
        viewer->spinOnce(1);
        viewer->saveScreenshot(out_path + "L2/det_" + ss.str() + ".png");
        
//        viewer->spinOnce(1);
//        viewer->saveScreenshot(out_path + "det_" + ss.str() + ".png");
//        t2 = get_wall_time();
//        std::cerr << t2 - t1 << std::endl;
    }
    
    free_and_destroy_model(&obj_model);
    return 1;
} 
//*/
