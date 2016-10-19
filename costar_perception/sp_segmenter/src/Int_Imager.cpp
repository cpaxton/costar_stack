#include "sp_segmenter/features.h"

IntImager::IntImager(float ap)
{
    ap_ratio = ap;
    img_h = 480 * ap_ratio;
    img_w = 640 * ap_ratio;
    
    fx = FOCAL_X;
    fy = FOCAL_Y;
    center_x = 320;
    center_y = 240;
    
    lab_layer = 4;
    layer_scale.resize(lab_layer+1);
    bin_to_layer.resize(lab_layer+1);
    total_bin = 0;
    for( int i = 1 ; i <= lab_layer ; i++ )
    {
        bin_to_layer[i] = total_bin;
        total_bin += i*i*i;
        layer_scale[i] = 1.00000001 / i;
    }
   
    fea_dim = -1;
    int_imgs.clear();
    target_flags.clear();
    target_flags.resize(1000);
    target_flags.assign(1000, false);
    
    cur_down_cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
}

void IntImager::setCameraParams(float fx_, float fy_, float center_x_, float center_y_)
{
    fx = fx_;
    fy = fy_;
    center_x = center_x_;
    center_y = center_y_;
}

std::vector<int> IntImager::getPoolIdx(cv::Mat lab)
{
    std::vector<int> idx(lab_layer);
    idx[0] = bin_to_layer[1];
    float *ptr = (float *)lab.data;
    for( int j = 2 ; j <= lab_layer ; j++ )
    {
        int idx_l = floor( *ptr / layer_scale[j] );
        int idx_a = floor( *(ptr+1) / layer_scale[j] );
        int idx_b = floor( *(ptr+2) / layer_scale[j] );

        if( idx_l >= j )  idx_l = j - 1;
        if( idx_a >= j )  idx_a = j - 1;
        if( idx_b >= j )  idx_b = j - 1;
        
        idx[j-1] = idx_l*j*j + idx_a*j + idx_b + bin_to_layer[j];
    }
    return idx;
}

void IntImager::setVoidFlags(std::vector<bool> &void_flags, std::vector<int> &valid_dims)
{
    void_ptr = &void_flags; 
    valid_idx = &valid_dims;
    
    fea_dim = void_ptr->size();
    valid_dim = valid_idx->size();
    int_imgs.resize(fea_dim);
    
    #pragma omp parallel for
    for( int i = 0 ; i < valid_dim ; i++ )
    {
        int idx = valid_idx->at(i);
        int_imgs[idx] = cv::Mat::zeros(img_h, img_w, CV_32FC1);
    }
}

void IntImager::Reset()
{
    int_map2d.release();
    int_uv.release();
    cur_down_cloud->clear();
    #pragma omp parallel for
    for( int i = 0 ; i < valid_dim ; i++ )
    {
        int idx = valid_idx->at(i);
        memset(int_imgs[idx].data, 0, img_h*img_w*sizeof(float));
    }
    dim_per_cell.clear();
}

void IntImager::ComputeInt(const MulInfoT &data, const std::vector<cv::Mat> &local_fea)
{
    double t1, t2;
    t1 = get_wall_time();
    Reset();    //clear up the integral map
    t2 = get_wall_time();
    std::cerr << "Integral-0: " << t2 - t1 << std::endl;
    
    dim_per_cell.clear();
    dim_per_cell.resize(local_fea.size());
    
    int total_dims = 0;
    std::vector<int> dim_s_idx(local_fea.size());
    for( size_t i = 0 ; i < local_fea.size() ; i++ )
    {
        dim_s_idx[i] = total_dims*total_bin;
        dim_per_cell[i] = local_fea[i].cols;
        total_dims += dim_per_cell[i];
    }
    int total_len = total_dims*total_bin;
    if( total_len != fea_dim )
    {
        std::cerr << "Model Dims != Extracted Dims!" << std::endl;
        std::cerr << total_len << " " << fea_dim << std::endl;
        exit(0);
    }
    
    t1 = get_wall_time();
    cur_down_cloud = data.down_cloud;
    int num = cur_down_cloud->size();
    SceneOn2D(cur_down_cloud, int_uv, int_map2d, ap_ratio, fx, fy, center_x, center_y);
    t2 = get_wall_time();
    std::cerr << "Integral-1: " << t2 - t1 << std::endl;
    
    t1 = get_wall_time();
    //Initialize Integral Map
    #pragma omp parallel for
    for( int i = 0 ; i < num ; i++ )
    {
        int r = int_uv.at<int>(i, 0);
        int c = int_uv.at<int>(i, 1);
        
        std::vector<int> idx = getPoolIdx(data.rgb.row(i));
        for( size_t j = 0 ; j < local_fea.size() ; j++ )
        {
            float *base_ptr = (float *)local_fea[j].row(i).data;
            for( std::vector<int>::iterator it = idx.begin() ; it < idx.end() ; it++ )
            {
                float *cur_ptr = base_ptr;
                int s_idx = (*it)*dim_per_cell[j] + dim_s_idx[j];
                int e_idx = s_idx + dim_per_cell[j];
                for( int k = s_idx ; k < e_idx ; k++, cur_ptr++ )
                {
                    if( void_ptr->at(k) == false )
                        int_imgs[k].at<float>(r, c) += *cur_ptr;    //sum_pooling
                }
            }
        }
    }
    t2 = get_wall_time();
    std::cerr << "Integral-2: " << t2 - t1 << std::endl;
    
    t1 = get_wall_time();
    #pragma omp parallel for
    for( int i = 0 ; i < valid_dim ; i++ )
    {
        int idx = valid_idx->at(i);
        
        float *ptr = (float *)int_imgs[idx].data;
        ptr++;  // skip the very first element
        for( int c = 1 ; c < img_w ; c++, ptr++ )
            *ptr = *(ptr-1) + *ptr;
        
        for( int r = 1 ; r < img_h ; r++ ){
            for( int c = 0 ; c < img_w ; c++, ptr++){
                if ( c == 0 )
                    *ptr = *(ptr-img_w) + *ptr;
                else
                    *ptr = *(ptr-img_w) + *(ptr-1) + *ptr - *(ptr-img_w-1);
            }
        }
    }
    t2 = get_wall_time();
    std::cerr << "Integral-3: " << t2 - t1 << std::endl;
    
}

void IntImager::Pooling(const cv::Rect &reg, sparseVec &pooled_fea)
{
    int tl_y = reg.tl().y;
    int tl_x = reg.tl().x;
    int br_y = reg.br().y;
    int br_x = reg.br().x;
    
    pooled_fea.clear();
    if( tl_y < 0 || tl_x < 0 || br_y >= img_h || br_x >= img_w )
        return;
    
    for( int i = 0 ; i < valid_dim ; i++ )
    {
        int idx = valid_idx->at(i);
        cv::Mat cur_int_image = int_imgs[idx];
        float val =   cur_int_image.at<float>(br_y, br_x) 
                    + cur_int_image.at<float>(tl_y, tl_x) 
                    - cur_int_image.at<float>(br_y, tl_x) 
                    - cur_int_image.at<float>(tl_y, br_x);
        
        if( val > 1e-6 )
        {
            feature_node tmp;
            tmp.index = idx + 1;
            tmp.value = val;
            pooled_fea.push_back(tmp);
        }
        if( val < 0  )
        {
            std::cerr << "Bug!!!" << std::endl;
            exit(0);
        }
    }
    int len = dim_per_cell[0];
    if( pooled_fea.empty() )
        return;
    //normalize per cell
    int cell_idx = floor((pooled_fea[0].index-1.0)/len);
    int s_idx = 0, e_idx = -1;
    int new_dim = pooled_fea.size();
    
    for(int i = 1 ; i < new_dim ; i++ )
    {
        int cur_idx = floor((pooled_fea[i].index-1.0)/len);
        if( cur_idx > cell_idx )
        {    
            e_idx = i;
            // normalize current cell
            float sum = 0;
            for( int k = s_idx ; k < e_idx ; k++ )
                sum += pooled_fea[k].value * pooled_fea[k].value;
            sum = sqrt(sum);
            for( int k = s_idx ; k < e_idx ; k++ )
                pooled_fea[k].value /=  sum;
            
            s_idx = i;
            e_idx = -1;
            cell_idx = cur_idx;
        }
        if( i == new_dim - 1 )
        {
            e_idx = new_dim;
            
            float sum = 0;
            for( int k = s_idx ; k < e_idx ; k++ )
                sum += pooled_fea[k].value * pooled_fea[k].value;
            sum = sqrt(sum);
            for( int k = s_idx ; k < e_idx ; k++ )
                pooled_fea[k].value /=  sum;
        }
    }
}


std::vector<Hypo> IntImager::DetectObjects(const model* obj_model, const std::pair<float, float> &cur_box, int step)
{   
    float bias = obj_model->bias;
    
    std::vector< std::pair<float, float> > all_boxes;
    all_boxes.push_back(cur_box);
    std::vector<cv::Rect> regs = UniformCubing(cur_down_cloud, int_map2d, all_boxes, step, ap_ratio, fx, fy, center_x, center_y);
    // make sure the integral images are already prepared    
    
    feature_node bias_term;
    bias_term.index = fea_dim +1;
    bias_term.value = bias;
    feature_node end_node;
    end_node.index = -1;
    end_node.value = 0;
    
    std::vector<Hypo> hypo_set;
    int regs_num = regs.size();
    #pragma omp parallel for
    for( int i = 0 ; i < regs_num ; i++ )
    {
        sparseVec tmp_fea;
        Pooling(regs[i], tmp_fea);
        if( tmp_fea.empty() == true )
            continue;
        
        tmp_fea.push_back(bias_term);
        tmp_fea.push_back(end_node);
        
        feature_node *cur_fea = new feature_node[tmp_fea.size()];
        std::copy(tmp_fea.begin(), tmp_fea.end(), cur_fea);
        double *dec_values = new double[obj_model->nr_class];
	double cur_label = predict_values(obj_model, cur_fea, dec_values);
        
        int pred_label = round(cur_label);
        if( target_flags[pred_label] == true )//&& dec_values[pred_label-1] > 0 )  //hit the target
        {
            Hypo cur_hypo;
            cur_hypo.box = regs[i];
            if(obj_model->nr_class > 2 )
                cur_hypo.score = dec_values[pred_label-1];
            else
                cur_hypo.score = dec_values[0] >= 0 ? dec_values[0] : -dec_values[0];
            cur_hypo.label = pred_label - 1 ;
            cur_hypo.ap_ratio = ap_ratio;
            
            #pragma omp critical
            {hypo_set.push_back(cur_hypo);}
        }
        
        delete[] cur_fea;
        delete[] dec_values;
    }
    
    std::vector<Hypo> final_hypo_set = hypo_set;
    //std::vector<Hypo> final_hypo_set = nonmax_suppress(hypo_set);
   
    return final_hypo_set;
}


void IntImager::setTarget(size_t label)
{
    if( label < target_flags.size() )
        target_flags[label]=true;
}

std::vector<Hypo> IntImager::nonmax_suppress(std::vector<Hypo> &hypo_set)
{
    std::sort(hypo_set.begin(), hypo_set.end(), Hypo_comp);
    
    int hypo_num = hypo_set.size();
    std::vector<Hypo> final_hypo_set;
    for(int i = 0 ; i < hypo_num ; i++ )
    {
        bool non_overlap = true;
        for(int j = i+1 ; j < hypo_num ; j++ ){
            if( overlap(hypo_set[i], hypo_set[j]) == true )
            {
                non_overlap = false;
                break;
            }
        }
        if( non_overlap )
        {
            Hypo new_cur_hypo;
            new_cur_hypo.ap_ratio = 1.0;
            new_cur_hypo.label = hypo_set[i].label;
            new_cur_hypo.score = hypo_set[i].score;
            new_cur_hypo.box.x = hypo_set[i].box.x / ap_ratio;
            new_cur_hypo.box.y = hypo_set[i].box.y / ap_ratio;
            new_cur_hypo.box.width = hypo_set[i].box.width / ap_ratio;
            new_cur_hypo.box.height = hypo_set[i].box.height / ap_ratio;
            
            final_hypo_set.push_back(new_cur_hypo);
        }
            
    }
    
    return final_hypo_set;
}


