#include <opencv2/core/core.hpp>
#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

#define MAX_POOLER_NUM 50
#define MAX_FEA_NUM 10

//float this_fx = 539.6096276855468;
//float this_fy = 539.6096276855468;
//float center_x = 319.5;
//float center_y = 239.5;

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

//void colorHypos(const std::vector<Hypo> &hypo_set, cv::Mat &rgb_frame);
//void colorConfMap(const std::vector<Hypo> &hypo_set, cv::Mat &rgb_frame, bool binary = false);
//pcl::PointCloud<PointT>::Ptr labelCloud(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<Hypo> &hypo_set);
pcl::PointCloud<PointT>::Ptr labelCloud_C(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<Hypo> &hypo_set, cv::Mat &label_frame, cv::Mat &conf_frame);
void calibrate(std::vector<Hypo> &hypo_set);
pcl::PointCloud<PointT>::Ptr removePlane(const pcl::PointCloud<PointT>::Ptr scene);

int main(int argc, char** argv)
{
    float radius = 0.03;
    float down_ss = 0.005;
    float ratio = 0;
    float ap_ratio = 0.5;
    
    int c1 = 0, c2 = -1;
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string workspace("../data_pool/ln_3x3_daux/");
    std::string scene_name("ln_new");
    std::string dict_path("BB_new_dict/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--w", workspace);
    pcl::console::parse_argument(argc, argv, "--sn", scene_name);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    pcl::console::parse_argument(argc, argv, "--ap", ap_ratio);
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    
    std::string model_path = workspace + "SVM_Model/";
    std::string out_path =  workspace + "result_pool/" + scene_name + "/" ;
    std::string cur_path(in_path + scene_name + "/");
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    //viewer->initCameraParameters();
    //viewer->addCoordinateSystem(0.1);
    //viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    Hier_Pooler fea_producer(radius);
    fea_producer.LoadDict_L0(dict_path, "200", "200");
    fea_producer.setRatio(ratio);
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;

//////////////////////////////////////////////////////////////////////////////////////////////////////
    model* cur_model = load_model((model_path + "svm_model.model").c_str());
    std::pair<float, float> cur_box = readBoxFile( model_path + "model.box");
    std::cerr << "Training Box: " << cur_box.first << "-" << cur_box.second << std::endl;
    
    int fea_dim = cur_model->nr_feature - 1;    //subtract the bias term
    std::cerr << "Feature Dimension: " << fea_dim << std::endl;
    
    size_t void_count = 0;
    std::vector<bool> void_flags(fea_dim);
    void_flags.assign(fea_dim, false);
    std::vector<int> valid_dims;
    for( int i = 0 ; i < fea_dim ; i++ )
    {
        bool cur_flag = true;
        if( cur_model->nr_class > 2 )
        {
            for( int j = 0 ; j < cur_model->nr_class ; j++ )
            {
                if( cur_model->w[i*cur_model->nr_class+j] != 0 )
                    cur_flag = false;
            }
        }
        else
        {
            if( cur_model->w[i] != 0 )
                cur_flag = false;
        }
        if( cur_flag )
        {
            void_count++;
            void_flags[i] = true;
        }
        else
            valid_dims.push_back(i);
    }
    std::cerr << "Void Dim Number: " << void_count << std::endl;
    
//////////////////////////////////////////////////////////////////////////////////////////////////////
    
    double avg_t1 = 0, avg_t2 = 0, avg_t3 = 0;
    double std_t1 = 0, std_t2 = 0, std_t3 = 0;
    int i_count = 0;
    
    IntImager inter(ap_ratio);
    inter.setVoidFlags(void_flags, valid_dims);
    //inter.setCameraParams(539.6096276855468, 539.6096276855468, 319.5, 239.5);
    inter.setCameraParams(this_fx, this_fy, center_x, center_y);
    for( int j = 2 ; j <= cur_model->nr_class ; j++ )
        inter.setTarget(j);
    //inter.setTarget(2);
    //inter.setTarget(3);
    
    //std::vector< std::vector<Hypo> > gt_set, hypo_set;
    //int frame_num = readGround(in_path + scene_name + "/" + scene_name + "_gt.txt", gt_set);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        
        std::stringstream ss;
        ss << i;
        
        std::string filename(cur_path + scene_name + "_" + ss.str() + ".pcd");
        if( exists_test(filename) == false )
            break;
        i_count++;
        std::string filename_n(cur_path + "normal_" + scene_name + "_" + ss.str() + ".pcd");
        
        std::cerr << "Loading-" << filename << std::endl;
            
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *full_cloud);
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        std::vector<int> idx_ff;
        pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);
        
        pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(filename_n, *cloud_normal);
        
        /*
        cloud = removePlane(cloud);
        pcl::NormalEstimation<PointT, NormalT> ne;
        ne.setInputCloud (cloud);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        ne.setSearchMethod (tree);
        
        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        ne.compute (*cloud_normal);
        /******************************************************************************/
        
///////////////////////////////////////////////////////////////////////////////////////////       
        t1 = get_wall_time();
        MulInfoT cur_scene = convertPCD(cloud, cloud_normal);
        //PreCloud(cur_scene, down_ss, true);
        if( PreCloud(cur_scene, down_ss, true) == false)
        {
            std::cerr << "Filtered Data Still Failed!" << std::endl;
            exit(0);
        }
        t2 = get_wall_time();
        //std::cerr << "Preprocess Time-" << t2 - t1 << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////       

///////////////////////////////////////////////////////////////////////////////////////////               
        t1 = get_wall_time();
        std::vector<cv::Mat> local_fea = fea_producer.getHierFea(cur_scene, 0);
        t2 = get_wall_time();
        avg_t1 += t2 - t1;
        std_t1 += (t2 - t1)*(t2 - t1);
        //std::cerr << "CSHOT Encoding-" << t2 - t1 << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////       

///////////////////////////////////////////////////////////////////////////////////////////       
        t1 = get_wall_time();
        inter.ComputeInt(cur_scene, local_fea);
        t2 = get_wall_time();
        avg_t2 += t2 - t1;
        std_t2 += (t2 - t1)*(t2 - t1);
        //std::cerr << "Integral Image Time-" << t2 - t1 << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////       

///////////////////////////////////////////////////////////////////////////////////////////               
        t1 = get_wall_time();
        std::vector<Hypo> hypos = inter.DetectObjects(cur_model, cur_box, 1);
        //hypo_set.push_back(hypos);
        t2 = get_wall_time();
        avg_t3 += t2 - t1;
        std_t3 += (t2 - t1)*(t2 - t1);
        //std::cerr << "Detection Time-" << t2 - t1 << std::endl;
///////////////////////////////////////////////////////////////////////////////////////////       

        /*
        cv::Mat rgb_frame = getImage(cloud, 1.0, this_fx, this_fy, center_x, center_y);
        cv::Mat ori_frame = rgb_frame.clone();
        cv::Mat mag_frame = rgb_frame.clone();
        //colorHypos(hypos, rgb_frame);
        colorConfMap(hypos, mag_frame);
        colorConfMap(hypos, rgb_frame, true);
        
        cv::Mat final_map;
        cv::hconcat(ori_frame, rgb_frame, final_map);
        cv::hconcat(final_map, mag_frame, final_map);
        cv::imwrite(out_path + "det_" + ss.str() + ".png", final_map);
        */
        
        //pcl::PointCloud<PointT>::Ptr label_cloud = labelCloud(cloud, hypos);
        //*
        
        //calibrate(hypos);
        //cv::Mat ori_frame = getImage(cloud, 1.0, this_fx, this_fy, center_x, center_y);
        cv::Mat ori_frame = getImage(cloud, 1.0, this_fx, this_fy, center_x, center_y);
        cv::Mat label_frame, conf_map;
        pcl::PointCloud<PointT>::Ptr label_cloud = labelCloud_C(cloud, hypos, label_frame, conf_map);
        cv::Mat final_map;
        
        cv::hconcat(ori_frame, label_frame, final_map);
        cv::hconcat(final_map, conf_map, final_map);
        cv::imwrite(out_path + "det_" + ss.str() + ".png", final_map);
        
        //pcl::io::savePCDFile(out_path + "seg_" + ss.str() + ".pcd", *label_cloud, true);
        //pcl::io::savePCDFile(out_path + "ori_" + ss.str() + ".pcd", *full_cloud, true);
        //*/
        //viewer->removeAllPointClouds();
        //viewer->addPointCloud(label_cloud, "labels");
        //viewer->spin();
        
    }
    
    avg_t1 /= i_count;
    avg_t2 /= i_count;
    avg_t3 /= i_count;
    std_t1 = sqrt(std_t1/i_count - avg_t1*avg_t1);
    std_t2 = sqrt(std_t2/i_count - avg_t2*avg_t2);
    std_t3 = sqrt(std_t3/i_count - avg_t3*avg_t3);
    
    std::cerr << "CSHOT-" << avg_t1 << " +- " << std_t1 << ",\tIntegral-" << avg_t2 << " +- " << std_t2 << ",\tSearching-" << avg_t3 << " +- " << std_t3 << std::endl;
    //saveGround(out_path + "loc.txt", hypo_set);
    //std::vector< std::pair<float, float> > pr_set = PRScore(hypo_set, gt_set, cur_model->nr_class);
    
    free_and_destroy_model(&cur_model);
    return 1;
}
//*/


pcl::PointCloud<PointT>::Ptr labelCloud_C(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<Hypo> &hypo_set, cv::Mat &label_frame, cv::Mat &conf_frame)
{
    size_t max_class = 11;
    
    pcl::PointCloud<PointT>::Ptr label_cloud(new pcl::PointCloud<PointT>());
    //pcl::copyPointCloud(*cloud, *label_cloud);
    label_frame = cv::Mat::zeros(480, 640, CV_8UC3);
    conf_frame = cv::Mat::zeros(480, 640, CV_8UC3);
    
    if( hypo_set.empty() )
    {
        label_cloud->push_back(PointT ());
        return label_cloud;
    }
    
    cv::Mat uv, map2d;
    SceneOn2D(cloud, uv, map2d, 1.0, this_fx, this_fy, center_x, center_y);
    
    std::vector< std::vector<float> > conf_vec(cloud->size());
    std::vector< std::vector<int> > conf_count(cloud->size());
    for( size_t i = 0 ; i < hypo_set.size() ; i++ )
    {
        if ( hypo_set[i].score <= 0 )
            continue;
        cv::Rect reg = hypo_set[i].box;
        float cur_ratio = hypo_set[i].ap_ratio;
        int r_s = reg.tl().y / cur_ratio;
        int r_e = reg.br().y / cur_ratio;
        int c_s = reg.tl().x / cur_ratio;
        int c_e = reg.br().x / cur_ratio;
        
        for(int r = r_s ; r <= r_e ; r++ ){
            for(int c = c_s ; c <= c_e ; c++ ){
                int idx = map2d.at<int>(r, c);
                if( idx >= 0 )
                {
                    if( conf_vec[idx].empty() == true )
                        conf_vec[idx].resize(max_class, 0);
                    if( conf_count[idx].empty() == true )
                        conf_count[idx].resize(max_class, 0);
                    //std::cerr << idx << " " << hypo_set[i].label << std::endl;
                    conf_vec[idx][hypo_set[i].label] += hypo_set[i].score;
                    conf_count[idx][hypo_set[i].label]++;
                }
            }
        }
    }
    
    for( size_t i = 0 ; i < cloud->size() ; i++ )
    {
        int y = uv.at<int>(i, 0);
        int x = uv.at<int>(i, 1);
        
        PointT tmp = cloud->at(i);
        
        int max_idx = -1, max_count = -1;
        float max_conf = -1;
        //std::cerr << i << std::endl;
        for( size_t j = 0 ; j < max_class ; j++ )
        {
            if( conf_count[i].empty() == true || conf_count[i][j] <= 0 )
                continue;
                //std::cerr << conf_vec[i][j]<< " " << conf_count[i][j] << std::endl;
            if( max_conf < conf_vec[i][j] )
            {
                max_conf = conf_vec[i][j];
                max_count = conf_count[i][j];
                max_idx = j;
            }
        }
        if( max_idx < 0 )
            continue;
        //std::cerr << max_idx << " " << max_conf << std::endl;
        
        max_conf /= max_count;
        
        tmp.rgba = max_idx;
        label_cloud->push_back(tmp);
        label_frame.at<uchar>(y, x*3+0) = color_label[max_idx][2];
        label_frame.at<uchar>(y, x*3+1) = color_label[max_idx][1];
        label_frame.at<uchar>(y, x*3+2) = color_label[max_idx][0];
        
        if( max_conf >= 0.5 )
        {
            conf_frame.at<uchar>(y, x*3+0) = 255;
            conf_frame.at<uchar>(y, x*3+1) = 255;
            conf_frame.at<uchar>(y, x*3+2) = 255;
        }
        else
        {
            float val = 255*2*max_conf;
            conf_frame.at<uchar>(y, x*3+0) = val;
            conf_frame.at<uchar>(y, x*3+1) = val;
            conf_frame.at<uchar>(y, x*3+2) = val;
        }
    }
    
    return label_cloud;
}



void calibrate(std::vector<Hypo> &hypo_set)
{
    for( std::vector<Hypo>::iterator it = hypo_set.begin() ; it < hypo_set.end() ; )
    {
        //std::cerr << it->label << " ";
        if( it->score < 0 )
            hypo_set.erase(it); //it->score = 0.01;
        else
        {
            if( it->label == 2 )
                it->score = -it->score;
            it++;
        }
    }
    //std::cin.get();
}

pcl::PointCloud<PointT>::Ptr removePlane(const pcl::PointCloud<PointT>::Ptr scene)
{
    pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(scene);
    seg.segment(*inliers, *plane_coef);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (scene);
    proj.setModelCoefficients (plane_coef);

    pcl::PointCloud<PointT>::Ptr scene_projected(new pcl::PointCloud<PointT>());
    proj.filter (*scene_projected);

    pcl::PointCloud<PointT>::iterator it_ori = scene->begin();
    pcl::PointCloud<PointT>::iterator it_proj = scene_projected->begin();
    
    pcl::PointCloud<PointT>::Ptr scene_f(new pcl::PointCloud<PointT>());
    for( int base = 0 ; it_ori < scene->end(), it_proj < scene_projected->end() ; it_ori++, it_proj++, base++ )
    {
        
        float diffx = it_ori->x-it_proj->x;
        float diffy = it_ori->y-it_proj->y;
        float diffz = it_ori->z-it_proj->z;

        if( diffx * it_ori->x + diffy * it_ori->y + diffz * it_ori->z >= 0 )
            continue;
        //distance from the point to the plane
        float dist = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
        
        if ( dist >= 0.03 )//fabs((*it_ori).x) <= 0.1 && fabs((*it_ori).y) <= 0.1 )
            scene_f->push_back(*it_ori);
    }
    
    return scene_f;
}

/*
 * 

pcl::PointCloud<PointT>::Ptr labelCloud_C(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<Hypo> &hypo_set, cv::Mat &label_frame, cv::Mat &conf_frame)
{
    pcl::PointCloud<PointT>::Ptr label_cloud(new pcl::PointCloud<PointT>());
    //pcl::copyPointCloud(*cloud, *label_cloud);
    label_frame = cv::Mat::zeros(480, 640, CV_8UC3);
    conf_frame = cv::Mat::zeros(480, 640, CV_8UC3);
    
    if( hypo_set.empty() )
    {
        label_cloud->push_back(PointT ());
        return label_cloud;
    }
    
    cv::Mat uv, map2d;
    std::vector<float> conf_vec(cloud->size(), 0);
    std::vector<int> conf_count(cloud->size(), 0);
    SceneOn2D(cloud, uv, map2d, 1.0, this_fx, this_fy, center_x, center_y);
    for( size_t i = 0 ; i < hypo_set.size() ; i++)
    {
        cv::Rect reg = hypo_set[i].box;
        float cur_ratio = hypo_set[i].ap_ratio;
        int r_s = reg.tl().y / cur_ratio;
        int r_e = reg.br().y / cur_ratio;
        int c_s = reg.tl().x / cur_ratio;
        int c_e = reg.br().x / cur_ratio;
        
        for(int r = r_s ; r <= r_e ; r++ ){
            for(int c = c_s ; c <= c_e ; c++ ){
                int idx = map2d.at<int>(r, c);
                if( idx >= 0 )
                {
                    conf_vec[idx] += hypo_set[i].score;
                    conf_count[idx]++;
                }
            }
        }
    }
    
    uint8_t tmp_color = 255;
    uint32_t red = tmp_color << 16;
    uint32_t blue = tmp_color;
    
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud (cloud);
    int K = 5;
    for( size_t i = 0 ; i < cloud->size() ; i++ )
    {
        int y = uv.at<int>(i, 0);
        int x = uv.at<int>(i, 1);
        float score = conf_vec[i] / conf_count[i];
        
        PointT tmp = cloud->at(i);
        if( score > 0 )
        {
            tmp.rgba = red;
            label_cloud->push_back(tmp);
            label_frame.at<uchar>(y, x*3+0) = 0;
            label_frame.at<uchar>(y, x*3+1) = 0;
            label_frame.at<uchar>(y, x*3+2) = 255;
            if( score >= 0.5 )
            {
                conf_frame.at<uchar>(y, x*3+0) = 0;
                conf_frame.at<uchar>(y, x*3+1) = 0;
                conf_frame.at<uchar>(y, x*3+2) = 255;
            }
            else
            {
                conf_frame.at<uchar>(y, x*3+0) = 0;
                conf_frame.at<uchar>(y, x*3+1) = 0;
                conf_frame.at<uchar>(y, x*3+2) = 255*2*score;
            }
        }
        else if(score < 0)
        {
            tmp.rgba = blue;
            label_cloud->push_back(tmp);
            label_frame.at<uchar>(y, x*3+0) = 255;
            label_frame.at<uchar>(y, x*3+1) = 0;
            label_frame.at<uchar>(y, x*3+2) = 0;
            if( score <= -0.5 )
            {
                conf_frame.at<uchar>(y, x*3+0) = 255;
                conf_frame.at<uchar>(y, x*3+1) = 0;
                conf_frame.at<uchar>(y, x*3+2) = 0;
            }
            else
            {
                conf_frame.at<uchar>(y, x*3+0) = 255*(-2*score);
                conf_frame.at<uchar>(y, x*3+1) = 0;
                conf_frame.at<uchar>(y, x*3+2) = 0;
            }
        }
        else
        {
            std::vector<int> idx(K);
            std::vector<float> sqr_dist(K);
            int nres = tree.nearestKSearch(tmp, K, idx, sqr_dist);
            for(int j = 0 ; j < nres ; j++ ){
                float score_tmp = conf_vec[idx[j]];
                if( score_tmp != 0 )
                {
                    if( score_tmp < 0)
                        tmp.rgba = blue;
                    else
                        tmp.rgba = red;
                    label_cloud->push_back(tmp);
                    break;
                }
            }
        }
    }
    
    return label_cloud;
}

 void colorHypos(const std::vector<Hypo> &hypo_set, cv::Mat &rgb_frame)
{
    cv::Scalar color_red(0, 0, 255);
    cv::Scalar color_green(0, 255, 0);
    cv::Scalar color_blue(255, 0, 0);
    
    for( size_t i = 0 ; i < hypo_set.size() ; i++)
    {
        float ap_ratio = hypo_set[i].ap_ratio;
        cv::Point new_tl(hypo_set[i].box.tl().x / ap_ratio, hypo_set[i].box.tl().y / ap_ratio);
        cv::Point new_br(hypo_set[i].box.br().x / ap_ratio, hypo_set[i].box.br().y / ap_ratio);
        
        cv::Scalar color;
        if(hypo_set[i].label == 0)
           color = color_red; 
        else if (hypo_set[i].label == 1)
            color = color_blue;
        cv::rectangle(rgb_frame, new_tl, new_br, color, 2, 8, 0 );
    }
}

void colorConfMap(const std::vector<Hypo> &hypo_set, cv::Mat &rgb_frame, bool binary)
{
    for( size_t i = 0 ; i < hypo_set.size() ; i++)
    {
        cv::Rect reg = hypo_set[i].box;
        cv::Point center( (reg.tl().x + reg.width/2) / hypo_set[i].ap_ratio, (reg.tl().y + reg.height/2) / hypo_set[i].ap_ratio);
        
        //int pred_label = hypo_set[i].label;
        float score = hypo_set[i].score;
        
        cv::Scalar color;
        if( binary == false )
        {
            if( score <= -0.5 )
                color = cv::Scalar (255, 0, 0);
            else if( score >= 0.5 )
                color = cv::Scalar (0, 0, 255);
            else if( score < 0 )
                color = cv::Scalar (255*(-2*score), 0, 0);
            else if( score >= 0 )
                color = cv::Scalar (0, 0, 255*2*score);

            cv::circle(rgb_frame, center, 2, color, 2);
        }
        else
        {
            if( score > 0 )
                color = cv::Scalar (0, 0, 255);
            else if ( score < 0 )
                color = cv::Scalar (255, 0, 0);
            
            cv::circle(rgb_frame, center, 2, color, 2);
        }
    }
}

pcl::PointCloud<PointT>::Ptr labelCloud(const pcl::PointCloud<PointT>::Ptr cloud, const std::vector<Hypo> &hypo_set)
{
    pcl::PointCloud<PointT>::Ptr label_cloud(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cloud, *label_cloud);
    
    if( hypo_set.empty() )
        return label_cloud;
    
    cv::Mat uv, map2d;
    SceneOn2D(cloud, uv, map2d, 1.0, this_fx, this_fy, center_x, center_y);
    
    uint8_t tmp_color = 255;
    uint32_t red = tmp_color << 16;
    uint32_t blue = tmp_color;
    
    pcl::PointCloud<PointT>::Ptr pivot_cloud(new pcl::PointCloud<PointT>());
    for( size_t i = 0 ; i < hypo_set.size() ; i++)
    {
        cv::Rect reg = hypo_set[i].box;
        int c = round((reg.tl().x + reg.width/2) / hypo_set[i].ap_ratio);
        int r = round((reg.tl().y + reg.height/2) / hypo_set[i].ap_ratio);
       
        //std::cerr << r << " " << c << " " << map2d.at<int>(r, c) << std::endl;
        if( map2d.at<int>(r, c) < 0 )
            continue;
        PointT tmp = cloud->at(map2d.at<int>(r, c));
        float score = hypo_set[i].score;
        
        //tmp.rgba = *reinterpret_cast<uint32_t*>(&score);
        //float buf = *reinterpret_cast<float*>(&tmp.rgba);
        if( score > 0 )
            tmp.rgba = red;
        else if ( score < 0 )
            tmp.rgba = blue;
        pivot_cloud->push_back(tmp);
    }
    
    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud (pivot_cloud);

    float T = 0.02*0.02;
    for ( pcl::PointCloud<PointT>::iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        std::vector<int> indices (1);
	std::vector<float> sqr_distances (1);
        int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
        if (nres == 1 && sqr_distances[0] < T)
            (*it).rgba = pivot_cloud->at(indices[0]).rgba;
        else
            (*it).rgba = 0;
    }
    
    return label_cloud;
}
 */