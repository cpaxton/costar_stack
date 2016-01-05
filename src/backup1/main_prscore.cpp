#include <opencv2/core/core.hpp>
#include "../include/features.h"
#include "../include/UWDataParser.h"

//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/UW_scene/");
    std::string workspace("../data_pool/UW_5x5_s30/");
    std::string model_path(workspace + "SVM_Model/");
    std::string scene_name("tmp");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", workspace);
    pcl::console::parse_argument(argc, argv, "--sname", scene_name);
    
    model* cur_model = load_model((model_path + "svm_model.model").c_str());
    std::string out_path( workspace + "result_pool/" + scene_name + "/" );
        
    std::vector< std::vector<Hypo> > gt_set, hypo_set;
    readGround(in_path + scene_name + "/" + scene_name + "_gt.txt", gt_set);
    readGround(out_path + "loc.txt", hypo_set);
    
    std::vector< std::pair<float, float> > pr_set = PRScore(hypo_set, gt_set, cur_model->nr_class);
    
    //for( int i = 1 ; i <= cur_model->nr_class ; i++ )
    //{
    //    std::cerr << std::endl << "Object-" << i << std::endl;
    //    if( obj_flag[i] == true )
    //    {
    //        std::cerr << "Precision:" << std::endl;
    //        for(size_t j = 0 ; j < whole_pr.size() ; j++ )
    //            std::cerr << "seq-" << j+s1 << ": " << whole_pr[j][i].first << std::endl;

    //        std::cerr << "Recall:" << std::endl;
    //        for(size_t j = 0 ; j < whole_pr.size() ; j++ )
    //            std::cerr << "seq-" << j+s1 << ": " << whole_pr[j][i].second << std::endl;
    //    }
    //}
    
    free_and_destroy_model(&cur_model);
    return 1;
}
//*/

/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/UW_scene/");
    std::string scene_name("desk_1");
    float ss = 0.003;
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--sn", scene_name);
    pcl::console::parse_argument(argc, argv, "--ss", ss);
    
    std::string cur_path( in_path + scene_name + "/" );
    
    std::vector< std::vector<Hypo> > gt_set;
    int frame_num = readGround(in_path + scene_name + "/" + scene_name + "_gt.txt", gt_set);
    
    for( int i = 1 ; i <= frame_num  ; i++ )
    {
        
        std::stringstream ss;
        ss << i;
        
        std::string filename(cur_path + scene_name + "_" + ss.str() + ".pcd");
        
        std::cerr << "Loading-" << filename << std::endl;
            
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *full_cloud);
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        std::vector<int> idx_ff;
        pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);
        
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (cloud);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0, 4.0);
        pass.filter (*cloud_filtered);
        
        //cv::Mat rgb = getImage(cloud_filtered);
        cv::Mat uv, map2d;
        SceneOn2D(cloud_filtered, uv, map2d);
        for( size_t j = 0 ; j < gt_set[i-1].size() ; j++ )
        {
            bool good = false;
            for(int r = gt_set[i-1][j].box.tl().y ; r <= gt_set[i-1][j].box.br().y ; r++ ){
                for(int c = gt_set[i-1][j].box.tl().x ; c <= gt_set[i-1][j].box.br().x ; c++ ){
                    if(map2d.at<int>(r, c) >= 0 )
                    {
                        good = true;
                        break;
                    }
                }
            }
            if( !good )
            {
                std::cerr << "Failed: " << i<< std::endl;
                std::cerr << gt_set[i-1][j].label << " " << gt_set[i-1][j].box.tl().x
                                                << " " << gt_set[i-1][j].box.tl().y
                                                << " " << gt_set[i-1][j].box.br().x
                                                << " " << gt_set[i-1][j].box.br().y << std::endl;
            }
        }
        
    }
    
    
    return 1;
}
//*/

/*
int sizeBin(const pcl::PointCloud<PointT>::Ptr cloud)
{
    int bin_id = -1;
    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;
    for( pcl::PointCloud<PointT>::const_iterator pt_ptr = cloud->begin() ; pt_ptr < cloud->end() ; pt_ptr++ )
    {
        if( pt_ptr->x > max_x ) max_x = pt_ptr->x;
        if( pt_ptr->x < min_x ) min_x = pt_ptr->x;
        if( pt_ptr->y > max_y ) max_y = pt_ptr->y;
        if( pt_ptr->y < min_y ) min_y = pt_ptr->y;
    }
    float width = max_x - min_x;
    float height = max_y - min_y;
    
    width = width >= 0.4 ? 0.39999 : width;
    height = height >= 0.4 ? 0.39999 : height;
    
    if( width > 0 && height > 0)
        bin_id = floor(width / 0.02) * 20 + floor(height / 0.02);
    return bin_id;
}

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/UW_scene/Object_Models/");
    
    int c1 = 0, c2 = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    int label_count = 1;
    std::vector<int> hist(400, 0);
    int count = 0;
    for( int i = c1 ; i <= c2  ; i++, label_count++ )
    {
        ObjectSet train_objects, test_objects;
        readUWInst(in_path, train_objects, test_objects, i, i, 1);
        std::cerr << "Loading Completed... " << std::endl;
        for( size_t j = 0 ; j < train_objects[0].size() ; j++ )
        {
            MulInfoT *inst_ptr = &train_objects[0][j];
            int idx = sizeBin(inst_ptr->cloud);
            hist[idx]++;
            count++;
        }
        
        for( size_t j = 0 ; j < test_objects[0].size() ; j++ )
        {
            MulInfoT *inst_ptr = &test_objects[0][j];
            int idx = sizeBin(inst_ptr->cloud);
            hist[idx]++;
            count++;
        }
    }
    std::ofstream fp("size_box.txt");
    
    fp << count << std::endl;
    for(size_t i = 0 ; i < hist.size() ; i++ )
    {
        float w = (i / 20) * 0.02 + 0.01;
        float h = (i % 20) * 0.02 + 0.01;
        fp << w << " " << h << " " << hist[i] << std::endl;
    }
    fp.close();
    
    return 1;
} 
//*/ 