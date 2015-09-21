#include <opencv2/highgui/highgui.hpp>
#include <bits/basic_string.h>
#include <bits/stl_vector.h>
#include <bits/basic_ios.h>

#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

std::string BBpath("/home/chi/BigBIRD/processed");
std::string UWpath("/home/chi/UW_RGBD/filtered_pcd");
std::string JHUpath("/home/chi/JHUIT/filtered_pcd");

struct poseT{
    std::string model_name;
    Eigen::Vector3f shift;
    Eigen::Quaternion<float> rotation;
};

int readCSV(std::string filename, std::string label, std::vector<poseT> &poses)
{
    std::ifstream fp;
    fp.open(filename.c_str());
    if(fp.is_open() == false)
        return 0;
    
    int num;
    fp >> num;
    for(int i = 0 ; i < num ; i++)
    {
        poseT tmp;
        fp >> tmp.shift(0) >> tmp.shift(1) >> tmp.shift(2);
        float x,y,z,w;
        fp >> x >> y >> z >> w;
        tmp.rotation = Eigen::Quaternionf (w,x,y,z);
        tmp.model_name = label;
        
        poses.push_back(tmp);
    }
    fp.close();
    return 1;
}

int writeCSV(std::string filename, std::string label, const std::vector<poseT> &poses)
{
    std::ofstream fp;
    fp.open(filename.c_str());
    if( fp.is_open() == false )
    if( fp.is_open() == false )
    {
        std::cerr << "Failed to open files" << std::endl;
        return 0;
    }
    
    int count=0;
    for( std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
        if( it->model_name == label )
            count++;
    
    fp << count << std::endl;
    for( std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++ )
    {
        if( it->model_name == label )
        {
            fp << it->shift(0) << " " << it->shift(1) << " " << it->shift(2) << " "
               << it->rotation.x() << " " << it->rotation.y() << " " << it->rotation.z() << " " << it->rotation.w() << std::endl;
        }
    }
    fp.close();
    return 1;
}


cv::Mat getFullImage(const pcl::PointCloud<PointT>::Ptr full_cloud)
{
    int width = full_cloud->width;
    int height = full_cloud->height;
    
    cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
    for( size_t i = 0 ; i < full_cloud->size() ; i++ )
    {
        uint32_t rgb = full_cloud->at(i).rgba;
        int r = i / width;
        int c = i % width;
        
        img.at<uchar>(r, c*3+2) = (rgb >> 16) & 0x0000ff;
        img.at<uchar>(r, c*3+1) = (rgb >> 8) & 0x0000ff;
        img.at<uchar>(r, c*3+0) = (rgb) & 0x0000ff;
    }
    
    return img;
}

void flipXZ(pcl::PointCloud<PointT>::Ptr cloud)
{
    for(pcl::PointCloud<PointT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        float tmp_x = it->x;
        float tmp_y = it->y;
        float tmp_z = it->z;
        
        it->z = tmp_x;
        it->x = tmp_y;
        it->y = -tmp_z;
    }
}

/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/ln_new/");
    std::string out_path("/home/chi/JHUIT/scene/ln_new_gt/");
    
    for( int i = 0 ; i < 490 ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        
        pcl::io::loadPCDFile(in_path + "ln_new_" + ss.str() + ".pcd", *cloud);
        pcl::copyPointCloud(*cloud, *cloud_rgb);
        
        std::cerr << "Loading..." << in_path + "ln_new_" + ss.str() + ".pcd" << std::endl;
        pcl::PCLPointCloud2::Ptr new_cloud(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*cloud_rgb, *new_cloud);
        
        pcl::io::savePCDFile(out_path + "ln_new_" + ss.str() + "_ros.pcd", *new_cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), true);
    }
}
*/

/*
int main(int argc, char** argv)
{
    //std::ifstream in;
    //in.open("idx.txt");
    
    std::string data_path("/home/chi/devel_mode/obj_ransac/result/for_init/");
    std::string out_path("/home/chi/devel_mode/obj_ransac/result/for_init_adjusted/");
    //std::ofstream out;
    //out.open("idx.txt");
    
    //std::vector<std::string> path_vec(49);
    //for(int i = 0 ; i < 490 ; i++ )
    //{
    //    std::string name1, name2;
    //    in >> name1 >> name2;
        //std::cerr << name1 << " " << name2 << std::endl;
        //out << i << " " << name2 << std::endl;
    //    if( i % 10 == 0 )
    //        path_vec[i/10] = name2;
    //}
    //in.close();
    
    Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f (1, 0, 0)));
    for(int i = 0 ; i < 49 ; i++ )
    {
        int idx = i * 10;
        std::stringstream ii;
        ii << idx;
        //std::cerr << path_vec[i] << std::endl;
        
        std::vector<poseT> link_poses, node_poses;
        readCSV(data_path+"link_pose_seg_"+ii.str()+".csv", "link", link_poses);
        readCSV(data_path+"node_pose_seg_"+ii.str()+".csv", "node", node_poses);
        
        for( int j = 0 ; j < link_poses.size() ; j++ )
            link_poses[j].rotation = link_poses[j].rotation*calibrate_rot;
        for( int j = 0 ; j < node_poses.size() ; j++ )
            node_poses[i].rotation = node_poses[j].rotation*calibrate_rot;
        
        writeCSV(out_path+"link_pose_seg_"+ii.str()+".csv", "link", link_poses);
        writeCSV(out_path+"node_pose_seg_"+ii.str()+".csv", "node", node_poses);
        
    }
    
    //out.close();
}
//*/
/*
int main(int argc, char** argv)
{
    std::ifstream in;
    in.open("idx_to_dir.txt");
    
    std::ofstream out;
    out.open("idx.txt");
    
    for(int i = 0 ; i < 490 ; i++ )
    {
        std::string name1, name2;
        in >> name1 >> name2;
        //std::cerr << name1 << " " << name2 << std::endl;
        out << i << " " << name2 << std::endl;
    }
    
    in.close();
    out.close();
}
//*/
/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/ln_raw_1/1_cloud/");
    std::string scene_name("ln_new");
    std::string out_path("/home/chi/JHUIT/scene/" + scene_name + "/");
    
    std::ofstream idx_class;
    idx_class.open("idx_to_dir.txt");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
                           
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    //viewer->initCameraParameters();
    //viewer->addCoordinateSystem(0.1);
    //viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);

//////////////////////////////////////////////////////////////////////////////////////////////////////    
    int min_num = 200;
    int interval = 10;
    
    boost::filesystem::path p0(in_path);
    std::vector< std::string > ret1;
    find_dirs(p0, ret1);
    
    int total_count = 0;
    for( size_t i = 0 ; i < ret1.size() ; i++ )
    {
        boost::filesystem::path p1(in_path + ret1[i] + "/");
        std::vector< std::string > ret2;
        find_dirs(p1, ret2);
        for( size_t k = 0 ; k < ret2.size() ; k++ )
        {
            std::string cur_path(in_path + ret1[i] + "/" + ret2[k] + "/");
            std::vector<std::string> pcd_files;
            getNonNormalPCDFiles(cur_path, pcd_files);
            
            //std::vector< std::vector<float> > links_poses, nodes_poses;
            //if( readCSV(cur_path+"links_gt.csv", links_poses) == 0 )
            //    exit(0);
            //if( readCSV(cur_path+"nodes_gt.csv", nodes_poses) == 0 )
            //    exit(0);
            
            size_t file_num = pcd_files.size();
            
            pcl::PointCloud<PointT>::Ptr cloud_set(new pcl::PointCloud<PointT>());
            for( size_t j = 0 ; j < file_num ; j++ )
            {
                std::string filename(cur_path + pcd_files[j]);
                std::string filename_n(cur_path+"normal_"+pcd_files[j]);
                
                std::cerr << "Loading... "<< filename << std::endl;
                //*
                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(filename, *full_cloud);
                if( full_cloud->empty() == false )
                {
                    
                    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                    std::vector<int> idx_ff;
                    pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);

                    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
                    tree->setInputCloud (cloud);
                    flipXZ(cloud);

                    //viewer->addPointCloud(cloud, "cloud");
                    //viewer->spin();

                    pcl::EuclideanClusterExtraction<PointT> ec;
                    ec.setClusterTolerance (0.015);     // 1.5cm
                    ec.setMinClusterSize (min_num);
                    ec.setMaxClusterSize (INF_);
                    ec.setSearchMethod (tree);
                    ec.setInputCloud (cloud);
                    std::vector<pcl::PointIndices> cluster_indices;
                    ec.extract (cluster_indices);

                    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
                    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it )
                        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                            filtered_cloud->push_back (cloud->at(*pit));

                    cloud_set->insert(cloud_set->end(), filtered_cloud->begin(), filtered_cloud->end());
                }
                if( (j+1) % interval == 0 || j == file_num - 1)
                {
                    std::stringstream idx_str;
                    idx_str << total_count;
                    
                    //*
                    pcl::PointCloud<PointT>::Ptr cloud_mean(new pcl::PointCloud<PointT>());
                    pcl::PointCloud<NormalT>::Ptr cloud_mean_normals(new pcl::PointCloud<NormalT>());
                    
                    pcl::VoxelGrid<PointT> sor;
                    sor.setInputCloud(cloud_set);
                    sor.setLeafSize(0.0015, 0.0015, 0.0015);
                    sor.filter(*cloud_mean);
                    
                    computeNormals(cloud_mean, cloud_mean_normals, 0.03);
                    pcl::io::savePCDFile(filename_n, *cloud_mean, true);
                    
                    pcl::io::savePCDFile(out_path + scene_name + "_" + idx_str.str() + ".pcd", *cloud_mean, true);
                    pcl::io::savePCDFile(out_path + "normal_"+scene_name+"_" + idx_str.str() + ".pcd", *cloud_mean_normals, true);
                    
                    //if( writeCSV(out_path + "link_gt_" + idx_str.str() + ".txt", links_poses) == 0 )
                    //    exit(0);
                    //if( writeCSV(out_path + "node_gt_" + idx_str.str() + ".txt", nodes_poses) == 0 )
                    //    exit(0);
                    
                    cv::Mat img = getImage(cloud_mean, 1.0, 539.6096276855468, 539.6096276855468, 319.5, 239.5);
                    cv::imwrite(out_path + "img_"+scene_name+"_" + idx_str.str() + ".png", img);
                    
                    idx_class << scene_name + "_" + idx_str.str() + ".pcd" << " " << cur_path << std::endl;
                    //cv::imshow("show", img);
                    //cv::waitKey();

                    total_count++;
                    cloud_set->clear();
                }
            }
        
        }
    }
    idx_class.close();
    return 1;
}
//*/

/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/jon_data/");
    std::string out_path("/home/chi/JHUIT/scene/ln/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
                           
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);

//////////////////////////////////////////////////////////////////////////////////////////////////////    
    int min_num = 200;
    
    boost::filesystem::path p0(in_path);
    std::vector< std::string > ret1;
    find_dirs(p0, ret1);
    
    int total_count = 0;
    for( size_t i = 0 ; i < ret1.size() ; i++ )
    {
        boost::filesystem::path p1(in_path + ret1[i] + "/");
        std::vector< std::string > ret2;
        find_dirs(p1, ret2);
        for( size_t k = 0 ; k < ret2.size() ; k++ )
        {
            std::string cur_path(in_path + ret1[i] + "/" + ret2[k] + "/");
            std::vector<std::string> pcd_files;
            getNonNormalPCDFiles(cur_path, pcd_files);
            
            size_t file_num = pcd_files.size();
            
            for( size_t j = 0 ; j < file_num ; j++ )
            {
                std::string filename(cur_path + pcd_files[j]);
                std::string filename_n(cur_path+"normal_"+pcd_files[j]);

                std::cerr << "Loading... "<< filename << std::endl;
                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(filename, *full_cloud);
                
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                std::vector<int> idx_ff;
                pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);
        
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
                if( exists_test(filename_n) == false )
                {
                    computeNormals(cloud, cloud_normal, 0.03);
                    pcl::io::savePCDFile(filename_n, *cloud_normal, true);
                }
                else
                    pcl::io::loadPCDFile(filename_n, *cloud_normal);
                
                pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
                tree->setInputCloud (cloud);

                pcl::EuclideanClusterExtraction<PointT> ec;
                ec.setClusterTolerance (0.015);     // 1.5cm
                ec.setMinClusterSize (min_num);
                ec.setMaxClusterSize (INF_);
                ec.setSearchMethod (tree);
                ec.setInputCloud (cloud);
                std::vector<pcl::PointIndices> cluster_indices;
                ec.extract (cluster_indices);
                
                pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
                pcl::PointCloud<NormalT>::Ptr filtered_cloud_normal(new pcl::PointCloud<NormalT>());
                    
                for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it )
                {
                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                    {
                        filtered_cloud->push_back (cloud->at(*pit));
                        filtered_cloud_normal->push_back(cloud_normal->at(*pit));
                    }
                    
                }
                std::stringstream idx_str;
                idx_str << total_count;

                pcl::io::savePCDFile(out_path + "ln_" + idx_str.str() + ".pcd", *filtered_cloud, true);
                pcl::io::savePCDFile(out_path + "normal_ln_" + idx_str.str() + ".pcd", *filtered_cloud_normal, true);
                
                //cv::Mat img = getImage(filtered_cloud);
                //cv::imshow("show", img);
                //cv::waitKey();
                
                total_count++;
            }
        }
    }

    return 1;
}
//*/


//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string scene_name("tmp/");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--sname", scene_name);
    
    float radius = 0.03;
    bool view_flag = false;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    
    if( view_flag )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer());
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    }
   
    double t1, t2;
    std::string workspace(in_path + scene_name);
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);
    //float fx=0, fy=0;
    int pcount = 0;
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        if( exists_test(workspace + "normal_" + (*it)) == true )
            continue;

        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        std::cerr << "Processing-" << workspace + (*it) << std::endl;
        pcl::io::loadPCDFile(workspace + (*it), *full_cloud);
        cv::Mat img = getFullImage(full_cloud);
        cv::imwrite(workspace + it->substr(0, it->size()-4) + ".png", img);

        t1 = get_wall_time();
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        std::vector<int> idx_ff;
        pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);

        pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
        computeNormals(cloud, cloud_normals, radius);

        t2 = get_wall_time();
        std::cerr << "Normal Est Time: "<< t2 - t1 << std::endl;

        if( view_flag )
        {
            viewer->removeAllPointClouds();
            //viewer->removeAllShapes();
            viewer->addPointCloud(cloud, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            //viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 5, 0.02, "cloud_normals");
            viewer->spin();
        }

        pcl::io::savePCDFile(workspace + "normal_" + (*it), *cloud_normals, true);
 
    }
    
    //std::cerr << "FX: " << fx/pcount << std::endl;
    //std::cerr << "FY: " << fy/pcount << std::endl;
        
    return 1;
}
//*/
 
/*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene");
    std::string scene_name("tmp");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--sname", scene_name);
   
    std::string workspace(in_path + "/" +scene_name);
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(workspace, pcd_files);
    float fx=0, fy=0;
    int pcount = 0;
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        std::cerr << "Processing-" << workspace + "/" + (*it) << std::endl;
        pcl::io::loadPCDFile(workspace + "/" + (*it), *full_cloud);

        float fx_sum = 0;
        float fy_sum = 0;
        size_t count = 0;
        for( int i = 0 ; i < full_cloud->size() ; i++ )
        {
            float x = full_cloud->at(i).x;
            float y = full_cloud->at(i).y;
            float z = full_cloud->at(i).z;
            
            if( z == z )
            {
                int img_y = i / 640;  
                int img_x = i % 640;

                float cur_fx = (img_x - 320)*z / x;
                float cur_fy = (img_y - 240)*z / y;
                
                fx_sum += cur_fx;
                fy_sum += cur_fy;
                
                count++;
                
            }
        }
            
        fx += fx_sum / count;
        fy += fy_sum / count;
        pcount++;
    }
        
    return 1;
}
*/
 /*
int main(int argc, char** argv)
{
    std::string path("/home/chi/UW_scene/desk_1/");
    
    pcl::console::parse_argument(argc, argv, "--p", path);
    
    boost::filesystem::path p(path);
    std::vector< std::string > png_files;
    find_files(p, ".png", png_files);
    
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    //viewer->initCameraParameters();
    //viewer->addCoordinateSystem(0.1);
    //viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    for( std::vector< std::string >::iterator it_p = png_files.begin() ; it_p < png_files.end() ; it_p++ )
    {
        if(it_p->at(it_p->size()-5) != 'h' )
        {
            std::string name = it_p->substr(0, it_p->size()-4);
            std::cerr << name << std::endl;
            
            //if( exists_test(path + "normal_" + name + ".pcd") )
            //    continue;
            std::string depth_name(name + "_depth.png");
            cv::Mat rgb, depth;
            rgb = cv::imread(path + (*it_p), CV_LOAD_IMAGE_COLOR);
            depth = cv::imread(path + depth_name, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file 
            depth.convertTo(depth, CV_32F);         // convert the image data to float type
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            for(int r = 0 ; r < rgb.rows ; r++ ){
                for(int c = 0 ; c < rgb.cols ; c++ ){
                    if( depth.at<float>(r, c) == depth.at<float>(r, c) && depth.at<float>(r, c) > 0 )
                    {    
                        float z = depth.at<float>(r, c) / 1000;
                        if ( z != z || z > 4.0 )
                            continue;
                        float x = (c - 320) / FOCAL_X * z;
                        float y = (r - 240) / FOCAL_Y * z;
                        
                        PointT cur_point;
                        cur_point.x = x;
                        cur_point.y = y;
                        cur_point.z = z;
                        
                        uint32_t rr = rgb.at<uchar>(r, c*3+2);
                        uint32_t gg = rgb.at<uchar>(r, c*3+1);
                        uint32_t bb = rgb.at<uchar>(r, c*3+0);
                        
                        uint32_t rgba = (rr << 16) + (gg << 8) + bb;
                        cur_point.rgba = rgba;
                        
                        cloud->push_back(cur_point);
                    }
                }
            }
            if( cloud->empty() )
            {
                std::cerr << "No Point Cloud!!!" << std::endl;
                continue;
            }
            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
            computeNormals(cloud, cloud_normals, 0.03);
            
            //cv::Mat rgb_frame = getImage(cloud, 1.0, FOCAL_X, FOCAL_Y);
            //cv::imshow("Window", rgb_frame);
            //cv::waitKey();
            //viewer->removeAllPointClouds();
            //viewer->removeAllShapes();
            //viewer->addPointCloud(cloud, "cloud");
            //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            //viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 5, 0.02, "cloud_normals");
            //viewer->spin();
            
            pcl::io::savePCDFile(path + name + ".pcd", *cloud, true);
            pcl::io::savePCDFile(path + "normal_" + name + ".pcd", *cloud_normals, true);
        }
    }
    
        
    return 1;
}
 //*/
/*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = UW_INST_MAX-1;
    
    std::string in_path_1("BB_8layer");
    std::string in_path_2("BB_max_8layer");
    pcl::console::parse_argument(argc, argv, "--p1", in_path_1);
    pcl::console::parse_argument(argc, argv, "--p2", in_path_2);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        {
            std::cerr << "Reading: " << in_path_1 + "/train_"+ss.str()+"_L0.smat" << std::endl;
            cv::Mat train_1 = readCvMatSparse(in_path_1 + "/train_"+ss.str()+"_L0.smat");
            std::cerr << "Reading: " << in_path_2 + "/train_"+ss.str()+"_L0.smat" << std::endl;
            cv::Mat train_2 = readCvMatSparse(in_path_2 + "/train_"+ss.str()+"_L0.smat");
        
            cv::Mat diff = train_1 - train_2;
            std::cerr << train_1.rows << " " << train_1.cols << std::endl;
            std::cerr << train_2.rows << " " << train_2.cols << std::endl;
            for( int i = 0 ; i < diff.rows ; i++ ){
                std::cerr << cv::norm(train_1.row(i)) << std::endl;
                std::cerr << cv::norm(train_2.row(i)) << std::endl;
                std::cin.get();
                //for( int j = 0 ; j < diff.cols ; j++ ){
                //    if( fabs(diff.at<float>(i, j)) > 1e-6 )
                //        std::cerr << "(" << i << " , " << j << ")" <<"---" << diff.at<float>(i, j) << std::endl;
                //}
            }
        }
        
        {
            std::cerr << "Reading: " << in_path_1 + "/test_"+ss.str()+"_L0.smat" << std::endl;
            cv::Mat test_1 = readCvMatSparse(in_path_1 + "/test_"+ss.str()+"_L0.smat");
            std::cerr << "Reading: " << in_path_2 + "/test_"+ss.str()+"_L0.smat" << std::endl;
            cv::Mat test_2 = readCvMatSparse(in_path_2 + "/test_"+ss.str()+"_L0.smat");
            cv::Mat diff = test_1 - test_2;
            for( int i = 0 ; i < diff.rows ; i++ ){
                for( int j = 0 ; j < diff.cols ; j++ ){
                    if( fabs(diff.at<float>(i, j)) > 1e-6 )
                        std::cerr << "(" << i << " , " << j << ")" <<"---" << diff.at<float>(i, j) << std::endl;
                }
            }
        
        }
	
        std::cerr << "Done " << i << std::endl;
    }
    
    
    return 1;
}
*/

/*
int main(int argc, char** argv)
{   
    int c1 = 0, c2 = UW_INST_MAX-1;
    
    std::string in_path_1("UW_6layer");
    std::string in_path_2("UW_6rad");
    pcl::console::parse_argument(argc, argv, "--p1", in_path_1);
    pcl::console::parse_argument(argc, argv, "--p2", in_path_2);
    
    std::string out_path("UW_6new");
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);   

    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    for( int i = c1 ; i <= c2  ; i++ )
    {
        std::stringstream ss;
        ss << i;
        
        {
        std::cerr << "Reading: " << in_path_1 + "/train_"+ss.str()+"_L0.smat" << std::endl;
        
        cv::Mat train_1 = readCvMatSparse(in_path_1 + "/train_"+ss.str()+"_L0.smat");
        cv::Mat new_train_1 = train_1.colRange(0, 324000);
        cv::Mat train_2 = readCvMatSparse(in_path_2 + "/train_"+ss.str()+"_L0.smat");
        cv::Mat new_train_2 = train_2.colRange(324000, 352800);
        
        cv::Mat final_train;
        cv::hconcat(new_train_1, new_train_2, final_train);
        saveCvMatSparse(out_path + "/train_"+ss.str()+"_L0.smat", final_train);
        }
        
        {
        std::cerr << "Reading: " << in_path_1 + "/test_"+ss.str()+"_L0.smat" << std::endl;
        
        cv::Mat test_1 = readCvMatSparse(in_path_1 + "/test_"+ss.str()+"_L0.smat");
        cv::Mat new_test_1 = test_1.colRange(0, 324000);
        cv::Mat test_2 = readCvMatSparse(in_path_2 + "/test_"+ss.str()+"_L0.smat");
        cv::Mat new_test_2 = test_2.colRange(324000, 352800);
        
        cv::Mat final_test;
        cv::hconcat(new_test_1, new_test_2, final_test);
        saveCvMatSparse(out_path + "/test_"+ss.str()+"_L0.smat", final_test);
        }
	
    }
    
    
    return 1;
}
*/

/*
int main(int argc, char** argv)
{
    //std::string type; // 0: partial, 1: n_test
    //pcl::console::parse_argument(argc, argv, "--t", type);
    
    //cv::Mat dict = read3d("UW_dict/old/kcenter_so2_"+type+".txt");
    //dict = -dict;
    //std::cerr << dict << std::endl;
    //saveMat("UW_dict/kcenter_so2_"+type+".cvmat", dict);

    //return 0;
    //*
    //int c1 = 0, c2 = BB_INST_MAX;
    int c1 = 0, c2 = UW_INST_MAX;
    float radius = 0.03;
    int type = 0; // 0: partial, 1: n_test
    pcl::console::parse_argument(argc, argv, "--r", radius);
    pcl::console::parse_argument(argc, argv, "--t", type);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    
    //batch_PreProcess_JHU("/home/chi/JHUIT/raw",JHUpath, c1 ,c2);
    std::string in_path("/home/chi/UW_scene/Raw_Models");
    std::string out_path("/home/chi/UW_scene/Object_Models");
    batch_PreProcess_UW(in_path, out_path, c1 ,c2);
    //sweepNaN(UWpath, c1, c2);
    //batch_PreProcess(UWpath, "/home/chi/UW_RGBD/filtered_pcd", c1 ,c2);
    //ObjectSet train_set, test_set;
    //readUWInst(UWpath, train_set, test_set, c1, c2);
    //for(int i = 0 ; i < train_set.size() ; i++ )
    //{
    //    std::cerr << train_set[i].size() << " " << test_set[i].size() << std::endl;
    //}
    //if( pcl::console::find_switch(argc, argv, "-v") == true )
    //{
        //viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ());
        //viewer->initCameraParameters();
        //viewer->addCoordinateSystem(0.1);
    //}
    //StreamingSIFT(path, c1, c2);
    //if( type == 1)
    //    StreamingNormal(path, "n_test", radius, c1, c2);
    //else if (type == 0)
    //    StreamingNormal(path, "partial", radius, c1, c2);
    //std::cerr<<"Min_x: "<<min_x<<", Max_x: "<<max_x<<std::endl;
    //std::cerr<<"Min_y: "<<min_y<<", Max_y: "<<max_y<<std::endl;
    //std::cerr<<"Min_z: "<<min_z<<", Max_z: "<<max_z<<std::endl;
      
    return 1;
}
//*/