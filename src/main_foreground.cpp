#include "sp_segmenter/greedyObjRansac.h"
#include <pcl/filters/fast_bilateral_omp.h>
#include <opencv2/core/core.hpp>
//#include <bits/algorithmfwd.h>

#define OBJECT_MAX 100
//=========================================================================================================================
//std::string link_mesh_name("data/link_uniform");
//std::string node_mesh_name("data/node_uniform");

//std::string drill_mesh_name("../driller_uniform");
//std::string sander_mesh_name("../sander_uniform");

std::vector<poseT> RefinePoses(const pcl::PointCloud<myPointXYZ>::Ptr scene, const std::vector<ModelT> &mesh_set, const std::vector<poseT> &all_poses)
{
    int pose_num = all_poses.size();
    std::vector<ModelT> est_models(pose_num);
    pcl::PointCloud<myPointXYZ>::Ptr down_scene(new pcl::PointCloud<myPointXYZ>());
    pcl::VoxelGrid<myPointXYZ> sor;
    sor.setInputCloud(scene);
    sor.setLeafSize(0.005, 0.005, 0.005);
    sor.filter(*down_scene);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for(int i = 0 ; i < pose_num ; i++ ){
        for( int j = 0 ; j < mesh_set.size() ; j++ ){
            if( mesh_set[j].model_label == all_poses[i].model_name )
            {
                est_models[i].model_label = all_poses[i].model_name;
                est_models[i].model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*mesh_set[j].model_cloud, *est_models[i].model_cloud, all_poses[i].shift, all_poses[i].rotation);
                break;
            }
        } 
    }
    
    std::vector< pcl::search::KdTree<myPointXYZ>::Ptr > tree_set(est_models.size());
    #pragma omp parallel for schedule(dynamic, 1)
    for( int i = 0 ; i < pose_num ; i++ )
    {
        tree_set[i] = pcl::search::KdTree<myPointXYZ>::Ptr (new pcl::search::KdTree<myPointXYZ>());
        tree_set[i]->setInputCloud(est_models[i].model_cloud);
    }   
    
    std::vector<int> votes(pose_num, 0);
    std::vector< std::vector<int> > adj_graph(pose_num);
    for( int i = 0 ; i < pose_num ; i++ )
        adj_graph[i].resize(pose_num, 0);
    float sqrT = 0.01*0.01;
    int down_num = down_scene->size();
    
    std::vector< std::vector<int> > bin_vec(down_num);
    #pragma omp parallel for
    for(int i = 0 ; i < pose_num ; i++ )
    {
        int count = 0;
        for( pcl::PointCloud<myPointXYZ>::const_iterator it = down_scene->begin() ; it < down_scene->end() ; it++, count++ )
        {
            std::vector<int> idx (1);
            std::vector<float> sqrDist (1);
            int nres = tree_set[i]->nearestKSearch(*it, 1, idx, sqrDist);
            if ( nres >= 1 && sqrDist[0] <= sqrT )
            {
                #pragma omp critical
                {   
                    bin_vec[count].push_back(i);
                }
                votes[i]++;
            }
        }
    }
    
    for( int it = 0 ; it < down_num ; it++ )
        for( std::vector<int>::iterator ii = bin_vec[it].begin() ; ii < bin_vec[it].end() ; ii++ )
            for( std::vector<int>::iterator jj = ii+1 ; jj < bin_vec[it].end() ; jj++ )
            {
                adj_graph[*ii][*jj]++;
                adj_graph[*jj][*ii]++;
            }
    std::vector<bool> dead_flag(pose_num, 0);
    for( int i = 0 ; i < pose_num ; i++ ){
        if( dead_flag[i] == true )
            continue;
        for( int j = i+1 ; j < pose_num ; j++ )
        {
            if( dead_flag[j] == true )
                continue;
            int min_tmp = std::min(votes[i], votes[j]);
            if( (adj_graph[i][j]+0.0) / min_tmp >= 0.3 )
            {
                std::cerr << votes[i] << " " << i << std::endl;
                std::cerr << votes[j] << " " << j << std::endl;
                if( votes[i] > votes[j] )
                    dead_flag[j] = true;
                else
                {
                    dead_flag[i] = true;
                    break;
                }
            }
        }
    }
    std::vector<poseT> refined_poses;
    for( int i = 0 ; i < pose_num ; i++ )   
        if( dead_flag[i] == false )
            refined_poses.push_back(all_poses[i]);
    
    return refined_poses;
}


int main(int argc, char** argv)
{
    double pairWidth = 0.1;
    double voxelSize = 0.003; 
    
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string ori_path("/home/chi/JHUIT/scene/");
//    std::string in_path("../../data_pool/GT_segs_full/");
    std::string in_path("../../data_pool/semanticLabels/");
    std::string out_path("fore_poses/");
    std::string scene_name("office");
    
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    pcl::console::parse_argument(argc, argv, "--w", pairWidth);
    pcl::console::parse_argument(argc, argv, "--i", scene_name);
    boost::filesystem::create_directories(out_path);

/***********************************************************************************************************************/    
    bool view_flag = false;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( view_flag == true )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer());
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setSize(640, 480);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    }
/***********************************************************************************************************************/    
    std::map<std::string, int> model_name_map;
    std::map<std::string, float> model_pairWidth;
    setObjID(model_name_map);
/***********************************************************************************************************************/    
            
    boost::filesystem::path mp(mesh_path);
    std::vector< std::string > ret;
    find_files(mp, ".obj", ret);
    boost::shared_ptr<greedyObjRansac> objrec(new greedyObjRansac(pairWidth, voxelSize));
    std::vector<std::string> model_name(OBJECT_MAX, "");
    std::vector<ModelT> mesh_set;
    for( size_t j = 0 ; j < ret.size() ; j++ )
    {
        std::string cur_name = ret[j].substr(0, ret[j].size()-4);
        
        int model_id = model_name_map[cur_name];
        if( model_id > 0 )
        {
            std::cerr << "Loading *** " << cur_name << std::endl;
            
            objrec->AddModel(mesh_path + cur_name, cur_name);
            model_name[model_id] = cur_name;
            ModelT mesh_buf = LoadMesh(mesh_path + cur_name + ".obj", cur_name);
            
            mesh_set.push_back(mesh_buf);
        }
    }
    
        
/***********************************************************************************************************************/    
    
    std::vector< std::string > prefix_set(1);
    prefix_set[0] = scene_name;
    
    pcl::VoxelGrid<myPointXYZ> sor;
    for( int tt = 0 ; tt <= 0 ; tt++ )
    {
        std::vector<std::string> scene_names;
        
        for( int i = 0 ; i < 10 ; i++ )
        {
            std::stringstream ss;
            ss << i;
            scene_names.push_back(std::string (prefix_set[tt] +"_"+ ss.str()));
        }

        for( std::vector<std::string>::iterator scene_it = scene_names.begin() ; scene_it < scene_names.end() ; scene_it++ )
        {
            std::string cur_path(in_path + *scene_it + "/");
            std::string cur_ori (ori_path + *scene_it + "/");
            std::string cur_out(out_path + *scene_it + "/");
            boost::filesystem::create_directories(cur_out);
            
            for( int i = 0 ; i <= 99 ; i++ )
            {
                std::stringstream ss;
                ss << i;
                
                std::string filename(cur_path + *scene_it + "_" + ss.str() + "_seg.pcd");
                std::cerr << filename << std::endl;
                
                pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
                pcl::io::loadPCDFile(filename, *label_cloud);
                pcl::PointCloud<PointT>::Ptr scene_ori(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(cur_ori + *scene_it + "_" + ss.str() + ".pcd", *scene_ori);
                
                pcl::PointCloud<myPointXYZ>::Ptr foreground(new pcl::PointCloud<myPointXYZ>());
                pcl::copyPointCloud(*label_cloud, *foreground);
                
                std::vector<poseT> all_poses1;
                objrec->StandardRecognize(foreground, all_poses1);
               
                pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
                pcl::copyPointCloud(*scene_ori, *scene_xyz);
                
                std::vector<poseT> all_poses =  RefinePoses(scene_xyz, mesh_set, all_poses1);
                for(size_t j = 1 ; j < OBJECT_MAX ; j++ )
                    if( model_name[j].empty() == false )
                        writeCSV(cur_out + model_name[j] + "_" + ss.str() + ".csv", model_name[j], all_poses);

                if( view_flag == true )
                {
                    viewer->removeAllPointClouds();
                    viewer->addPointCloud(scene_ori, "whole_scene");
//                    int cur_color[3] = {255, 128, 0};
                    objrec->visualize_m(viewer, all_poses, model_name_map, model_color);
                    viewer->spin();
                    objrec->clearMesh(viewer, all_poses);
                    viewer->removeAllPointClouds();
//                    viewer->spinOnce(1);
//                    viewer->saveScreenshot(cur_out + "demo_" + ss.str() + ".png");
                }
                
            }
        }
    }
    return 0;
     
}


    
