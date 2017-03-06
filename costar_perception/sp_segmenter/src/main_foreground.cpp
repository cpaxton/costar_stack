#include "sp_segmenter/greedyObjRansac.h"
#include "sp_segmenter/refinePoses.h"
#include <pcl/filters/fast_bilateral_omp.h>
#include <opencv2/core/core.hpp>
//#include <bits/algorithmfwd.h>

#define OBJECT_MAX 100
//=========================================================================================================================
//std::string link_mesh_name("data/link_uniform");
//std::string node_mesh_name("data/node_uniform");

//std::string drill_mesh_name("../driller_uniform");
//std::string sander_mesh_name("../sander_uniform");
       
uchar model_color[11][3] = 
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


    
