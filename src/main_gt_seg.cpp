#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/common.h"

std::map<std::string, int> model_name_map;

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


void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3])
{
    pcl::PointCloud<PointT>::Ptr view_cloud(new pcl::PointCloud<PointT>());
    for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false || it->label <= 0)
            continue;
        
        PointT pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        pt.rgba = colors[it->label][0] << 16 | colors[it->label][1] << 8 | colors[it->label][2];
        view_cloud->push_back(pt);
    }
    
    viewer->addPointCloud(view_cloud, "label_cloud");
//    viewer->spinOnce(1);
//    viewer->removePointCloud("label_cloud");
}

std::vector<int> spGt(const pcl::PointCloud<PointLT>::Ptr gt_cloud, const std::vector<pcl::PointCloud<PointT>::Ptr> segs)
{
    std::vector<int> tmp_labels(segs.size(), 0);
    if( gt_cloud->empty() == true )
        return tmp_labels;
    
    pcl::search::KdTree<PointLT> tree;
    tree.setInputCloud(gt_cloud);
    
    float T = 0.001;
    float sqrT = T*T;
    
    //#pragma omp parallel for schedule(dynamic, 1)
    for( size_t i = 0 ; i < segs.size() ; i++ )
    {
        if( segs[i]->empty() == true )
            continue;
        
        std::vector<int> count(1000, 0);
        int max = -1000;
        int max_id = -1;
        for( pcl::PointCloud<PointT>::const_iterator it = segs[i]->begin(); it < segs[i]->end() ; it++ )
        {
            std::vector<int> indices(1);
            std::vector<float> sqr_distances(1);
            PointLT tmp_pt;
            tmp_pt.x = it->x;
            tmp_pt.y = it->y;
            tmp_pt.z = it->z;
            
            int nres = tree.nearestKSearch(tmp_pt, 1, indices, sqr_distances);
            int cur_label = gt_cloud->at(indices[0]).label;
            if ( nres >= 1 && sqr_distances[0] <= sqrT )
            {
                count[cur_label]++;
                if( count[cur_label] > max )
                {
                    max = count[cur_label];
                    max_id = cur_label;
                }
            }
            else
            {
                count[0]++;
                if( count[0] > max )
                {
                    max = count[0];
                    max_id = 0;
                }
            }
            //std::cerr << max << " ";
        }
        tmp_labels[i] = max_id;
    }
    
    
    return tmp_labels;
}

int main(int argc, char** argv)
{
    std::string in_path("/home/chi/JHUIT/scene/");
    std::string out_path("../../data_pool/GT_segs/");
    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    
    setObjID(model_name_map);
    std::vector<ModelT> model_set;
    std::vector<std::string> model_names = readMesh(mesh_path, model_set);
    int model_num = model_names.size();
    
    bool view_flag = false;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( view_flag )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
        omp_set_num_threads(1);
    }
    
    std::vector< std::string > prefix_set(1);
    prefix_set[0] = "shelf";
//    prefix_set[2] = "office";
//    prefix_set[1] = "labpod";
//    prefix_set[0] = "barrett";
    
    for( int tt = 0 ; tt < prefix_set.size() ; tt++ )
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
            std::string gt_path(in_path + *scene_it + "/poses/");
            std::string cur_out(out_path + *scene_it + "/");
            boost::filesystem::create_directories(cur_out);
            
            #pragma omp parallel for schedule(dynamic, 1)
            for( int i = 0 ; i <= 99 ; i++ )
            {
                std::stringstream ss;
                ss << i;
                
                std::string filename(cur_path + *scene_it + "_" + ss.str() + ".pcd");
                std::cerr << filename << std::endl;
                
                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
                pcl::io::loadPCDFile(filename, *full_cloud);
                
                std::vector<poseT> cur_gt = readGT(gt_path, ss.str());
                if(cur_gt.empty() == true )
                    continue;
                
                pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
                std::vector<int> idx_ff;
                pcl::removeNaNFromPointCloud(*full_cloud, *tmp_cloud, idx_ff);
                pcl::PassThrough<PointT> pass;
                pass.setInputCloud (tmp_cloud);
                pass.setFilterFieldName ("z");
                pass.setFilterLimits (0.1, 2.0);
                //pass.setFilterLimitsNegative (true);
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                pass.filter (*cloud);
                
                pcl::PointCloud<PointLT>::Ptr gt_cloud = genSeg(cloud, model_set, cur_gt, model_name_map);
                spExt sp_ext(-1);
                sp_ext.LoadPointCloud(cloud);
         
                std::vector<pcl::PointCloud<PointT>::Ptr> segs = sp_ext.getSPCloud(0);
                std::vector<int> segs_labels = spGt(gt_cloud, segs);
                
                pcl::PointCloud<PointLT>::Ptr new_gt_cloud(new pcl::PointCloud<PointLT>());
                for( size_t j = 0 ; j < segs.size() ; j++ )
                {
                    int cur_label = segs_labels[j];
                    for(pcl::PointCloud<PointT>::iterator it = segs[j]->begin(); it < segs[j]->end() ; it++ )
                    {
                        PointLT pt;
                        pt.x = it->x;
                        pt.y = it->y;
                        pt.z = it->z;
                        pt.label = cur_label;
                        new_gt_cloud->push_back(pt);
                    }
                }
                
//                gt_cloud->width = full_cloud->width;
//                gt_cloud->height = full_cloud->height;
//                gt_cloud->is_dense = full_cloud->is_dense;
                
                pcl::io::savePCDFile<PointLT>(cur_out + *scene_it + "_" + ss.str() + "_gtseg.pcd", *new_gt_cloud, true);
                if( view_flag )
                {
                    viewer->addPointCloud(full_cloud, "full_cloud");
                    visualizeLabels(new_gt_cloud, viewer, color_label);
                    viewer->spinOnce(1);
                    viewer->saveScreenshot(cur_out + *scene_it + "_" + ss.str() + "_gtseg.png");
                    viewer->removeAllPointClouds();
                }
            }
        }
    }
    return 1;
} 

