#ifndef SP_SEGMENTER_TRAIN_H
#define SP_SEGMENTER_TRAIN_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"


pcl::PointCloud<PointLT>::Ptr genSeg(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map)
{
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*scene, *scene_xyz);
    
    pcl::search::KdTree<myPointXYZ> tree;
    tree.setInputCloud (scene_xyz);
    
    std::vector<int> obj_labels(scene->size(), -1);
    std::vector<float> obj_dist(scene->size(), 1000);
    float T = 0.01;
    
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
    
pcl::PointCloud<PointLT>::Ptr seg_cloud(new pcl::PointCloud<PointLT>());
    for(size_t i = 0 ; i < scene->size() ; i++ )
    {
        PointLT new_pt;
        new_pt.x = scene->at(i).x;
        new_pt.y = scene->at(i).y;
        new_pt.z = scene->at(i).z;
        if(obj_labels[i] > 0 )
            new_pt.label = obj_labels[i];
        else
            new_pt.label = 0;
        seg_cloud->push_back(new_pt);
    }
    return seg_cloud;
}


void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3])
{
    pcl::PointCloud<PointT>::Ptr view_cloud(new pcl::PointCloud<PointT>());
    for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        
        PointT pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        pt.rgba = colors[it->label][0] << 16 | colors[it->label][1] << 8 | colors[it->label][2];
        view_cloud->push_back(pt);
    }
    
    viewer->addPointCloud(view_cloud, "label_cloud");
    viewer->spinOnce(2000);
    viewer->removePointCloud("label_cloud");
}

pcl::PointCloud<PointLT>::Ptr densifyLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, const pcl::PointCloud<PointT>::Ptr ori_cloud)
{
    pcl::search::KdTree<PointLT> tree;
    tree.setInputCloud(label_cloud);
    
    pcl::PointCloud<PointLT>::Ptr dense_cloud(new pcl::PointCloud<PointLT>());
    
    float T = 0.01;
    float sqrT = T*T;
    for( pcl::PointCloud<PointT>::const_iterator it = ori_cloud->begin() ; it < ori_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        PointLT tmp;
        tmp.x = it->x;
        tmp.y = it->y;
        tmp.z = it->z;
       
        std::vector<int> ind (1);
	std::vector<float> sqr_dist (1);
        int nres = tree.nearestKSearch(tmp, 1, ind, sqr_dist);
        if ( nres >= 1 && sqr_dist[0] <= sqrT )
        {
            tmp.label = label_cloud->at(ind[0]).label;
            dense_cloud->push_back(tmp);
        }   
    }
    return dense_cloud;
}


std::vector<int> spGt(const pcl::PointCloud<PointT>::Ptr gt_cloud, const std::vector<pcl::PointCloud<PointT>::Ptr> segs)
{
    std::vector<int> tmp_labels(segs.size(), 0);
    if( gt_cloud->empty() == true )
        return tmp_labels;
    
    pcl::search::KdTree<PointT> tree;
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
            int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
            int cur_label = gt_cloud->at(indices[0]).rgba;
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
#endif