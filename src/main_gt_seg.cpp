#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/features.h"
#include "../include/JHUDataParser.h"

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

//int main(int argc, char** argv)
//{
//    std::string in_path("/home/chi/JHUIT/scene/");
//    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
//    std::string out_path("tmp/");
//    std::string shot_path("JHU_kmeans_dict/");
//    std::string sift_path("JHU_fpfh_dict/");
//    pcl::console::parse_argument(argc, argv, "--p", in_path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    boost::filesystem::create_directories(out_path);
//    
///***************************************************************************************************************/
//    
//    setObjID(model_name_map);
//    std::vector<ModelT> model_set;
//    std::vector<std::string> model_names = readMesh(mesh_path, model_set);
//    int model_num = model_names.size();
//    
///***************************************************************************************************************/
//
//    float radius = 0.02;
//    float down_ss = 0.005;
//    float ratio = 0.1;
//    int box_num = 1;
//    pcl::console::parse_argument(argc, argv, "--nn", box_num);
//    pcl::console::parse_argument(argc, argv, "--rd", radius);
//    pcl::console::parse_argument(argc, argv, "--rt", ratio);
//    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
//    std::cerr << "Ratio: " << ratio << std::endl;
//    std::cerr << "Downsample: " << down_ss << std::endl;
//
//    
//    int fea_dim = -1;
//    Hier_Pooler hie_producer(radius);
//    hie_producer.LoadDict_L0(shot_path, "200", "200");
//    hie_producer.setRatio(ratio);
//   
//    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set(1+1);
//    for( size_t i = 1 ; i < fpfh_pooler_set.size() ; i++ )
//    {
//        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
//        fpfh_pooler_set[i] = cur_pooler;
//    }
//    fpfh_pooler_set[1]->LoadSeedsPool(sift_path+"dict_fpfh_L0_200.cvmat");
////    fpfh_pooler_set[2]->LoadSeedsPool(sift_path+"dict_fpfh_L0_50.cvmat");
////    fpfh_pooler_set[3]->LoadSeedsPool(sift_path+"dict_fpfh_L0_100.cvmat");
////    fpfh_pooler_set[4]->LoadSeedsPool(sift_path+"dict_fpfh_L0_200.cvmat");
//
//    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
//    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
//    {
//        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
//        cur_pooler->setHSIPoolingParams(i);
//        lab_pooler_set[i] = cur_pooler;
//    }
//    
///***************************************************************************************************************/  
//    for( int i = 0 ; i <= 9  ; i++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readJHUInstWithImg("/home/chi/JHUIT/ht10/", train_objects, test_objects, i, i, true);
//        std::cerr << "Loading Completed... " << std::endl;
//        
//        int train_num = train_objects[0].size();
//        std::cerr << "Train " << i << " --- " << train_num << std::endl;
//        
//        if( train_num > 0 )
//        {
//            std::vector< sparseVec> final_train;
//            #pragma omp parallel for schedule(dynamic, 1)
//            for( int j = 0 ; j < train_num ; j++ )
//            {
//                pcl::PointCloud<PointT>::Ptr mycloud = train_objects[0][j].cloud;
//            	pcl::PointCloud<NormalT>::Ptr mycloud_normals(new pcl::PointCloud<NormalT>());
//            	computeNormals(mycloud, mycloud_normals, radius);
//		MulInfoT tmp_data = convertPCD(mycloud, mycloud_normals);
//              
//                pcl::VoxelGrid<PointT> sor;
//                sor.setInputCloud(tmp_data.cloud);
//                sor.setLeafSize(down_ss, down_ss, down_ss);
//                sor.filter(*tmp_data.down_cloud);
//                PreCloud(tmp_data, -1, false);
//                
//                std::vector<cv::Mat> main_fea = hie_producer.getHierFea(tmp_data, 0);
//                cv::Mat lab_fea = multiPool(lab_pooler_set, tmp_data, main_fea);
//		cv::Mat fpfh_fea = multiFPFHPool(fpfh_pooler_set, tmp_data, main_fea, radius);
//                
//		cv::Mat cur_final;
//		cv::hconcat(fpfh_fea, lab_fea, cur_final);
//
//                if( fea_dim > 0 && cur_final.cols != fea_dim )
//                {
//                    std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << cur_final.cols << std::endl;
//                    exit(0);
//                }
//                else if( fea_dim < 0 )
//                {
//                    #pragma omp critical
//                    {
//                        fea_dim = cur_final.cols;
//                        std::cerr << "Fea Dim: " << fea_dim << std::endl;
//                    }
//                }
//                 
//                std::vector< sparseVec> this_sparse;
//                sparseCvMat(cur_final, this_sparse);
//                #pragma omp critical
//                {
//                    final_train.push_back(this_sparse[0]);
//                }
//                
//            }
//            std::stringstream ss;
//            ss << i+1;
//        
//            saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train, fea_dim);
//            final_train.clear();
//        }
//        train_objects.clear();
//    }
//    
///***************************************************************************************************************/
//    
//    std::vector< std::string > prefix_set(3);
//    prefix_set[0] = "office";
//    prefix_set[1] = "labpod";
//    prefix_set[2] = "barrett";
//    
//    for( int tt = 0 ; tt <= 2 ; tt++ )
//    {
//        std::vector< std::vector<sparseVec> > final_test(model_num+1);
//        std::vector<std::string> scene_names;
//        
//        for( int i = 0 ; i < 10 ; i++ )
//        {
//            std::stringstream ss;
//            ss << i;
//            scene_names.push_back(std::string (prefix_set[tt] +"_"+ ss.str()));
//        }
//
//        for( std::vector<std::string>::iterator scene_it = scene_names.begin() ; scene_it < scene_names.end() ; scene_it++ )
//        {
//            std::string cur_path(in_path + *scene_it + "/");
//            std::string gt_path(in_path + *scene_it + "/poses/");
//
//            #pragma omp parallel for schedule(dynamic, 1)
//            for( int i = 0 ; i <= 99 ; i++ )
//            {
//                pcl::VoxelGrid<PointT> sor;
//                
//                std::stringstream ss;
//                ss << i;
//
//                std::string filename(cur_path + *scene_it + "_" + ss.str() + ".pcd");
//                std::cerr << filename << std::endl;
//
//                if( exists_test(filename) == false )//|| exists_test(filename_n) == false )
//                {
//                    pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
//                    continue;
//                }
//                pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
//                pcl::io::loadPCDFile(filename, *full_cloud);
//                
//                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//                std::vector<int> idx_ff;
//                pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);
//                
//                pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
//                computeNormals(cloud, cloud_normals, radius);
//
//                std::vector<poseT> cur_gt = readGT(gt_path, ss.str());
//                if(cur_gt.empty() == true )
//                    continue;
// 
//                pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
//                if( down_ss > 0 )
//                {
//                    sor.setInputCloud(cloud);
//                    sor.setLeafSize(down_ss, down_ss, down_ss);
//                    sor.filter(*down_cloud);
//                }
//                else
//                    down_cloud = cloud;
//
//                pcl::PointCloud<PointT>::Ptr all_gt_cloud = genSeg_all(down_cloud, model_set, cur_gt, model_name_map);
//                
//                std::vector<pcl::PointCloud<PointT>::Ptr> gt_segs(model_num + 1);
//                for( size_t kk = 0 ; kk < gt_segs.size() ; kk++ )
//                    gt_segs[kk] = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
//                for( size_t kk = 0 ; kk < all_gt_cloud->size(); kk++ )
//                    if( all_gt_cloud->at(kk).rgba > 0 )
//                        gt_segs[all_gt_cloud->at(kk).rgba]->push_back(down_cloud->at(kk));
//                
//                for( size_t j = 0 ; j < gt_segs.size() ; j++ )
//                {
//                    int cur_label = j;
//                    if( gt_segs[j]->empty() == false && cur_label > 0 )
//                    {
//                        MulInfoT cur_data = convertPCD(cloud, cloud_normals);
//                        cur_data.down_cloud = gt_segs[j];
//                        PreCloud(cur_data, -1, false);
//
//                        std::vector<cv::Mat> main_fea = hie_producer.getHierFea(cur_data, 0);
//                        cv::Mat lab_fea = multiPool(lab_pooler_set, cur_data, main_fea);
//                        cv::Mat fpfh_fea = multiFPFHPool(fpfh_pooler_set, cur_data, main_fea, radius);
//
//                        cv::Mat cur_final;
//                        cv::hconcat(fpfh_fea, lab_fea, cur_final);
//                        
//                        if( fea_dim > 0 && cur_final.cols != fea_dim )
//                        {
//                            std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << cur_final.cols << std::endl;
//                            exit(0);
//                        }
//                        else if( fea_dim < 0 )
//                            fea_dim = cur_final.cols;
//                        std::vector< sparseVec> this_sparse;
//                        sparseCvMat(cur_final, this_sparse);
//                        //#pragma omp critical
//                        //{
//			//    if( counter % 2 == 0 )
//			//	my_train[cur_label].push_back(this_sparse[0]);
//			//    else
//                        //    	final_test[cur_label].push_back(this_sparse[0]);
//                        //}
//                        #pragma omp critical
//                        {
//                            final_test[cur_label].push_back(this_sparse[0]);
//                        }
//                    }
//                }
//                
//            }
//        }
//
//        std::stringstream tt_str;
//        tt_str << tt;
//        for( size_t i = 0 ; i < final_test.size() ; i++ )
//        {
//            std::stringstream ss;
//            ss << i;
//            std::cerr << "Saving Test " << i << " --- " << final_test[i].size() << std::endl;
//            if( final_test[i].empty() == false )
//                saveCvMatSparse(out_path + "test_"+ss.str()+"_L"+tt_str.str()+".smat", final_test[i], fea_dim);
//            final_test[i].clear();
//        }
//    }
//    std::cerr << "Fea_dim: " << fea_dim << std::endl;
//   //*/ 
//  return 1;
//} 
