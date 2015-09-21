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

pcl::PointCloud<PointT>::Ptr genSeg_all(const pcl::PointCloud<PointT>::Ptr scene, const std::vector<ModelT> &model_set, const std::vector<poseT> &gt_poses, std::map<std::string, int> &model_map)
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

std::vector<cv::Mat> SIFTPooling(const MulInfoT &data, const std::vector<cv::SiftFeatureDetector*> &sift_det_vec, cv::SiftDescriptorExtractor * sift_ext, 
                    Hier_Pooler &hie_producer, const std::vector< boost::shared_ptr<Pooler_L0> > &pooler_set, const cv::Mat &atlas, int MODEL_MAX = 100)
{
    // should prepare data with img and map2d
    cv::Mat cur_rgb = data.img;
    cv::Mat cur_map2d = data.map2d;
    cv::Mat cur_gray(cur_rgb.size(), CV_8UC1);
    cv::cvtColor(cur_rgb, cur_gray, CV_BGR2GRAY);

    std::vector<cv::KeyPoint> sift_keys = extSIFTKeys(cur_rgb, sift_det_vec);
//    for( size_t i = 0 ; i < sift_det_vec.size() ; i++ )
//    {
//        std::vector<cv::KeyPoint> tmp_keys;
//	sift_det_vec[i]->detect(cur_gray, tmp_keys);
//	sift_keys.insert(sift_keys.end(), tmp_keys.begin(), tmp_keys.end());
//	//std::cerr << sift_keys.size() << std::endl;
//    }    
    
    std::vector<MulInfoT> data_set(MODEL_MAX+1);
    std::vector< std::vector<cv::KeyPoint> > siftKey_set(MODEL_MAX+1);
    std::vector<int> active_idx;
    std::vector<bool> active_flag(MODEL_MAX+1, false);
    
    for( size_t k = 0 ; k < sift_keys.size() ; k++ )
    {
        int row = round(sift_keys[k].pt.y);
        int col = round(sift_keys[k].pt.x);
        
        int tmp_idx = cur_map2d.at<int>(row, col);
        int label = atlas.at<int>(row, col);
        if( tmp_idx >= 0 && label > 0 )
        {
            if( active_flag[label] == false )
            {
                data_set[label]= convertPCD(data.cloud, data.cloud_normals);
                
                active_idx.push_back(label);
                active_flag[label] = true;
            }
            data_set[label].down_cloud->push_back(data_set[label].cloud->at(tmp_idx));
            siftKey_set[label].push_back(sift_keys[k]);
        }
    }    
    std::vector<cv::Mat> sift_pool_fea(MODEL_MAX+1);
    for( size_t k = 0 ; k < active_idx.size() ; k++ )
    {
//        std::cerr << " Filtered Keypoints: " << siftKey_set[active_idx[k]].size() << " " << data_set[active_idx[k]].down_cloud->size() << std::endl;
    
        cv::Mat cur_sift_descr;
        sift_ext->compute(cur_gray, siftKey_set[active_idx[k]], cur_sift_descr);
        for(int r = 0 ; r < cur_sift_descr.rows ; r++ )
        {
            cv::normalize(cur_sift_descr.row(r), cur_sift_descr.row(r));
//            std::cerr << cur_sift_descr.row(k) << std::endl;
//            std::cin.get();
        }
        std::vector<cv::Mat> main_fea = hie_producer.getHierFea(data_set[active_idx[k]], 0);
        //std::cerr << main_fea[0].rows << " " << main_fea[0].cols << " " << cur_sift_descr.rows<< std::endl;
	//std::cin.get();
	std::vector<cv::Mat> pool_fea_vec;
	for( size_t j = 1 ; j < pooler_set.size() ; j++ )
    	{
            cv::Mat cur_final1 = pooler_set[j]->PoolOneDomain(cur_sift_descr, main_fea[0], 2, false);
            cv::Mat cur_final2 = pooler_set[j]->PoolOneDomain(cur_sift_descr, main_fea[1], 2, false);
            pool_fea_vec.push_back(cur_final1);
            pool_fea_vec.push_back(cur_final2);
    	}
	cv::hconcat(pool_fea_vec, sift_pool_fea[active_idx[k]]);
        //cv::Mat cur_final1 = genericPooler.PoolOneDomain(cur_sift_descr, main_fea[0], 2, false);
        //cv::Mat cur_final2 = genericPooler.PoolOneDomain(cur_sift_descr, main_fea[1], 2, false);
        //cv::hconcat(cur_final1, cur_final2, sift_pool_fea[active_idx[k]]);
    }
    
    return sift_pool_fea;
}

int main(int argc, char** argv)
{
    std::string in_path("/home/cli53/JHUIT_scene/");
//    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
    std::string mesh_path("/home/cli53/mesh/");
    std::string out_path("tmp/");
    std::string shot_path("JHU_kmeans_dict/");
    std::string sift_path("JHU_sift_dict09/");
    //std::string sift_path("JHU_densesift_dict/");
    std::string fpfh_path("JHU_fpfh_dict/");
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    boost::filesystem::create_directories(out_path);
    
/***************************************************************************************************************/
    bool view_flag = false;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
    {
        omp_set_num_threads(1);
        view_flag = true;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( view_flag == true )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
    }
/***************************************************************************************************************/
    
    setObjID(model_name_map);
    std::vector<ModelT> model_set;
    std::vector<std::string> model_names = readMesh(mesh_path, model_set);
    int model_num = model_names.size();
    
/***************************************************************************************************************/

    float radius = 0.02;
    float down_ss = 0.005;
    float ratio = 0.1;
    int box_num = 1;
    //float sigma = 0.9;
    pcl::console::parse_argument(argc, argv, "--nn", box_num);
    pcl::console::parse_argument(argc, argv, "--rd", radius);
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    //pcl::console::parse_argument(argc, argv, "--sigma", sigma);
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;

    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
    for( float sigma = 0.9 ; sigma <= 1.6 ; sigma += 0.1 )
    {	
        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
        	0, // nFeatures
        	4, // nOctaveLayers
        	-10000, // contrastThreshold 
        	100000, //edgeThreshold
        	sigma//sigma
        	);
        sift_det_vec.push_back(sift_det);	
    }
    cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
    
    int fea_dim = -1;
    Hier_Pooler hie_producer(radius);
    hie_producer.LoadDict_L0(shot_path, "200", "200");
    hie_producer.setRatio(ratio);
   
    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set(1+1);
    for( size_t i = 1 ; i < sift_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
        sift_pooler_set[i] = cur_pooler;
    }
    //sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_25.cvmat");
    //sift_pooler_set[2]->LoadSeedsPool(sift_path+"dict_sift_L0_50.cvmat");
    //sift_pooler_set[3]->LoadSeedsPool(sift_path+"dict_sift_L0_100.cvmat");
    //sift_pooler_set[4]->LoadSeedsPool(sift_path+"dict_sift_L0_200.cvmat");
    sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_100.cvmat"); 

    std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        lab_pooler_set[i] = cur_pooler;
    }

    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set(1+1);
    for( size_t i = 1 ; i < fpfh_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
        fpfh_pooler_set[i] = cur_pooler;
    }
    //fpfh_pooler_set[1]->LoadSeedsPool(sift_path+"dict_fpfh_L0_25.cvmat");
    //fpfh_pooler_set[2]->LoadSeedsPool(sift_path+"dict_fpfh_L0_50.cvmat");
    //fpfh_pooler_set[3]->LoadSeedsPool(sift_path+"dict_fpfh_L0_100.cvmat");
    //fpfh_pooler_set[4]->LoadSeedsPool(sift_path+"dict_fpfh_L0_200.cvmat");
    fpfh_pooler_set[1]->LoadSeedsPool(fpfh_path+"dict_fpfh_L0_100.cvmat");

    //Pooler_L0 genericPooler(-1);
    //genericPooler.LoadSeedsPool(sift_path+"dict_sift_L0_200.cvmat");
    
/***************************************************************************************************************/  
    for( int i = 0 ; i <= 9  ; i++ )
    {
        ObjectSet train_objects, test_objects;
        readJHUInstWithImg("/home/cli53/filtered_pcd/", train_objects, test_objects, i, i, true);
        std::cerr << "Loading Completed... " << std::endl;
        
        int train_num = train_objects[0].size();
        std::cerr << "Train " << i << " --- " << train_num << std::endl;
        
        if( train_num > 0 )
        {
            std::vector< sparseVec> final_train;
            #pragma omp parallel for schedule(dynamic, 1)
            for( int j = 0 ; j < train_num ; j++ )
            {
                pcl::PointCloud<PointT>::Ptr mycloud = train_objects[0][j].cloud;
            	pcl::PointCloud<NormalT>::Ptr mycloud_normals(new pcl::PointCloud<NormalT>());
            	computeNormals(mycloud, mycloud_normals, radius);
		MulInfoT tmp_data = convertPCD(mycloud, mycloud_normals);
                tmp_data.img = train_objects[0][j].img;
                tmp_data.map2d = train_objects[0][j].map2d;
                tmp_data._3d2d = train_objects[0][j]._3d2d;
//                if( view_flag == true )
//                {
//                    viewer->addPointCloud(tmp_data.cloud, "cloud");
//                    viewer->addPointCloud(tmp_data.down_cloud, "keys");
//                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "keys");
//                    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keys");
//                    viewer->spin();
//                }
              
                cv::Mat cur_atlas = cv::Mat::zeros(tmp_data.img.rows, tmp_data.img.cols, CV_32SC1);
                for(int k = 0 ; k < tmp_data._3d2d.rows ; k++ )
                    cur_atlas.at<int>(tmp_data._3d2d.at<int>(k, 1), tmp_data._3d2d.at<int>(k, 0) ) = 1;
                
                std::vector<cv::Mat> cur_final_vec = SIFTPooling(tmp_data, sift_det_vec, sift_ext, hie_producer, sift_pooler_set, cur_atlas, 1);
		cv::Mat ext_fea;
		{
         	    pcl::VoxelGrid<PointT> sor;
                    sor.setInputCloud(tmp_data.cloud);
                    sor.setLeafSize(down_ss, down_ss, down_ss);
                    sor.filter(*tmp_data.down_cloud);
                    PreCloud(tmp_data, -1, false);
                    std::vector<cv::Mat> main_fea = hie_producer.getHierFea(tmp_data, 0);
                    
                    //std::vector<cv::Mat> cur_final_vec = SIFTPooling_new(tmp_data, main_fea, sift_ext, sift_pooler_set, cur_atlas, 1);
                    
		    cv::Mat lab_fea = multiPool(lab_pooler_set, tmp_data, main_fea);
		    cv::Mat fpfh_fea = multiFPFHPool(fpfh_pooler_set, tmp_data, main_fea, radius);
		    cv::hconcat(fpfh_fea, lab_fea, ext_fea);
		}
		cv::Mat cur_final;
		cv::hconcat(cur_final_vec[1], ext_fea, cur_final);
                //cv::Mat cur_final = cur_final_vec[1];
//                std::cerr << cur_final.cols << std::endl;
//                cv::Mat cur_final = multiPool(pooler_set, tmp_data, main_fea);
                if( fea_dim > 0 && cur_final.cols != fea_dim )
                {
                    std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << cur_final.cols << std::endl;
                    exit(0);
                }
                else if( fea_dim < 0 )
		{
		#pragma omp critical
		{
                    fea_dim = cur_final.cols;
		    std::cerr << "Fea Dim: " << fea_dim << std::endl;
		}
		}	
                std::vector< sparseVec> this_sparse;
                sparseCvMat(cur_final, this_sparse);
                #pragma omp critical
                {
                    final_train.push_back(this_sparse[0]);
                }
                
            }
            std::stringstream ss;
            ss << i+1;
        
            saveCvMatSparse(out_path + "train_"+ss.str()+"_L0.smat", final_train, fea_dim);
            final_train.clear();
        }
        train_objects.clear();
    }
    
/***************************************************************************************************************/
    
    std::vector< std::string > prefix_set(3);
    prefix_set[0] = "office";
    prefix_set[1] = "labpod";
    prefix_set[2] = "barrett";
    
    for( int tt = 0 ; tt <= 2 ; tt++ )
    {
        std::vector< std::vector<sparseVec> > final_test(model_num+1);
        std::vector<std::string> scene_names;
        
        for( int i = 0 ; i < 10 ; i++ )
        {
            std::stringstream ss;
            ss << i;
            scene_names.push_back(std::string (prefix_set[tt] +"_"+ ss.str()));
        }

        for( std::vector<std::string>::iterator scene_it = scene_names.begin() ; scene_it < scene_names.end() ; scene_it++ )
        {
//            std::string cur_path(in_path + *scene_it + "/for_recog/");
            std::string cur_path(in_path + *scene_it + "/");
            std::string gt_path(in_path + *scene_it + "/poses/");

//            #pragma omp parallel for schedule(dynamic, 1)
            for( int i = 0 ; i <= 99 ; i++ )
            {
                pcl::VoxelGrid<PointT> sor;
                
                std::stringstream ss;
                ss << i;

                std::string filename(cur_path + *scene_it + "_" + ss.str() + ".pcd");
                std::cerr << filename << std::endl;

                if( exists_test(filename) == false )//|| exists_test(filename_n) == false )
                {
                    pcl::console::print_warn("Failed to Read: %s\n", filename.c_str());
                    continue;
                }
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
                pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
                pcl::io::loadPCDFile(filename, *cloud);
                //pcl::io::loadPCDFile(filename_n, *cloud_normals);
            	computeNormals(cloud, cloud_normals, radius);

                std::vector<poseT> cur_gt = readGT(gt_path, ss.str());
                if(cur_gt.empty() == true )
                    continue;
                
                pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
                if( down_ss > 0 )
                {
                    sor.setInputCloud(cloud);
                    sor.setLeafSize(down_ss, down_ss, down_ss);
                    sor.filter(*down_cloud);
                }
                else
                    down_cloud = cloud;

                pcl::PointCloud<PointT>::Ptr all_gt_cloud = genSeg_all(cloud, model_set, cur_gt, model_name_map);
                pcl::PointCloud<PointT>::Ptr down_gt_cloud = genSeg_all(down_cloud, model_set, cur_gt, model_name_map);
                MulInfoT cur_data = convertPCD(cloud, cloud_normals);
//                cur_data.img = cv::imread(cur_path + *scene_it + "_" + ss.str() + ".png");
                cur_data.img = getFullImage(cloud);
                cur_data.map2d = cv::Mat::ones(cur_data.img.rows, cur_data.img.cols, CV_32SC1) * -1;
                cv::Mat cur_atlas = cv::Mat::ones(cur_data.img.rows, cur_data.img.cols, CV_32SC1) * -1;
                //cv::Mat view_mat = cur_data.img.clone();
                
		std::vector<pcl::PointCloud<PointT>::Ptr> gt_segs(model_num + 1);
                for( size_t kk = 0 ; kk < gt_segs.size() ; kk++ )
                    gt_segs[kk] = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
                for( size_t kk = 0 ; kk < down_gt_cloud->size(); kk++ )
                    if( down_gt_cloud->at(kk).rgba > 0 )
                        gt_segs[down_gt_cloud->at(kk).rgba]->push_back(down_cloud->at(kk));

                int pt_count = 0;
                int max_model_id = -1;
                for( pcl::PointCloud<PointT>::iterator it = all_gt_cloud->begin() ; it < all_gt_cloud->end() ; it++, pt_count++ )
                {
                    if( it->rgba > 0 )
                    {
//                        int img_x = round(it->x / it->z * this_fx + center_x);
//                        int img_y = round(it->y / it->z * this_fy + center_y);
                        int img_x = pt_count % 640;
                        int img_y = pt_count / 640;
                        cur_data.map2d.at<int>(img_y, img_x) = pt_count;
                        cur_atlas.at<int>(img_y, img_x) = it->rgba;
//                        view_mat.at<uchar>(img_y, img_x*3) = 128;
//                        view_mat.at<uchar>(img_y, img_x*3+1) = 128;
//                        view_mat.at<uchar>(img_y, img_x*3+2) = 128;
                        if( (int)it->rgba > max_model_id )
                            max_model_id = (int)it->rgba;
                    }
                }
//                cv::imshow("View", view_mat);
//                cv::waitKey();
                
                std::vector<cv::Mat> cur_final_vec = SIFTPooling(cur_data, sift_det_vec, sift_ext, hie_producer, sift_pooler_set, cur_atlas, max_model_id);
                
		for( size_t j = 0 ; j < cur_final_vec.size() ; j++ )
                {
                    int cur_label = j;
                    //if( cur_final_vec[j].empty() == false && cur_label > 0 )
//                cv::Mat tmp_atlas = cv::Mat::ones(cur_data.img.rows, cur_data.img.cols, CV_32SC1);
//                for( int j = 0 ; j < max_model_id+1 ; j++ )
//                {
//                    int cur_label = j;
//                    if( gt_segs[j]->empty() == false && cur_label > 0 ) 
                    if( cur_final_vec[j].empty() == false && cur_label > 0 )
		    {
                        cv::Mat sift_final = cur_final_vec[j];
                     
			MulInfoT this_data = convertPCD(cloud, cloud_normals);
			this_data.img = cur_data.img;
                        this_data.down_cloud = gt_segs[j];
                        PreCloud(this_data, -1, false);

                        std::vector<cv::Mat> cur_local_fea = hie_producer.getHierFea(this_data, 0);
                        
                        //std::vector<cv::Mat> cur_final_vec = SIFTPooling_new(this_data, cur_local_fea, sift_ext, sift_pooler_set, tmp_atlas, 1);
                        //cv::Mat sift_final = cur_final_vec[1];

                        cv::Mat lab_final = multiPool(lab_pooler_set, this_data, cur_local_fea);
		    	cv::Mat fpfh_final = multiFPFHPool(fpfh_pooler_set, this_data, cur_local_fea, radius);
		    	cv::Mat ext_final;
			cv::hconcat(fpfh_final, lab_final, ext_final);
			
			cv::Mat cur_final;
			cv::hconcat(sift_final, ext_final, cur_final);
		        if( fea_dim > 0 && cur_final.cols != fea_dim )
                        {
                            std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << cur_final.cols << std::endl;
                            exit(0);
                        }
                        else if( fea_dim < 0 )
                            fea_dim = cur_final.cols;
                        std::vector< sparseVec> this_sparse;
                        sparseCvMat(cur_final, this_sparse);
                        #pragma omp critical
                        {
                            final_test[cur_label].push_back(this_sparse[0]);
                        }
                    }
                }

            }
        }

        std::stringstream tt_str;
        tt_str << tt;
        for( size_t i = 0 ; i < final_test.size() ; i++ )
        {
            std::stringstream ss;
            ss << i;
            std::cerr << "Saving Test " << i << " --- " << final_test[i].size() << std::endl;
            if( final_test[i].empty() == false )
                saveCvMatSparse(out_path + "test_"+ss.str()+"_L"+tt_str.str()+".smat", final_test[i], fea_dim);
            final_test[i].clear();
        }
    }
    std::cerr << "Fea_dim: " << fea_dim << std::endl;
   //*/ 
  return 1;
} 

//int main(int argc, char** argv)
//{
//    std::string in_path("/home/chi/JHUIT/scene/");
//    std::string mesh_path("/home/chi/devel_mode/ObjRecRANSAC/data/mesh/");
//    std::string out_path("../../data_pool/whole_segs/");
//    std::string dict_path("JHU_dict/");
//    pcl::console::parse_argument(argc, argv, "--p", in_path);
//    pcl::console::parse_argument(argc, argv, "--o", out_path);
//    boost::filesystem::create_directories(out_path);
//    
////    int c1 = 0, c2 = 99;
////    pcl::console::parse_argument(argc, argv, "--c1", c1);
////    pcl::console::parse_argument(argc, argv, "--c2", c2);
//    
///***************************************************************************************************************/
////    bool view_flag = false;
////    if( pcl::console::find_switch(argc, argv, "-v") == true )
////    {
////        omp_set_num_threads(1);
////        view_flag = true;
////    }
////    pcl::visualization::PCLVisualizer::Ptr viewer;
////    if( view_flag == true )
////    {
////        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
////        viewer->initCameraParameters();
////        viewer->addCoordinateSystem(0.1);
////        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
////        viewer->setSize(1280, 960);
////    }
///***************************************************************************************************************/
//    
////    setObjID(model_name_map);
////    std::vector<ModelT> model_set;
////    std::vector<std::string> model_names = readMesh(mesh_path, model_set);
////    int model_num = model_names.size();
//    
///***************************************************************************************************************/
//
////    float radius = 0.03;
////    float down_ss = 0.005;
////    float ratio = 0;
////    int layer = 5;
////    int box_num = 30;
////    pcl::console::parse_argument(argc, argv, "--nn", box_num);
////    pcl::console::parse_argument(argc, argv, "--rd", radius);
////    pcl::console::parse_argument(argc, argv, "--rt", ratio);
////    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
////    pcl::console::parse_argument(argc, argv, "--ll", layer);
//    
////    int fea_dim = -1;
////    Hier_Pooler hie_producer(radius);
////    hie_producer.LoadDict_L0(dict_path, "200", "200");
////    hie_producer.setRatio(ratio);
////    
////    std::vector< boost::shared_ptr<Pooler_L0> > pooler_set(layer+1);
////    for( size_t i = 1 ; i < pooler_set.size() ; i++ )
////    {
////        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
////        cur_pooler->setHSIPoolingParams(i);
////        pooler_set[i] = cur_pooler;
////    }
////    Pooler_L0 genericPooler(-1);
////    genericPooler.LoadSeedsPool(dict_path+"dict_depth_L0_200.cvmat");
//    
////    std::cerr << "Ratio: " << ratio << std::endl;
////    std::cerr << "Downsample: " << down_ss << std::endl;
//
///***************************************************************************************************************  
//    for( int i = 0 ; i <= 9  ; i++ )
//    {
//        ObjectSet train_objects, test_objects;
//        readJHUInst("/home/chi/JHUIT/filtered_pcd/", train_objects, test_objects, i, i);
//        std::cerr << "Loading Completed... " << std::endl;
//        
//        train_objects[0].insert(train_objects[0].end(), test_objects[0].begin(), test_objects[0].end());
//        test_objects.clear();
//        
//        int train_num = train_objects[0].size();
//        std::cerr << "Train " << i << " --- " << train_num << std::endl;
//        
//        if( train_num > 0 )
//        {
//            std::vector< sparseVec> final_train;
//            //#pragma omp parallel for schedule(dynamic, 1)
//            for( int j = 0 ; j < train_num ; j++ )
//            {
//                pcl::VoxelGrid<PointT> sor;
//                pcl::ExtractIndices<PointT> ext;
//                ext.setNegative(false);
//                
//		pcl::PointCloud<PointT>::Ptr mycloud = train_objects[0][j].cloud;
//            	pcl::PointCloud<NormalT>::Ptr mycloud_normals(new pcl::PointCloud<NormalT>());
//            	computeNormals(mycloud, mycloud_normals, radius);
//		MulInfoT tmp_data = convertPCD(mycloud, mycloud_normals);		
//
//                //MulInfoT *inst_ptr = &train_objects[0][j];
//                //MulInfoT tmp_data = convertPCD(inst_ptr->cloud, inst_ptr->cloud_normals);
//                if( down_ss > 0 )
//                {
//                    sor.setInputCloud(tmp_data.cloud);
//                    sor.setLeafSize(down_ss, down_ss, down_ss);
//                    sor.filter(*tmp_data.down_cloud);
//                }
//                else
//                    tmp_data.down_cloud = tmp_data.cloud;
//                PreCloud(tmp_data, -1, false);
//                std::vector<cv::Mat> main_fea = hie_producer.getHierFea(tmp_data, 0);
//                
//                if( box_num > 1 )
//                {
//                    std::vector<pcl::PointIndices::Ptr> inlier_set3 = CropSegs(tmp_data, tmp_data.down_cloud->size()*0.1, tmp_data.down_cloud->size()*0.3, 30);
//                    std::vector<pcl::PointIndices::Ptr> inlier_set1 = CropSegs(tmp_data, tmp_data.down_cloud->size()*0.3, tmp_data.down_cloud->size()*0.5, 30);
//                    std::vector<pcl::PointIndices::Ptr> inlier_set2 = CropSegs(tmp_data, tmp_data.down_cloud->size()*0.5, tmp_data.down_cloud->size()*0.8, 30);
//                    std::vector<pcl::PointIndices::Ptr> inlier_set;// = CropSegs(tmp_data, tmp_data.down_cloud->size(), tmp_data.down_cloud->size(), 1);
//		    inlier_set.insert(inlier_set.end(), inlier_set1.begin(), inlier_set1.end());
//		    inlier_set.insert(inlier_set.end(), inlier_set2.begin(), inlier_set2.end());
//                    inlier_set.insert(inlier_set.end(), inlier_set3.begin(), inlier_set3.end());
//                    if( inlier_set.empty() == true )
//                        continue;
//
//                    std::vector< sparseVec> tmp_final;
//                    for(std::vector<pcl::PointIndices::Ptr>::iterator it_inlier = inlier_set.begin() ; it_inlier < inlier_set.end() ; it_inlier++ )
//                    {
//                        std::cerr << (*it_inlier)->indices.size() << " ";
//                        continue;
//                        pcl::PointCloud<PointT>::Ptr cur_seg(new pcl::PointCloud<PointT>());
//                        // Extract the inliers
//                        ext.setInputCloud (tmp_data.down_cloud);
//                        ext.setIndices (*it_inlier);
//                        ext.filter (*cur_seg);
//                        MulInfoT cur_data = convertPCD(tmp_data.cloud, tmp_data.cloud_normals);
//                        cur_data.down_cloud = cur_seg;
//
//                        PreCloud(cur_data, -1, false);
//                        std::vector<cv::Mat> cur_fea = extFea(main_fea, (*it_inlier)->indices);
//                        
//                        //cv::Mat final_temp = genericPooler.PoolOneDomain(cur_fea[0], cur_fea[1], 2, false);
//                        cv::Mat final_temp = multiPool(pooler_set, cur_data, cur_fea);
//
//                        if( fea_dim > 0 && final_temp.cols != fea_dim )
//                        {
//                            std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << final_temp.cols << std::endl;
//                            exit(0);
//                        }
//                        else if( fea_dim < 0 )
//                            fea_dim = final_temp.cols;
//                        std::vector< sparseVec> final_sparse;
//                        sparseCvMat(final_temp, final_sparse);
//                        tmp_final.push_back(final_sparse[0]);
//                    }
//                    std::cerr << std::endl;
//                    std::cin.get();
//                    #pragma omp critical
//                    {
//                        final_train.insert(final_train.end(), tmp_final.begin(), tmp_final.end());
//                    }
//                }
//                else if( box_num == 1 )
//                {
////                    cv::Mat cur_final = genericPooler.PoolOneDomain(main_fea[0], main_fea[1], 2, false);
////                    std::cerr << cur_final << std::endl;
////                    std::cerr << cur_final.cols << std::endl;
////                    std::cin.get();
//                    cv::Mat cur_final = multiPool(pooler_set, tmp_data, main_fea);
//                    if( fea_dim > 0 && cur_final.cols != fea_dim )
//                    {
//                        std::cerr << "Error: fea_dim > 0 && cur_final.cols != fea_dim   " << fea_dim << " " << cur_final.cols << std::endl;
//                        exit(0);
//                    }
//                    else if( fea_dim < 0 )
//                        fea_dim = cur_final.cols;
//
//                    std::vector< sparseVec> this_sparse;
//                    sparseCvMat(cur_final, this_sparse);
//                    #pragma omp critical
//                    {
//                        final_train.push_back(this_sparse[0]);
//                    }
//                }
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
////        std::vector< std::vector<sparseVec> > final_test(model_num+1);
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
//            std::string cur_path(in_path + *scene_it + "/for_recog/");
//            std::string gt_path(in_path + *scene_it + "/poses/");
//
//            for( int i = 0 ; i <= 99 ; i++ )
//            {
//                std::stringstream ss;
//                ss << i;
//                
//                std::cerr << "Loading---" << cur_path + *scene_it + "_" + ss.str() + ".png" << std::endl;
//                cv::Mat cur_img = cv::imread(cur_path + *scene_it + "_" + ss.str() + ".png");
//                cv::imshow("RGB", cur_img);
//                cv::Mat gray(cur_img.size(), CV_8UC1);
//                cv::cvtColor(cur_img, gray, CV_BGR2GRAY);
//                cv::imshow("Gray", gray);
//                
//                cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
//                        0, // nFeatures
//                        4, // nOctaveLayers
//                        -10000, // contrastThreshold 
//                        100000, //edgeThreshold
//                        0.9 //sigma
//                        );
//
//                //cv::DenseFeatureDetector *sift_det = new cv::DenseFeatureDetector(12.0f, 1, 0.1f, 1);
//                
//                cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
//                // Compute keypoints and descriptor from the source image in advance
//                std::vector<cv::KeyPoint> keypoints;
//                cv::Mat descriptors;
//
//                
//                sift_det->detect(gray, keypoints);
//                printf("%d sift keypoints are found.\n", (int)keypoints.size());
//                //*
//                for(int i = 0 ; i < keypoints.size() ; i++ )
//                {
////                    keypoints[i].angle = 0;
////                    std::cerr << keypoints[i].pt.y << " " << keypoints[i].pt.x << " "
////                            << keypoints[i].angle << " " << keypoints[i].class_id << " " 
////                            << keypoints[i].octave <<" " << keypoints[i].size << " " 
////                            << keypoints[i].response << std::endl; 
////                    std::cin.get();
//                    ;
//                }
//                //*/
//                
//                sift_ext->compute(gray, keypoints, descriptors);
////                for(int i = 0 ; i < descriptors.cols ; i++ )
////                {
////                    std::cerr << descriptors.row(i) << std::endl;
////                    std::cin.get();
////                }
//                
//                if( true )
//                {
//                    cv::Mat out_image;
//                    cv::drawKeypoints(gray, keypoints, out_image);
//                    cv::imshow("keypoints", out_image);
//                    
//                }
//                
//                cv::waitKey();
//            }
//        }
//
//        
//    }
//
//    return 1;
//} 


