#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/SIFTpooling.h"

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

                pcl::PointCloud<PointT>::Ptr all_gt_cloud = genSeg(cloud, model_set, cur_gt, model_name_map);
                pcl::PointCloud<PointT>::Ptr down_gt_cloud = genSeg(down_cloud, model_set, cur_gt, model_name_map);
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


