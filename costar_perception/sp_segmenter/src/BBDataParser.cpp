#include "sp_segmenter/BBDataParser.h"

void readBBTrainALL(std::string path, std::string sub_path, CloudSet &clouds, NormalSet &normals, int c1, int c2)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? BB_INST_MAX - 1 : c2;
    //std::string model_file
    std::ifstream model_list((path + "/models.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    ObjectSet objects;
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path +"/" + class_name + "/" + sub_path);
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            std::string filename(workspace+"/"+*it);
            std::string filename_n(workspace+"/normal_"+*it);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(filename_n, *cloud_normal);
            
            if((*it)[2] == '1' || (*it)[2] == '3' || (*it)[2] == '5' )  
            {
                clouds.push_back(cloud);
                normals.push_back(cloud_normal);
            }
        }
    }
    model_list.close();
}


void readBB(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, bool all)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? BB_INST_MAX - 1 : c2;
    //std::string model_file
    std::ifstream model_list((path + "models.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path + class_name + "/n_test/");
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        std::vector<MulInfoT> cur_train, cur_test;
        #pragma omp parallel for schedule(dynamic, 1)
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            std::string filename(workspace+*it);
            std::string filename_n(workspace+"normal_"+*it);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            //pcl::PointCloud<PointT>::Ptr xy_cloud = NormalizeXY(cloud);
            //NormalizeCloud(cloud);
            
            pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(filename_n, *cloud_normal);
            
            MulInfoT temp = convertPCD(cloud, cloud_normal);
            //MulInfoT temp = convertPCD(xy_cloud, cloud_normal);
            
            if( all == true )
            {
                #pragma omp critical
                {
                    //MulInfoT temp = convertPCD(cloud, cloud_normal);
                    cur_train.push_back(temp);
                }
                //if((*it)[2] == '2' || (*it)[2] == '3' || (*it)[2] == '4' )  
                //{
                //    #pragma omp critical
                //    {
                //        cur_train.push_back(temp);
                //    }
                //}
                //if( (*it)[2] == '3' )  
                //{
                //    #pragma omp critical
                //    {
                //        cur_train.push_back(temp);
                //    }
                //}
            }
            else
            {
                if((*it)[2] == '1' || (*it)[2] == '3' || (*it)[2] == '5' )  
                {
                    #pragma omp critical
                    {
                        cur_train.push_back(temp);
                    }
                    //cur_train.push_back(temp);
                }
                else
                {
                    #pragma omp critical
                    {
                        /*
                        float ss = 0.005;
                        pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
                        
                        pcl::VoxelGrid<PointT> sor;
                        sor.setInputCloud(cloud);
                        sor.setLeafSize(ss, ss, ss);
                        sor.filter(*down_cloud);
                        
                        pcl::PointCloud<NormalT>::Ptr down_normal(new pcl::PointCloud<NormalT>());
                        pcl::NormalEstimationOMP<PointT, NormalT> est;
                        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
                        est.setSearchMethod (tree);
                        est.setRadiusSearch(0.03);
                        est.setSearchSurface(cloud);
                        est.setInputCloud (down_cloud);
                        est.compute (*down_normal);
                        
                        MulInfoT temp = convertPCD(down_cloud, down_normal);
                        */
                        cur_test.push_back(temp);
                    }
                }
            }
            
                
        }
        train_set.push_back(cur_train);
        test_set.push_back(cur_test);
    }
    
    model_list.close();
}

float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000, max_z = -1000, min_z = 1000;
void updateBBObject(const pcl::PointCloud<PointT>::Ptr cloud)
{
    for( pcl::PointCloud<PointT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {    
        float x = (*it).x, y = (*it).y, z = (*it).z;
        if( max_x < x ) max_x = x;
        if( max_y < y ) max_y = y;
        if( max_z < z ) max_z = z;
        
        if( min_x > x ) min_x = x;
        if( min_y > y ) min_y = y;
        if( min_z > z ) min_z = z;
    }
}

void BBShiftClouds(std::string in_path, std::string out_path)
{
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(in_path, pcd_files);
    
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(in_path + "/" + (*it), *cloud);
        
        pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
        ComputeCentroid(cloud, center);
        Eigen::Matrix4f toOrigin = Eigen::Matrix4f::Identity();
        toOrigin(0, 3) = -center->at(0).x;
        toOrigin(1, 3) = -center->at(0).y;
        toOrigin(2, 3) = -center->at(0).z;
        
        Eigen::Matrix4f flip = -Eigen::Matrix4f::Identity();
        flip(3,3) = 1.0;
        pcl::transformPointCloud(*cloud, *cloud, flip*toOrigin);
        
        /*
        if( viewer != NULL )
        {
            viewer->removePointCloud("cloud");
            viewer->addPointCloud(cloud, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            viewer->spin();
        }
        */
        pcl::io::savePCDFile(out_path + "/" + *it, *cloud, true);
    }
}

void BBStreamingShift(std::string path, std::string sub_in_path, std::string sub_out_path, int c1, int c2)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? BB_INST_MAX - 1 : c2;
    //std::string model_file
    std::ifstream model_list((path + "/models.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        BBShiftClouds(path + "/" + class_name + "/" + sub_in_path, path + "/" + class_name + "/" + sub_out_path);
    }
    
}

void BBStreamingNormal(std::string path, std::string sub_path, float radius, int c1, int c2)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? BB_INST_MAX - 1 : c2;
    
    std::ifstream model_list((path + "/models.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path +"/" + class_name + "/" + sub_path);
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        int count = 0;
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(workspace + "/" + (*it), *cloud);
            //updateBBObject(cloud);
            
            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());

            pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
            normal_estimation.setSearchMethod (tree);
            normal_estimation.setNumberOfThreads(8);
            normal_estimation.setRadiusSearch(radius);
            normal_estimation.setInputCloud (cloud);
            normal_estimation.setViewPoint(0,0,1);
            normal_estimation.compute (*cloud_normals);
           
            pcl::io::savePCDFile(workspace + "/normal_" + (*it), *cloud_normals, true);
            /*
            if( viewer != NULL )
            {
                viewer->removeAllPointClouds();
                viewer->addPointCloud(cloud, "cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
                std::cerr << count << std::endl;
                //viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 2, 0.02, "cloud_normals");
                std::cerr<<"Min_x: "<<min_x<<", Max_x: "<<max_x<<std::endl;
                std::cerr<<"Min_y: "<<min_y<<", Max_y: "<<max_y<<std::endl;
                std::cerr<<"Min_z: "<<min_z<<", Max_z: "<<max_z<<std::endl;
                count++;
                viewer->spin();
            }
            */
        }
    }

    model_list.close();
}

void BBStreamingSIFT(std::string path, int c1, int c2)
{   
    std::ifstream model_list((path + "/models.txt").c_str(), std::ios::in);
    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < c1 )
            continue;
        else if (m_id > c2)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        std::string sift_path(path+"/"+class_name+"/sift");
        if( exists_dir(sift_path) == false )
            boost::filesystem::create_directories(sift_path);
        
        std::string idx_path(path+"/"+class_name+"/cloud_2D_idx");
        std::string img_path(path+"/"+class_name+"/img");
        
        //searching all jpg images in img path
        boost::filesystem::path p(img_path);
        std::vector< std::string > ret;
        find_files(p, ".jpg", ret);
        std::string prefix("sift");
        
        cv::namedWindow("debug");
        std::vector< std::string >::iterator it;
        for( it = ret.begin() ; it < ret.end() ; it++ )
        {
            cv::Mat img = cv::imread(img_path + "/" + (*it));
            cv::Mat gray;
            cv::cvtColor(img, gray, CV_BGR2GRAY);
            
            std::string actual_name((*it).substr(0, (*it).size()-4));
            cv::Mat idxs;
            readMat(idx_path + "/idx2d_" + actual_name + ".cvmat", idxs);
            //construct keypoint vector
            //cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
            //    0, // nFeatures
            //    4, // nOctaveLayers
            //    -1000000.0, // contrastThreshold 0.04 
            //    100000000, //edgeThreshold 10
            //    1.6 //sigma
            //    );
            //cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
            /*
            //cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
            cv::Mat mask = cv::Mat::ones(img.rows, img.cols, CV_8UC1)*255;
            for( int i = 0 ; i < idxs.rows ; i++ )
            {
                std::vector<cv::KeyPoint> keypoints;
                //mask.at<uchar>(idxs.at<float>(i, 0), idxs.at<float>(i, 1)) = 255;
                //cv::KeyPoint pt;
                //pt.pt.x = idxs.at<float>(i, 0);
                //pt.pt.x = idxs.at<float>(i, 1);
                //pt.angle = -1000000000000;
                //pt.octave = -1;
                
                //keypoints.push_back(pt);
                //cv::Mat descriptors;
                //sift_ext->compute(gray, keypoints, descriptors);
                //std::cerr << descriptors << std::endl;
                //std::cin.get();
                sift_det->detect(gray, keypoints, mask);
                std::cerr << keypoints.size() << std::endl;
                std::cin.get();
                //mask.at<uchar>(idxs.at<float>(i, 0), idxs.at<float>(i, 1)) = 255;
            }
            */
            /*
            cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
            
            cv::Mat descriptors;

            sift_det->detect(frame.gray, keypoints);
            sift_ext->compute(frame.gray, keypoints, descriptors);
            
            if( show_keys )
            {
                cv::Mat out_image;
                cv::drawKeypoints(frame.gray, keypoints, out_image);
                cv::imshow("keypoints", out_image);
                cv::waitKey();
            }
            for(int i = 0 ; i < keypoints.size() ; i++ )
            {
                int r = keypoints[i].pt.y;
                int c = keypoints[i].pt.x;
                int idx = frame.map2D3D.at<int>(r, c);
                if ( idx < 0 )
                    continue;

                keyDescrT temp_descr;
                temp_descr.feaDescr = cv::Mat::zeros(1, descriptors.cols, CV_32FC1);
                descriptors.row(i).copyTo(temp_descr.feaDescr);
                cv::normalize(temp_descr.feaDescr,temp_descr.feaDescr, 1.0);
                temp_descr.fea_type = 0;
                temp_descr.idx = key_vec.size();

                key_descr.push_back(temp_descr);

                keyT temp;
                temp.xyz = frame.surface->at(idx);
                if( frame.normals->empty() == false )
                    temp.normal = frame.normals->at(idx);
                key_vec.push_back(temp);
            }
            */
        }
        //find the corresponding interest indexes in idx path
        
    }
    
}

/*
ObjectSet readBBTrainCADView(std::string path, std::string sub_path, int c1 = -1, int c2 = -1)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? BB_INST_MAX - 1 : c2;
    //std::string model_file
    std::ifstream model_list((path + "/models.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    ObjectSet objects;
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path +"/" + class_name + "/" + sub_path);
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        std::vector<MulInfoT> cur_train;
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            std::string filename(workspace+"/"+*it);
            std::string filename_n(workspace+"/normal_"+*it);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(filename_n, *cloud_normal);
            
            MulInfoT temp = convertPCD(cloud, cloud_normal);
            
            cur_train.push_back(temp);
        }
        objects.push_back(cur_train);
    }

    model_list.close();
    return objects;
}
 */