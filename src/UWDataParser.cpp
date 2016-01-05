#include "../include/UWDataParser.h"

void PreProcess_UW(std::string in_path, std::string out_path, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    int min_num = 100;
    float radius = 0.03;
    
    //browse all pcd point clouds
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(in_path, pcd_files);
    
    #pragma omp parallel for schedule(dynamic, 1) 
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        pcl::PointCloud<PointXYZRGBIM>::Ptr cur_cloud(new pcl::PointCloud<PointXYZRGBIM>());
        pcl::io::loadPCDFile(in_path + "/" + (*it), *cur_cloud);
        
        //std::cerr<<"Loading: "<< in_path + "/" + (*it) << std::endl;
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*cur_cloud, *cloud);
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // First filtering the point cloud
        pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
        std::vector<pcl::PointIndices> cluster_indices;
        if( cloud->size() >= min_num )
        {
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
            tree->setInputCloud (cloud);

            pcl::EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance (0.015);     // 1.5cm
            ec.setMinClusterSize (min_num);
            ec.setMaxClusterSize (INF_);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud);
            ec.extract (cluster_indices);
        }

        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                filtered_cloud->push_back (cloud->at(*pit)); 
        
        if( filtered_cloud->size() <= 0 )
        {
            std::cerr<<"Filtering Failed --- "<< in_path+"/"+(*it) << " " << cloud->size() << std::endl;
            pcl::copyPointCloud(*cloud, *filtered_cloud);
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /*/ Second translate point cloud to origin and align viewing angle to z axis
        pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
        ComputeCentroid(filtered_cloud, center);
        Eigen::Matrix4f toOrigin = Eigen::Matrix4f::Identity();
        toOrigin(0, 3) = -center->at(0).x;
        toOrigin(1, 3) = -center->at(0).y;
        toOrigin(2, 3) = -center->at(0).z;
        
        Eigen::Matrix4f flip = -Eigen::Matrix4f::Identity();
        flip(3,3) = 1.0;
        pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, flip*toOrigin);
        
        /*///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Eigen::Vector3f rot_axis1(1, 0, 0);
        pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, Eigen::Vector3f (0,0,0), Eigen::Quaternionf(Eigen::AngleAxisf(M_PI/2, rot_axis1)) );
        //Eigen::Vector3f rot_axis2(0, 0, 1);
        //pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, Eigen::Vector3f (0,0,0), Eigen::Quaternionf(Eigen::AngleAxisf(M_PI, rot_axis2)) );
        if( viewer != NULL )
        {
            viewer->removePointCloud("cloud");
            viewer->removeAllShapes();
            viewer->addPointCloud(filtered_cloud, "cloud");
            viewer->spin();
        }
        
        // Third compute surface normals with 0.03 radius
        pcl::PointCloud<NormalT>::Ptr filtered_normals(new pcl::PointCloud<NormalT>());

        pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimation.setSearchMethod (tree);
        normal_estimation.setNumberOfThreads(8);
        normal_estimation.setRadiusSearch(radius);
        normal_estimation.setInputCloud (filtered_cloud);
        //normal_estimation.setViewPoint(0,0,1);
        normal_estimation.compute (*filtered_normals);
        
        if( viewer != NULL )
        {
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            viewer->addPointCloud(filtered_cloud, "cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            viewer->addPointCloudNormals<PointT, NormalT>(filtered_cloud, filtered_normals, 5, 0.02, "cloud_normals");
            viewer->spin();
        }
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // save both filtered point cloud and cloud normals in out path
        pcl::io::savePCDFile(out_path + "/" + (*it), *filtered_cloud, true);
        pcl::io::savePCDFile(out_path + "/normal_" + (*it), *filtered_normals, true);
    }
}

void batch_PreProcess_UW(std::string in_path, std::string out_path, int c1, int c2)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? UW_INST_MAX - 1 : c2;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    //viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer("viewer"));
    //viewer->initCameraParameters();
    //viewer->addCoordinateSystem(0.1);
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    
    std::ifstream model_list((in_path + "/models_insts.txt").c_str(), std::ios::in);
    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
        
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        std::string cur_in_path(in_path + "/" + class_name);
        std::string cur_out_path(out_path + "/" + class_name);
        if( exists_dir(cur_out_path) == false )
            boost::filesystem::create_directories(cur_out_path);
        
        PreProcess_UW(cur_in_path, cur_out_path, viewer);
    }
    
    model_list.close();
}

int readSeqID_UW(std::string name)
{
    int id = -1;
    for( int i = name.size() - 1 ; i >= 0 ; i-- )
    {
        if( name[i] == '_' )
        {
            id = name[i-1] - '0';
            break;
        }
    }
    return id;
}

void sweepNaN(std::string path, int c1, int c2)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? BB_INST_MAX - 1 : c2;
    std::ifstream model_list((path + "/models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path +"/" + class_name);
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        
        pcl::ExtractIndices<PointT> extract;
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            std::string filename(workspace+"/"+*it);
            std::string filename_n(workspace+"/normal_"+*it);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            
            pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(filename_n, *cloud_normal);
            
            pcl::PointCloud<NormalT>::Ptr new_normal(new pcl::PointCloud<NormalT>());
            pcl::PointCloud<PointT>::Ptr new_cloud(new pcl::PointCloud<PointT>());
            
            std::vector<int> idxs;
            pcl::removeNaNNormalsFromPointCloud(*cloud_normal, *new_normal, idxs);
            extract.setInputCloud (cloud);
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            inliers->indices = idxs;
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*new_cloud);
            
            if( cloud_normal->is_dense == false || cloud->is_dense == false )
            {
                std::cerr << "Ha" << std::endl;
            }
            
            if( new_normal->size() != cloud_normal->size() )
            {
                std::cerr << cloud_normal->size() << " " << cloud->size() << std::endl;
                std::cerr << new_normal->size() << " " << new_cloud->size() << std::endl;
                std::cin.get();
            }
        }
        
    }
    
    model_list.close();
}

int readFrameID(std::string name)
{
    int e_idx, s_idx;
    for( int i = name.size() - 1 ; i >= 0 ; i-- )
    {
        if( name[i] == '.' )
            e_idx = i;
        if( name[i] == '_' )
        {
            s_idx = i;
            break;
        }   
    }
    int id = std::atoi(name.substr(s_idx+1, e_idx).c_str());
            
            
    return id;
}

void readUWInst(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, int inter)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? UW_INST_MAX - 1 : c2;
    std::ifstream model_list((path + "models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    train_set.clear();
    test_set.clear();
    //train_set.resize(e-s+1);
    //test_set.resize(e-s+1);
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path + class_name + "/");
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        //int idx = m_id - s;
        std::vector<MulInfoT> cur_train, cur_test;
        //for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        size_t file_num = pcd_files.size();
//        #pragma omp parallel for schedule(dynamic, 1)
        for( size_t j = 0 ; j < file_num ; j++ )
        {
            std::string filename(workspace+pcd_files[j]);
            std::string filename_n(workspace+"normal_"+pcd_files[j]);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            
            pcl::io::loadPCDFile(filename, *cloud);
            
//            std::cerr << cloud->size() << " " << filename << std::endl;
//            std::cin.get();
            if( cloud->size() >= 30 )
            {
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
//                pcl::io::loadPCDFile(filename_n, *cloud_normal);

                MulInfoT temp = convertPCD(cloud, cloud_normal);
                //std::cerr << temp.lrf->size() << " " << temp.cloud->size() << std::endl;
                int id = readSeqID_UW(pcd_files[j]);
                int frames = readFrameID(pcd_files[j]);
                
                if( inter <= 0 )
                {
                    #pragma omp critical
                    {
                        cur_train.push_back(temp);
                    } 
                }
                else if( frames % inter == 0 )
                {
                    if( id == 1 || id == 4 )
                    {
                        #pragma omp critical
                        {
                            cur_train.push_back(temp);
                        }     
                    }
                    else
                    {
                        #pragma omp critical
                        {
                            cur_test.push_back(temp);
                        }
                    }
                }
                    
            }
        }
        train_set.push_back(cur_train);
        test_set.push_back(cur_test);
    }
    
    model_list.close();
}

void readUWInstAll(std::string path, CloudSet &clouds, NormalSet &normals, int c1, int c2, float prob)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? UW_INST_MAX - 1 : c2;
    std::ifstream model_list((path + "/models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path +"/" + class_name);
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        std::vector<MulInfoT> cur_train, cur_test;
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            std::string filename(workspace+"/"+*it);
            std::string filename_n(workspace+"/normal_"+*it);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            
            if( cloud->size() >= 30 && play_dice(prob) == true )
            {
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
                pcl::io::loadPCDFile(filename_n, *cloud_normal);
                
                clouds.push_back(cloud);
                normals.push_back(cloud_normal);
            }
        }
    }
    model_list.close();
}

void readUWInstWithImg(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, int inter)
{
    int bound_px = 3;
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? UW_INST_MAX - 1 : c2;
    std::ifstream model_list((path + "models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    float constant = 570.3;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( false )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
    }
    
    train_set.clear();
    test_set.clear();
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path + class_name + "/");
        std::vector<std::string> rgb_files;
        
        find_files(workspace, "_crop.png", rgb_files);
        
        std::vector<MulInfoT> cur_train, cur_test;
        size_t file_num = rgb_files.size();
//        std::cerr << file_num << std::endl;
//        #pragma omp parallel for schedule(dynamic, 1)
        for( size_t j = 0 ; j < file_num ; j++ )
        {
            std::string cur_name(rgb_files[j].substr(0, rgb_files[j].size() - 9));
            
            int frames = readFrameID(cur_name);
            if( frames % inter != 0 )
                continue;
            
            std::string rfile(workspace+rgb_files[j]);
            std::string dfile(workspace + cur_name +"_depthcrop.png");
            std::string mfile(workspace + cur_name +"_maskcrop.png");
            std::string lfile(workspace+cur_name+"_loc.txt");
            if( exists_test(dfile) == false || exists_test(mfile) == false || exists_test(lfile) == false )
                continue;
            cv::Mat rgb = cv::imread(rfile);
            cv::Mat depth = cv::imread(dfile, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file 
            depth.convertTo(depth, CV_32F); // convert the image data to float type
            cv::Mat mask = cv::imread(mfile);
             
            std::ifstream fp;
            fp.open(lfile.c_str());
            
            int topleft_x, dump, topleft_y;
            char tmp;
            fp >> topleft_x;
            fp >> tmp;
//            fp >> dump;
            fp >> topleft_y;
//            std::cerr << workspace+rgb_files[j].substr(0, rgb_files[j].size() - 9)+"_loc.txt" << std::endl;
//            std::cerr << topleft_x << " " << topleft_y << std::endl;
            fp.close();
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            cloud->resize(rgb.rows*rgb.cols);
            cloud->is_dense = false;
            cloud->height = rgb.rows;
            cloud->width = rgb.cols;
            int width = cloud->width;
            int height = cloud->height;
            
            int valid_count = 0;
            for( int r = 0 ; r < rgb.rows ; r++ ){
                for( int c = 0 ; c < rgb.cols ; c++ )
                {
                    int idx = r*width+c;
                    uint32_t rr = rgb.at<uchar>(r, c*3+2);
                    uint32_t gg = rgb.at<uchar>(r, c*3+1);
                    uint32_t bb = rgb.at<uchar>(r, c*3+0);
                    cloud->at(idx).rgba =  rr<<16 | gg<<8 | bb;
                    
                    float depth_tmp = depth.at<float>(r, c) / 1000;
                    int r1 = r-bound_px < 0 ? 0 : r-bound_px;
                    int r2 = r+bound_px >= height ? 0 : r+bound_px;
                    int c1 = c-bound_px < 0 ? 0 : c-bound_px;
                    int c2 = c+bound_px >= width ? 0 : c+bound_px;
                    
                    if( mask.at<bool>(r, c*3) > 0 && mask.at<bool>(r, c1*3) > 0 && mask.at<bool>(r, c2*3) > 0 &&
                            mask.at<bool>(r1, c*3) > 0 && mask.at<bool>(r2, c*3) > 0 && depth_tmp > 0 )
                    {
                        valid_count++;
                        cloud->at(idx).z = depth_tmp;
                        cloud->at(idx).x = depth_tmp / constant * (c + topleft_x - 320);
                        cloud->at(idx).y = depth_tmp / constant * (r + topleft_y - 240);
                    }
                    else
                    {
                        cloud->at(idx).x = std::numeric_limits<float>::quiet_NaN();
                        cloud->at(idx).y = std::numeric_limits<float>::quiet_NaN();
                        cloud->at(idx).z = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }
//            viewer->addPointCloud(cloud, "cloud");
//            viewer->spin();
//            viewer->removeAllPointClouds();
//            std::cerr << valid_count <<" " << cur_name << std::endl;
//            std::cin.get();
            if( valid_count >= 30 )
            {
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
//                pcl::io::loadPCDFile(filename_n, *cloud_normal);

                MulInfoT temp = convertPCD(cloud, cloud_normal);
                int id = readSeqID_UW(cur_name);
                
                
//                std::cerr << rgb_files[j] << " " << id << " " << frames << std::endl;
                
                if( inter <= 0 )
                {
                    #pragma omp critical
                    {
                        cur_train.push_back(temp);
                    } 
                }
                else 
                {
                    #pragma omp critical
                    {
                        if( id == 1 || id == 4 )
                        {

    //                            std::cerr << id << " ";
                                cur_train.push_back(temp);

                        }
                        else
                        {

                                cur_test.push_back(temp);
                        }
                    }
                }
            }
        }
        train_set.push_back(cur_train);
        test_set.push_back(cur_test);
    }
    
    model_list.close();
}

void readUWInstTwo(std::string path_img, std::string path_cloud, ObjectSet &train_img, ObjectSet &train_cloud, ObjectSet &test_img, ObjectSet &test_cloud, int c1, int c2, int inter_train, int inter_test)
{
    int bound_px = 3;
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? UW_INST_MAX - 1 : c2;
    std::ifstream model_list((path_img + "models_insts.txt").c_str(), std::ios::in);

    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    
    float constant = 570.3;
    
    train_img.clear();
    train_cloud.clear();
    test_img.clear();
    test_cloud.clear();
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path_img + class_name + "/");
        std::vector<std::string> rgb_files;
        
        find_files(workspace, "_crop.png", rgb_files);
        
        std::vector<MulInfoT> cur_train, cur_test;
        std::vector<MulInfoT> cur_train1, cur_test1;
        size_t file_num = rgb_files.size();
//        std::cerr << file_num << std::endl;
//        #pragma omp parallel for schedule(dynamic, 1)
        for( size_t j = 0 ; j < file_num ; j++ )
        {
            std::string cur_name(rgb_files[j].substr(0, rgb_files[j].size() - 9));
            
            int frames = readFrameID(cur_name);
            int id = readSeqID_UW(cur_name);
            if( id == 1 || id == 4 )
            {
                if( frames % inter_train != 0 )
                    continue;
            }
            else
            {
                if( frames % inter_test != 0 )
                    continue;
            }
            
            std::string rfile(workspace+rgb_files[j]);
            std::string dfile(workspace + cur_name +"_depthcrop.png");
            std::string mfile(workspace + cur_name +"_maskcrop.png");
            std::string lfile(workspace + cur_name+"_loc.txt");
            std::string cloud_file(path_cloud + class_name + "/" + cur_name+".pcd");
            if( exists_test(dfile) == false || exists_test(mfile) == false || exists_test(lfile) == false || exists_test(cloud_file) == false )
                continue;
            cv::Mat rgb = cv::imread(rfile);
            cv::Mat depth = cv::imread(dfile, CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR ); // Read the file 
            depth.convertTo(depth, CV_32F); // convert the image data to float type
            cv::Mat mask = cv::imread(mfile);
             
            std::ifstream fp;
            fp.open(lfile.c_str());
            
            int topleft_x, dump, topleft_y;
            char tmp;
            fp >> topleft_x;
            fp >> tmp;
//            fp >> dump;
            fp >> topleft_y;
//            std::cerr << workspace+rgb_files[j].substr(0, rgb_files[j].size() - 9)+"_loc.txt" << std::endl;
//            std::cerr << topleft_x << " " << topleft_y << std::endl;
            fp.close();
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            cloud->resize(rgb.rows*rgb.cols);
            cloud->is_dense = false;
            cloud->height = rgb.rows;
            cloud->width = rgb.cols;
            int width = cloud->width;
            int height = cloud->height;
            
            int valid_count = 0;
            for( int r = 0 ; r < rgb.rows ; r++ ){
                for( int c = 0 ; c < rgb.cols ; c++ )
                {
                    int idx = r*width+c;
                    uint32_t rr = rgb.at<uchar>(r, c*3+2);
                    uint32_t gg = rgb.at<uchar>(r, c*3+1);
                    uint32_t bb = rgb.at<uchar>(r, c*3+0);
                    cloud->at(idx).rgba =  rr<<16 | gg<<8 | bb;
                    
                    float depth_tmp = depth.at<float>(r, c) / 1000;
                    int r1 = r-bound_px < 0 ? 0 : r-bound_px;
                    int r2 = r+bound_px >= height ? 0 : r+bound_px;
                    int c1 = c-bound_px < 0 ? 0 : c-bound_px;
                    int c2 = c+bound_px >= width ? 0 : c+bound_px;
                    
                    if( mask.at<bool>(r, c*3) > 0 && mask.at<bool>(r, c1*3) > 0 && mask.at<bool>(r, c2*3) > 0 &&
                            mask.at<bool>(r1, c*3) > 0 && mask.at<bool>(r2, c*3) > 0 && depth_tmp > 0 )
                    {
                        valid_count++;
                        cloud->at(idx).z = depth_tmp;
                        cloud->at(idx).x = depth_tmp / constant * (c + topleft_x - 320);
                        cloud->at(idx).y = depth_tmp / constant * (r + topleft_y - 240);
                    }
                    else
                    {
                        cloud->at(idx).x = std::numeric_limits<float>::quiet_NaN();
                        cloud->at(idx).y = std::numeric_limits<float>::quiet_NaN();
                        cloud->at(idx).z = std::numeric_limits<float>::quiet_NaN();
                    }
                }
            }

            if( valid_count >= 30 )
            {
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());

                MulInfoT temp = convertPCD(cloud, cloud_normal);
                
                
                pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT>());
                pcl::PointCloud<NormalT>::Ptr cloud_normal1(new pcl::PointCloud<NormalT>());
                pcl::io::loadPCDFile(cloud_file, *cloud1);
                MulInfoT temp1 = convertPCD(cloud1, cloud_normal1);
                
               
                #pragma omp critical
                {
                    if( id == 1 || id == 4 )
                    {
                        cur_train.push_back(temp);
                        cur_train1.push_back(temp1);
                    }
                    else
                    {
                        cur_test.push_back(temp);
                        cur_test1.push_back(temp1);
                    }
                }
                
            }
        }
        train_img.push_back(cur_train);
        test_img.push_back(cur_test);
        train_cloud.push_back(cur_train1);
        test_cloud.push_back(cur_test1);
    }
    
    model_list.close();
}
