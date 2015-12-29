#include "../include/JHUDataParser.h"

#include <pcl/visualization/pcl_visualizer.h>

int readSeqID_JHU(std::string name)
{
    int id = -1;
    int underscore_count = 0;
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

void PreProcess_JHU(std::string in_path, std::string out_path, pcl::visualization::PCLVisualizer::Ptr viewer)
{
    int min_num = 100;
    float radius = 0.03;
    float elev = 0.052;
    
    //browse all pcd point clouds
    std::vector<std::string> pcd_files;
    getNonNormalPCDFiles(in_path, pcd_files);
    
    //pcd_files.clear();
    //pcd_files.push_back("clipper_blue_0_35.pcd");
    
    #pragma omp parallel for schedule(dynamic, 1) 
    for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
    {
        pcl::PointCloud<PointT>::Ptr cur_cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(in_path + "/" + (*it), *cur_cloud);
        
        int cur_w = cur_cloud->width;
        int cur_h = cur_cloud->height;
        
        std::vector<int> idx_f;
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::removeNaNFromPointCloud(*cur_cloud, *cloud, idx_f);
        
        std::cerr<<"Loading: "<< in_path + "/" + (*it) << std::endl;
        
        std::string inst_name((*it).substr(0, (*it).size()-4));
        std::string tmp1(in_path + "/PlaneCoef_"+inst_name+".txt");
        std::ifstream coef_fp(tmp1.c_str(), std::ios::in);
        int seq_id = readSeqID_JHU(*it);
        if( coef_fp.is_open() == false )
        {
            std::ostringstream ss;
            ss << seq_id;
            std::string tmp2(in_path + "/PlaneCoef_"+ss.str()+".txt");
            coef_fp.open(tmp2.c_str(), std::ios::in);
            if( coef_fp.is_open() == false )
            {
                std::cerr << "No Plane Coeficients Provided!" << std::endl;
                exit(0);
            }
        }
        pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
        coef->values.resize(4);
        coef_fp >> coef->values[0] >> coef->values[1] >> coef->values[2] >> coef->values[3];
        
        // Segmenting the object out from the background using Elevation
        pcl::PointCloud<PointT>::Ptr cloud_proj(new pcl::PointCloud<PointT>());
        pcl::ProjectInliers<PointT> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        proj.setInputCloud (cloud);
        proj.setModelCoefficients (coef);

        proj.filter (*cloud_proj);

        pcl::PointCloud<PointT>::iterator it_ori = cloud->begin();
        pcl::PointCloud<PointT>::iterator it_proj = cloud_proj->begin();
        pcl::PointCloud<PointT>::Ptr cloud_object(new pcl::PointCloud<PointT>());
        std::vector<int> in_idx;
        for( int base = 0 ; it_ori < cloud->end(), it_proj < cloud_proj->end() ; it_ori++, it_proj++, base++ )
        {
            float diffx = (*it_ori).x-(*it_proj).x;
            float diffy = (*it_ori).y-(*it_proj).y;
            float diffz = (*it_ori).z-(*it_proj).z;

            //distance from the point to the plane
            float dist = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
            //if( dist > elev && fabs((*it_ori).x) <= 0.11 && fabs((*it_ori).y) <= 0.1)
            if ( dist > elev )
            {
                int idx_ori = idx_f[base];
                in_idx.push_back(idx_ori);
                cloud_object->push_back(*it_ori);
            }
        }
        
        pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>());
        std::vector<pcl::PointIndices> filtered_idx;
        if( cloud_object->size() >= min_num )
        {
            pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
            tree->setInputCloud (cloud);

            pcl::EuclideanClusterExtraction<PointT> ec;
            ec.setClusterTolerance (0.015);     // 1.5cm
            ec.setMinClusterSize (min_num);
            ec.setMaxClusterSize (INF_);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_object);
            ec.extract (filtered_idx);
            
            if( filtered_idx.empty() == true )
            {
                std::cerr<<"Filtering Failed --- "<< in_path+"/"+(*it) << " " << cloud_object->size() << std::endl;
                filtered_cloud = cloud_object;
            }
            else
            {
                std::vector<int> tmp_idx = in_idx;
                in_idx.clear();
                for (std::vector<pcl::PointIndices>::const_iterator it = filtered_idx.begin (); it != filtered_idx.end (); ++it){
                    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
                    {
                        in_idx.push_back(tmp_idx[*pit]);
                        filtered_cloud->push_back(cloud_object->at(*pit));
                    }
                }
            }
        }
        else
            filtered_cloud = cloud_object;
        
        int max_r = -1000, min_r = 1000, max_c = -1000, min_c = 1000;
        cv::Mat idx_3D_2D = cv::Mat::zeros(in_idx.size(), 1, CV_32SC1);
        int *ptr = (int *)idx_3D_2D.data;
        for(std::vector<int>::iterator pit = in_idx.begin() ; pit < in_idx.end() ; pit++, ptr++ )
        {
            *ptr = *pit;
            int r = *pit / cur_w;
            int c = *pit % cur_w; 

            if( max_r < r ) max_r = r;
            if( max_c < c ) max_c = c;

            if( min_r > r ) min_r = r;
            if( min_c > c ) min_c = c;
        }
        
        int c_range = max_c - min_c + 1;
        int r_range = max_r - min_r + 1;
        cv::Mat rgb = cv::Mat::zeros(r_range, c_range, CV_8UC3);
        cv::Mat depth = cv::Mat::zeros(r_range, c_range, CV_32FC1);
        for( int r = min_r ; r <= max_r ; r++ )
        {
            int diff_r = r - min_r;
            for( int c = min_c ; c <= max_c ; c++ )
            {
                int diff_c = c - min_c;
                int cur_idx = r*cur_w + c;
                float cur_z = cur_cloud->at(cur_idx).z;
                if( cur_z == cur_z )
                    depth.at<float>(diff_r, diff_c) = cur_z;
                
                uint32_t tmp = cur_cloud->at(cur_idx).rgba;
                
                rgb.at<uchar>(diff_r, diff_c*3+2) = (tmp >> 16) & 0x0000ff;
                rgb.at<uchar>(diff_r, diff_c*3+1) = (tmp >> 8)  & 0x0000ff;
                rgb.at<uchar>(diff_r, diff_c*3+0) = (tmp)       & 0x0000ff;
            }
        }
        cv::Mat mask = cv::Mat::zeros(r_range, c_range, CV_8UC1);
        for(std::vector<int>::iterator it = in_idx.begin(); it < in_idx.end() ; it++ )
            mask.at<uchar>((*it)/cur_w-min_r, (*it)%cur_w-min_c) = 255;
        
        //cv::imshow("cropped_rgb", rgb);
        //cv::imshow("cropped_mask", mask);
        //cv::waitKey(1);
        
        cv::imwrite(out_path+"/rgb_"+inst_name+".png", rgb);
        cv::imwrite(out_path+"/mask_"+inst_name+".png", mask);
        saveMat(out_path+"/depth_"+inst_name+".cvmat", depth);
        saveMat(out_path+"/3D2D_"+inst_name+".cvmat", idx_3D_2D);
      
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Second translate point cloud to origin and align viewing angle to z axis
        pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
        ComputeCentroid(filtered_cloud, center);
        Eigen::Matrix4f toOrigin = Eigen::Matrix4f::Identity();
        toOrigin(0, 3) = -center->at(0).x;
        toOrigin(1, 3) = -center->at(0).y;
        toOrigin(2, 3) = -center->at(0).z;
        
        Eigen::Matrix4f flip = -Eigen::Matrix4f::Identity();
        flip(3,3) = 1.0;
        pcl::transformPointCloud(*filtered_cloud, *filtered_cloud, flip*toOrigin);
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Third compute surface normals with 0.03 radius
        pcl::PointCloud<NormalT>::Ptr filtered_normals(new pcl::PointCloud<NormalT>());

        pcl::NormalEstimationOMP<PointT, NormalT> normal_estimation;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        normal_estimation.setSearchMethod (tree);
        normal_estimation.setNumberOfThreads(8);
        normal_estimation.setRadiusSearch(radius);
        normal_estimation.setInputCloud (filtered_cloud);
        normal_estimation.setViewPoint(0,0,1);
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

void batch_PreProcess_JHU(std::string in_path, std::string out_path, int c1, int c2)
{
    int s, e;
    s = c1 < 0 ? 0 : c1;
    e = c2 < 0 ? JHU_INST_MAX - 1 : c2;
    
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
    
    for( int m_id = 0 ; m_id < JHU_INST_MAX ; m_id++ )
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
        
        PreProcess_JHU(cur_in_path, cur_out_path, viewer);
    }
    model_list.close();
}

void readJHUInst(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, bool all)
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
        else if(class_name[0] == '#')
            continue;
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path + class_name + "/");
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        //int idx = m_id - s;
        std::vector<MulInfoT> cur_train, cur_test;
        //for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        size_t file_num = pcd_files.size();
        //#pragma omp parallel for schedule(dynamic, 1)
        for( size_t j = 0 ; j < file_num ; j++ )
        {
            std::string filename(workspace + pcd_files[j]);
//            std::string filename_n(workspace+"normal_"+pcd_files[j]);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            
            if( cloud->size() >= 30 )
            {
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
//                pcl::io::loadPCDFile(filename_n, *cloud_normal);
                
                MulInfoT temp = convertPCD(cloud, cloud_normal);
                
                
                int id = readSeqID_JHU(pcd_files[j]);
                if( all == true )
                {
                    #pragma omp critical
                    {
                        cur_train.push_back(temp);
                    }  
                }
                else
                {
                    if( id < 4 )
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


void readJHUInstWithImg(std::string path, ObjectSet &train_set, ObjectSet &test_set, int c1, int c2, bool all)
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
    
    for( int m_id = 0 ; m_id < UW_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < s )
            continue;
        else if (m_id > e)
            break;    
        else if(class_name[0] == '#')
            continue;
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        std::string workspace(path + class_name + "/");
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        
        std::vector<MulInfoT> cur_train, cur_test;
        size_t file_num = pcd_files.size();
        for( size_t j = 0 ; j < file_num ; j++ )
//        for( size_t j = 0 ; j < 10 ; j++ )
        {
            std::string core_name(pcd_files[j].substr(0, pcd_files[j].size()-4));
            
            std::string cloud_name(workspace + pcd_files[j]);
//            std::string normal_name(workspace+"normal_"+pcd_files[j]);
            std::string img_name(workspace+core_name+"_rgbcrop.png");
            std::string idx_name(workspace+"3D2D_"+core_name+".cvmat");
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(cloud_name, *cloud);
            
            if( cloud->size() >= 30 )
            {
                pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
//                pcl::io::loadPCDFile(normal_name, *cloud_normal);
                
                MulInfoT temp = convertPCD(cloud, cloud_normal);
                temp.img = cv::imread(img_name);
                
                readMat(idx_name, temp._3d2d);
                temp.map2d = cv::Mat::ones(temp.img.rows, temp.img.cols, CV_32SC1) * -1;
                for(int k = 0 ; k < temp._3d2d.rows ; k++ )
                    temp.map2d.at<int>(temp._3d2d.at<int>(k, 1), temp._3d2d.at<int>(k, 0) ) = k;
              
//                cv::Mat mask = cv::Mat::zeros(temp.img.rows, temp.img.cols, CV_8UC1);
//                for(int k = 0 ; k < idx_mat.rows ; k++ )
//                {
//                    mask.at<uchar>(idx_mat.at<int>(k, 1), idx_mat.at<int>(k, 0) ) = 255;
//                }
//                
//                cv::imshow("Temp", temp.img);
//                cv::imshow("Mask", mask);
//                cv::waitKey();
                
                int id = readSeqID_JHU(pcd_files[j]);
                if( all == true )
                    cur_train.push_back(temp);
                else
                {
                    if( id < 4 )
                        cur_train.push_back(temp);
                    else
                        cur_test.push_back(temp);
                }
                
                    
            }
        }
        train_set.push_back(cur_train);
        test_set.push_back(cur_test);
    }
    
    model_list.close();
}
