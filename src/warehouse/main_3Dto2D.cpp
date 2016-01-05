#include "../include/utility.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include <flann/io/hdf5.h>

std::string path("/home/chi/BigBIRD/processed");

cv::Mat proj2D(const pcl::PointCloud<PointT>::Ptr cloud, double fx, double fy, double x0, double y0, cv::Mat &img)
{
    //cv::Mat img = cv::Mat::zeros(480, 640, CV_8UC3);
    //cv::Mat img = cv::Mat::zeros(1048, 1280, CV_8UC3);
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        int r = floor(y0 + (*it).y * fy / (*it).z+0.5);
        int c = floor(x0 + (*it).x * fx / (*it).z+0.5);
        //int r = y0 + (*it).y * fy / (*it).z;
        //int c = x0 + (*it).x * fx / (*it).z;
        //std::cerr << r << " " << c << std::endl;
        //std::cerr << (*it).y * fy / (*it).z << " " << (*it).x * fx / (*it).z << std::endl;
        
        //std::cin.get();
        //img.at<uchar>(r, c*3+2) = ((*it).rgba >> 16) & 0x0000ff;
        //img.at<uchar>(r, c*3+1) = ((*it).rgba >> 8)  & 0x0000ff;
        //img.at<uchar>(r, c*3+0) = ((*it).rgba)       & 0x0000ff;
        img.at<uchar>(r, c*3+2) = 255;
        img.at<uchar>(r, c*3+1) = 255;
        img.at<uchar>(r, c*3+0) = 255;
    }
    
    return img;
}

cv::Mat crop2D(const pcl::PointCloud<PointT>::Ptr cloud, double fx, double fy, double x0, double y0, const cv::Mat &rgb)
{
    std::vector<int> r_idx(cloud->size());
    std::vector<int> c_idx(cloud->size());
    int base =0;
    int max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++, base++ )
    {
        int r = floor(y0 + (*it).y * fy / (*it).z+0.5);
        int c = floor(x0 + (*it).x * fx / (*it).z+0.5);
        
        r_idx[base] = r;
        c_idx[base] = c;
        
        if( max_x < c ) max_x = c;
        if( max_y < r ) max_y = r;
        
        if( min_x > c ) min_x = c;
        if( min_y > r ) min_y = r;
    }
    
    cv::Mat img = cv::Mat::zeros(max_y-min_y+1, max_x-min_x+1, CV_8UC3);
    rgb.rowRange(min_y, max_y+1).colRange(min_x, max_x+1).copyTo(img);
    /*
    base = 0;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++, base++ )
    {
        int r = r_idx[base] - min_y;
        int c = c_idx[base] - min_x;
        
        img.at<uchar>(r, c*3+2) = ((*it).rgba >> 16) & 0x0000ff;
        img.at<uchar>(r, c*3+1) = ((*it).rgba >> 8)  & 0x0000ff;
        img.at<uchar>(r, c*3+0) = ((*it).rgba)       & 0x0000ff;
    }
    */
    
    return img;
}

cv::Mat get_proj2D_idx(const pcl::PointCloud<PointT>::Ptr cloud, double fx, double fy, double x0, double y0)
{
    cv::Mat idxs = cv::Mat::zeros(cloud->size(), 2, CV_32FC1);
    float *ptr = (float *)idxs.data;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        *ptr = static_cast<float>(y0 + (*it).y * fy / (*it).z);
        ptr++;
        *ptr = static_cast<float>(x0 + (*it).x * fx / (*it).z);
        ptr++;
    }
    
    return idxs;
}

int main(int argc, char** argv)
{
    
    //pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    //viewer->initCameraParameters();
    //viewer->addCoordinateSystem(0.1);
    
    int c1 = 0, c2 = BB_INST_MAX-1;
    pcl::console::parse_argument(argc, argv, "--p", path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
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
        
        std::string workspace(path + "/" + class_name);
        
        std::vector<std::string> pcd_files;
        boost::filesystem::path p(workspace);
        find_files(p, ".jpg", pcd_files);
        
        std::string save_path(path + "/../filtered_rgbd/" + class_name);
        if( exists_dir(save_path) == false )
            boost::filesystem::create_directories(save_path);
        
        flann::Matrix<double> K1, K2, K3, K4, K5;
        flann::load_from_file(K1, workspace+"/calibration.h5", "NP1_rgb_K");
        flann::load_from_file(K2, workspace+"/calibration.h5", "NP2_rgb_K");
        flann::load_from_file(K3, workspace+"/calibration.h5", "NP3_rgb_K");
        flann::load_from_file(K4, workspace+"/calibration.h5", "NP4_rgb_K");
        flann::load_from_file(K5, workspace+"/calibration.h5", "NP5_rgb_K");
        
        double rgb_params[5][4] ={ {K1[0][0], K1[1][1], K1[0][2], K1[1][2]},
                                   {K2[0][0], K2[1][1], K2[0][2], K2[1][2]},
                                   {K3[0][0], K3[1][1], K3[0][2], K3[1][2]},
                                   {K4[0][0], K4[1][1], K4[0][2], K4[1][2]},
                                   {K5[0][0], K5[1][1], K5[0][2], K5[1][2]} };
        
        
        flann::Matrix<double> KK1, KK2, KK3, KK4, KK5;
        flann::load_from_file(KK1, workspace+"/calibration.h5", "NP1_depth_K");
        flann::load_from_file(KK2, workspace+"/calibration.h5", "NP2_depth_K");
        flann::load_from_file(KK3, workspace+"/calibration.h5", "NP3_depth_K");
        flann::load_from_file(KK4, workspace+"/calibration.h5", "NP4_depth_K");
        flann::load_from_file(KK5, workspace+"/calibration.h5", "NP5_depth_K");
        
        double depth_params[5][4] ={{KK1[0][0], KK1[1][1], KK1[0][2], KK1[1][2]},
                                    {KK2[0][0], KK2[1][1], KK2[0][2], KK2[1][2]},
                                    {KK3[0][0], KK3[1][1], KK3[0][2], KK3[1][2]},
                                    {KK4[0][0], KK4[1][1], KK4[0][2], KK4[1][2]},
                                    {KK5[0][0], KK5[1][1], KK5[0][2], KK5[1][2]} };
        
        delete[] K1.ptr();
        delete[] KK1.ptr();
        delete[] K2.ptr();
        delete[] KK2.ptr();
        delete[] K3.ptr();
        delete[] KK3.ptr();
        delete[] K4.ptr();
        delete[] KK4.ptr();
        delete[] K5.ptr();
        delete[] KK5.ptr();
        
        
        poseVec depth_to_color(5);
        for( int i = 0 ; i < 5 ; i++ )
        {
            std::ostringstream ss;
            ss << i+1;
            
            //std::cerr << "H_NP"+ss.str()+"_ir_from_NP5" << std::endl;
            flann::Matrix<double> ir_from_5, rgb_from_5;
            flann::load_from_file(ir_from_5, workspace+"/calibration.h5", "H_NP"+ss.str()+"_ir_from_NP5");
            flann::load_from_file(rgb_from_5, workspace+"/calibration.h5", "H_NP"+ss.str()+"_from_NP5");
            Eigen::Matrix4f mat_temp1, mat_temp2;
            mat_temp1 << ir_from_5[0][0], ir_from_5[0][1], ir_from_5[0][2], ir_from_5[0][3], 
                         ir_from_5[1][0], ir_from_5[1][1], ir_from_5[1][2], ir_from_5[1][3], 
                         ir_from_5[2][0], ir_from_5[2][1], ir_from_5[2][2], ir_from_5[2][3], 
                         ir_from_5[3][0], ir_from_5[3][1], ir_from_5[3][2], ir_from_5[3][3];
            mat_temp2 << rgb_from_5[0][0], rgb_from_5[0][1], rgb_from_5[0][2], rgb_from_5[0][3], 
                         rgb_from_5[1][0], rgb_from_5[1][1], rgb_from_5[1][2], rgb_from_5[1][3], 
                         rgb_from_5[2][0], rgb_from_5[2][1], rgb_from_5[2][2], rgb_from_5[2][3], 
                         rgb_from_5[3][0], rgb_from_5[3][1], rgb_from_5[3][2], rgb_from_5[3][3];
            depth_to_color[i] = mat_temp2 * mat_temp1.inverse();
            
            delete[] ir_from_5.ptr();
            delete[] rgb_from_5.ptr();
            //std::cerr << mat_temp1 << std::endl;
            //std::cerr << mat_temp2 << std::endl;
            //std::cerr << depth_to_color[i] << std::endl;
        }
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            
            char tmp[2] = {(*it)[2], 0};
            int camera_idx = std::atoi(tmp);
            camera_idx--;
            //std::cerr <<"***" << camera_idx<<std::endl;
            std::string name_no_postfix((*it).substr(0, (*it).size()-4));
            
            if( exists_test(workspace+"/"+name_no_postfix+".h5") == false)
                exit(0);
            
            std::string jpg_file(workspace + "/" + name_no_postfix+ ".jpg");
            std::string mask_file(workspace + "/masks/" + name_no_postfix+ "_mask.pbm");
            cv::Mat raw_img = cv::imread(jpg_file);
            cv::Mat mask_ = cv::imread(mask_file);
            
            cv::Mat mask;
            cv::cvtColor(mask_, mask, CV_BGR2GRAY);
            
            int max_x = -100000, min_x = 100000, max_y = -100000, min_y = 100000;
            for( int r = 0 ; r < mask.rows; r++ ){
                for( int c = 0 ; c < mask.cols ; c++ ){                   
                    if( mask.at<uchar>(r, c) == 0 )
                    {
                        if( max_x < c ) max_x = c;
                        if( max_y < r ) max_y = r;

                        if( min_x > c ) min_x = c;
                        if( min_y > r ) min_y = r;
                    }
                }
            }
            cv::Mat cropped_rgb = cv::Mat::zeros(max_y-min_y+1, max_x-min_x+1, CV_8UC3);
            cv::Mat cropped_mask= cv::Mat::zeros(max_y-min_y+1, max_x-min_x+1, CV_8UC1);
            
            raw_img.rowRange(min_y, max_y+1).colRange(min_x, max_x+1).copyTo(cropped_rgb);
            mask.rowRange(min_y, max_y+1).colRange(min_x, max_x+1).copyTo(cropped_mask);
            
            //cv::imshow("viewer", cropped_rgb);
            //cv::waitKey(0);
            
            cv::imwrite(save_path+"/"+name_no_postfix+"_rgbcrop.png", cropped_rgb);
            cv::imwrite(save_path+"/"+name_no_postfix+"_rgbcrop_mask.png", cropped_mask);
            
            /********************** Reading Depth Maps ************************/
            flann::Matrix<double> depth;
            flann::load_from_file(depth, path+"/"+class_name+"/"+name_no_postfix+".h5", "depth");
            
            max_x = -100000, min_x = 100000, max_y = -100000, min_y = 100000;
            std::vector<int> r_vec;
            std::vector<int> c_vec;
            //cv::Mat mask_tmp = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC1);
            
            for(int r = 0 ; r < depth.rows ; r++ ){
                for( int c = 0 ; c < depth.cols ; c++ ){
                    float ori_z1 = depth[r][c]/10000;
                    if( ori_z1 > 0 )
                    {
                        float ori_x1 = (c - depth_params[camera_idx][2])/depth_params[camera_idx][0] * ori_z1;
                        float ori_y1 = (r - depth_params[camera_idx][3])/depth_params[camera_idx][1] * ori_z1;
                        
                        float x1 = depth_to_color[camera_idx](0, 0)*ori_x1 + depth_to_color[camera_idx](0, 1)*ori_y1 + depth_to_color[camera_idx](0, 2)*ori_z1 + depth_to_color[camera_idx](0, 3);
                        float y1 = depth_to_color[camera_idx](1, 0)*ori_x1 + depth_to_color[camera_idx](1, 1)*ori_y1 + depth_to_color[camera_idx](1, 2)*ori_z1 + depth_to_color[camera_idx](1, 3);
                        float z1 = depth_to_color[camera_idx](2, 0)*ori_x1 + depth_to_color[camera_idx](2, 1)*ori_y1 + depth_to_color[camera_idx](2, 2)*ori_z1 + depth_to_color[camera_idx](2, 3);
                        
                        int rgb_c = floor(rgb_params[camera_idx][2] + x1 * rgb_params[camera_idx][0] / z1+0.5);
                        int rgb_r = floor(rgb_params[camera_idx][3] + y1 * rgb_params[camera_idx][1] / z1+0.5);
                        //std::cerr << rgb_r << " " << rgb_c << std::endl;
                        //mask_tmp.at<uchar>(rgb_r, rgb_c) = 255;
                        
                        if( mask.at<uchar>(rgb_r, rgb_c) == 0 )
                        {
                            
                            r_vec.push_back(r);
                            c_vec.push_back(c);
                            if( max_x < c ) max_x = c;
                            if( max_y < r ) max_y = r;

                            if( min_x > c ) min_x = c;
                            if( min_y > r ) min_y = r;
                        }
                    }
                }
            }
            //cv::imshow("depth_viewer", mask_tmp);
            //cv::waitKey(0);
            
            cv::Mat cropped_depth, mask_depth;
            //std::cerr << min_y << " " << max_y << " " << min_x << " " << max_x << std::endl;
            if( min_y > max_y )
            {
                std::cerr << "Empty: " << class_name << "-" << (*it) << std::endl;
                cropped_depth = cv::Mat::zeros(cropped_rgb.rows, cropped_rgb.cols, CV_16UC1);
                mask_depth = 255*cv::Mat::ones(cropped_rgb.rows, cropped_rgb.cols, CV_16UC1);
                min_x = 0;min_y=0;
            }
            else
            {
                cropped_depth = cv::Mat::zeros(max_y-min_y+1, max_x-min_x+1, CV_16UC1);
                for(int r = min_y ; r <= max_y ; r++ )
                    for( int c = min_x ; c <= max_x ; c++ )
                        cropped_depth.at<uint16_t>(r-min_y, c-min_x) = depth[r][c]/10;    
                mask_depth = 255*cv::Mat::ones(max_y-min_y+1, max_x-min_x+1, CV_8UC1);
                
                for( size_t i = 0 ; i < r_vec.size() ; i++ )
                    mask_depth.at<uchar>(r_vec[i]-min_y, c_vec[i]-min_x) = 0;
            }
            
            //cv::imshow("depth_viewer", mask_depth);
            //cv::waitKey(0);
            std::ofstream fp((save_path+"/"+name_no_postfix+"_loc.txt").c_str(), std::ios::out);
            fp << min_y << " " << min_x;
            fp.close();
            cv::imwrite(save_path+"/"+name_no_postfix+"_depthcrop.png", cropped_depth);
            cv::imwrite(save_path+"/"+name_no_postfix+"_depthcrop_mask.png", mask_depth);
            
            delete[] depth.ptr();            
            //pcl::PointCloud<PointT>::Ptr this_cloud(new pcl::PointCloud<PointT>());
            //pcl::io::loadPCDFile(path + "/../processed/" + class_name + "/clouds/" + name_no_postfix + ".pcd", *this_cloud);
            //viewer->removeAllPointClouds();
            //viewer->addPointCloud(this_cloud, "cloud");
            //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
            //viewer->spin();
        }
    }

    model_list.close();
    return 1;
}


/*
// run one model test
int main(int argc, char** argv)
{
    int c1 = 0, c2 = BB_INST_MAX-1;
    pcl::console::parse_argument(argc, argv, "--p", path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    std::ifstream model_list((path + "/models.txt").c_str(), std::ios::in);
    
    if( model_list.is_open() == false )
    {
        std::cerr<<"No Model File or Test File Selected!"<<std::endl;
        exit(0);
    }
    //cv::namedWindow("viewer");
    
    for( int m_id = 0 ; m_id < BB_INST_MAX ; m_id++ )
    {
        std::string class_name;
        model_list >> class_name;
   
        if( m_id < c1 )
            continue;
        else if (m_id > c2)
            break;    
        
        std::cerr << "Loading-"<< m_id <<": "<< class_name << std::endl;
        
        flann::Matrix<double> K1, K2, K3, K4, K5;
        flann::load_from_file(K1, path+"/"+class_name+"/calibration.h5", "NP1_rgb_K");
        flann::load_from_file(K2, path+"/"+class_name+"/calibration.h5", "NP2_rgb_K");
        flann::load_from_file(K3, path+"/"+class_name+"/calibration.h5", "NP3_rgb_K");
        flann::load_from_file(K4, path+"/"+class_name+"/calibration.h5", "NP4_rgb_K");
        flann::load_from_file(K5, path+"/"+class_name+"/calibration.h5", "NP5_rgb_K");
        
        double fx1 = K1[0][0], fy1 = K1[1][1], x1 = K1[0][2], y1 = K1[1][2];
        double fx2 = K2[0][0], fy2 = K2[1][1], x2 = K2[0][2], y2 = K2[1][2];
        double fx3 = K3[0][0], fy3 = K3[1][1], x3 = K3[0][2], y3 = K3[1][2];
        double fx4 = K4[0][0], fy4 = K4[1][1], x4 = K4[0][2], y4 = K4[1][2];
        double fx5 = K5[0][0], fy5 = K5[1][1], x5 = K5[0][2], y5 = K5[1][2];
        
        std::string workspace(path +"/" + class_name + "/clouds");
        std::vector<std::string> pcd_files;
        getNonNormalPCDFiles(workspace, pcd_files);
        
        std::string save_path(path +"/" + class_name + "/cropped_img");
        if( exists_dir(save_path) == false )
            boost::filesystem::create_directories(save_path);
    
        for( std::vector<std::string>::iterator it = pcd_files.begin() ; it < pcd_files.end() ; it++ )
        {
            std::string filename(workspace+"/"+*it);
            
            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
            pcl::io::loadPCDFile(filename, *cloud);
            
            double cur_fx, cur_fy, cur_x, cur_y;
            switch((*it)[2])
            {
                case '1':
                    cur_fx = fx1;
                    cur_fy = fy1;
                    cur_x = x1;
                    cur_y = y1;
                    break;
                case '2':
                    cur_fx = fx2;
                    cur_fy = fy2;
                    cur_x = x2;
                    cur_y = y2;
                    break;
                case '3':
                    cur_fx = fx3;
                    cur_fy = fy3;
                    cur_x = x3;
                    cur_y = y3;
                    break;
                case '4':
                    cur_fx = fx4;
                    cur_fy = fy4;
                    cur_x = x4;
                    cur_y = y4;
                    break;
                case '5':
                    cur_fx = fx5;
                    cur_fy = fy5;
                    cur_x = x5;
                    cur_y = y5;
                    break;
                default:break;
            }
            std::cerr << class_name << "-" << (*it) << std::endl;
            std::string jpg_file((*it).substr(0, (*it).size()-3)+ "jpg");
            cv::Mat raw_img = cv::imread(path +"/" + class_name + "/img/" + jpg_file);
            //cv::Mat img = proj2D(cloud, cur_fx, cur_fy, cur_x, cur_y, raw_img);
            
            cv::Mat img = crop2D(cloud, cur_fx, cur_fy, cur_x, cur_y, raw_img);
            cv::imshow("viewer", img);
            cv::waitKey(0);
            //cv::Mat idxs = get_proj2D_idx(cloud, cur_fx, cur_fy, cur_x, cur_y);
            //std::string idx_file((*it).substr(0, (*it).size()-3)+ "cvmat");
            //saveMat(save_path+"/idx2d_"+idx_file, idxs);
        }
    }

    model_list.close();
    return 1;
}
//*/


/*
        flann::Matrix<double> bias1, bias2, bias3, bias4, bias5;
        std::cerr << path+"/"+class_name+"/calibration.h5" << std::endl;
        flann::load_from_file(bias1, path+"/"+class_name+"/calibration.h5", "NP1_depth_bias");
        std::cerr << "HiHiHi" << std::endl;
        flann::load_from_file(bias2, path+"/"+class_name+"/calibration.h5", "NP2_depth_bias");
        flann::load_from_file(bias3, path+"/"+class_name+"/calibration.h5", "NP3_depth_bias");
        flann::load_from_file(bias4, path+"/"+class_name+"/calibration.h5", "NP4_depth_bias");
        flann::load_from_file(bias5, path+"/"+class_name+"/calibration.h5", "NP5_depth_bias");
        if( bias1[0][0] != 0 || bias2[0][0] != 0 || bias3[0][0] != 0 || bias4[0][0] != 0 || bias5[0][0] != 0)
        {
            std::cerr << class_name << " Bias not equal to 0!" << std::endl;
        }
        std::cerr << "Hi" << std::endl;
        flann::Matrix<double> scale1, scale2, scale3, scale4, scale5;
        flann::load_from_file(scale1, path+"/"+class_name+"/calibration.h5", "NP1_depth_scale");
        flann::load_from_file(scale2, path+"/"+class_name+"/calibration.h5", "NP2_depth_scale");
        flann::load_from_file(scale3, path+"/"+class_name+"/calibration.h5", "NP3_depth_scale");
        flann::load_from_file(scale4, path+"/"+class_name+"/calibration.h5", "NP4_depth_scale");
        flann::load_from_file(scale5, path+"/"+class_name+"/calibration.h5", "NP5_depth_scale");
        if( scale1[0][0] != 0 || scale2[0][0] != 0 || scale3[0][0] != 0 || scale4[0][0] != 0 || scale5[0][0] != 0)
        {
            std::cerr << class_name << " Scale not equal to 0!" << std::endl;
        }
        std::cerr << "HiHi" << std::endl;
        */
