#include "sp_segmenter/utility/utility.h"
#include "sp_segmenter/features.h"
#include "sp_segmenter/BBDataParser.h"
#include "sp_segmenter/UWDataParser.h"
#include "sp_segmenter/JHUDataParser.h"
#include <pcl/ros/conversions.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/ply_io.h>

#include <opencv2/nonfree/nonfree.hpp>

#define ELEV 0.132
#define SIFT_RATIO 0.6

typedef std::vector<cv::KeyPoint> KEYSET;

Eigen::Matrix4f DenseColorICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &initial_guess);
pcl::PointCloud<PointT>::Ptr FilterBoundary(const pcl::PointCloud<PointT>::Ptr cloud);
pcl::PointCloud<PointT>::Ptr cropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, float elev);
pcl::CorrespondencesPtr matchSIFT(const cv::Mat &descr1, const cv::Mat &descr2);
pcl::PointCloud<PointT>::Ptr Meshing(const std::vector< pcl::PointCloud<PointT>::Ptr > &cloud_set,
                                     const std::vector< KEYSET > &keypt_set,
                                     const std::vector< cv::Mat > &keydescr_set);

void AdjustCloudNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<NormalT>::Ptr cloud_normals);

int main(int argc, char** argv)
{
    std::string path("/home/chi/arco/test/build/temp/");
    std::string filename("temp_0_0");
    pcl::console::parse_argument(argc, argv, "--f", filename);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(path+filename+".pcd", *cloud);
    
    int w = cloud->width;
    int h = cloud->height;
    cv::Mat img = cv::Mat::zeros(h, w, CV_8UC3);
    for( size_t i = 0 ; i < cloud->size() ; i++ )
    {
        int r = i / w;
        int c = i % w;
        
        uint32_t rgb_tmp = cloud->at(i).rgba;
        
        img.at<uchar>(r, c*3+2) = (rgb_tmp >> 16) & 0x0000ff;
        img.at<uchar>(r, c*3+1) = (rgb_tmp >> 8) & 0x0000ff;
        img.at<uchar>(r, c*3+0) = (rgb_tmp) & 0x0000ff;
    }
    
    cv::imwrite(filename+".png", img);
    return 0;
    
    
    std::string in_path("/home/chi/JHUIT/raw/");
    std::string out_path("img_tmp/");
    std::string inst_name("driller_kel_0");
    std::string seq_id("1");;
    
    float elev = ELEV;
    int max_frames = 200;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--s", seq_id);
    pcl::console::parse_argument(argc, argv, "--i", inst_name);
    pcl::console::parse_argument(argc, argv, "--elev", elev);
    pcl::console::parse_argument(argc, argv, "--max", max_frames);
        
    std::cerr << "Loading-"<< inst_name << std::endl;
    if( exists_dir(out_path) == false )
            boost::filesystem::create_directories(out_path);
    
    std::ifstream fp;
    fp.open((in_path+"/PlaneCoef_"+ seq_id + ".txt").c_str(), std::ios::in);
    if( fp.is_open() == false )
    {
        std::cerr << "Failed to open: " << in_path+"/PlaneCoef_"+ seq_id + ".txt" << std::endl;
        exit(0);
    }
    pcl::ModelCoefficients::Ptr planeCoef(new pcl::ModelCoefficients());
    planeCoef->values.resize(4);
    fp >> planeCoef->values[0] >> planeCoef->values[1] >> planeCoef->values[2] >> planeCoef->values[3];
    fp.close();
    
    std::vector< pcl::PointCloud<PointT>::Ptr > cloud_set;
    std::vector< KEYSET > keypt_set;
    std::vector< cv::Mat > keydescr_set;
    
    for( size_t j = 0 ; j < max_frames ; j++ )
    {
        std::stringstream ss;
        ss << j;
        
        if( j % 3 != 0 )
            continue;
        
        std::string filename(in_path + inst_name + "/" + inst_name + "_" + seq_id + "_" + ss.str() + ".pcd");
        
        if( exists_test(filename) == false )
            continue;
        
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(filename, *cloud);
        
        int w = cloud->width;
        int h = cloud->height;
        
        //std::cerr << w << " " << h << " " << cloud->size() << std::endl;
        
        cv::Mat rgb = cv::Mat::zeros(h, w, CV_8UC3);
        cv::Mat gray = cv::Mat::zeros(h, w, CV_8UC1);

        for(size_t i = 0 ; i < cloud->size() ; i++ )
        {
            int r_idx = i / w;
            int c_idx = i % w;
            
            uint32_t rgb_tmp = cloud->at(i).rgba;
            rgb.at<uchar>(r_idx, c_idx*3+2) = (rgb_tmp >> 16) & 0x0000ff;
            rgb.at<uchar>(r_idx, c_idx*3+1) = (rgb_tmp >> 8)  & 0x0000ff;
            rgb.at<uchar>(r_idx, c_idx*3+0) = (rgb_tmp)       & 0x0000ff;  
            
        }
        
        cv::cvtColor(rgb,gray,CV_BGR2GRAY);
        
        //cv::imshow("GRAY", rgb);
        //cv::waitKey();
        
        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
            0, // nFeatures
            4, // nOctaveLayers
            0.04, // contrastThreshold 
            10, //edgeThreshold
            1.6 //sigma
            );
    
        cv::SiftDescriptorExtractor * sift_ext = new cv::SiftDescriptorExtractor();
        
        KEYSET keypoints;
        cv::Mat descriptors;
        sift_det->detect(gray, keypoints);
        sift_ext->compute(gray, keypoints, descriptors);
        printf("%d sift keypoints are found.\n", (int)keypoints.size());

        keypt_set.push_back(keypoints);
        keydescr_set.push_back(descriptors);
        
        cloud = cropCloud(cloud, planeCoef, 0.114);
        cloud_set.push_back(cloud);
        
        //cv::imshow("GRAY", in_rgb);
        //cv::waitKey();
        //cv::imwrite(out_path + ss.str() + ".jpg", in_rgb);
        
        //viewer->addPointCloud(cloud, "cloud");
        //viewer->spin();
        //if( show_keys )
        /*
        {
            cv::Mat out_image;
            cv::drawKeypoints(gray, keypoints, out_image);
            cv::imshow("keypoints", out_image);
            cv::waitKey();
        }
        //*/
        
    }
    
    int total = cloud_set.size();
    /*
    std::vector< pcl::PointCloud<PointT>::Ptr > first_half, second_half;
    std::vector< KEYSET > first_keypt, second_keypt;
    std::vector< cv::Mat > first_keydescr, second_keydescr;
    
    first_half.insert(first_half.end(), cloud_set.begin(), cloud_set.begin()+total/2);
    first_keypt.insert(first_keypt.end(), keypt_set.begin(), keypt_set.begin()+total/2);
    first_keydescr.insert(first_keydescr.end(), keydescr_set.begin(), keydescr_set.begin()+total/2);
    
    second_half.push_back(cloud_set[0]);
    second_keypt.push_back(keypt_set[0]);
    second_keydescr.push_back(keydescr_set[0]);
    for( int i = 1 ; i < total/2 ; i++ )
    {
        second_half.push_back(cloud_set[total-i]);
        second_keypt.push_back(keypt_set[total-i]);
        second_keydescr.push_back(keydescr_set[total-i]);
    }
    
    pcl::PointCloud<PointT>::Ptr first_model = Meshing(first_half, first_keypt, first_keydescr);
    pcl::PointCloud<PointT>::Ptr second_model = Meshing(second_half, second_keypt, second_keydescr);
    pcl::PointCloud<PointT>::Ptr full_model (new pcl::PointCloud<PointT>());
    full_model->insert(full_model->end(), first_model->begin(), first_model->end());
    full_model->insert(full_model->end(), second_model->begin(), second_model->end());
    */
    pcl::PointCloud<PointT>::Ptr full_model = Meshing(cloud_set, keypt_set, keydescr_set);
    
    full_model = cropCloud(full_model, planeCoef, elev);
    
    pcl::io::savePCDFile("temp_full.pcd", *full_model);
    
    pcl::PLYWriter ply_writer;
    ply_writer.write("temp_full.ply", *full_model, true);
    
    /*
    pcl::PointCloud<PointT>::Ptr full_model(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile("temp_full.pcd", *full_model);
    
    std::vector<int> idx_ff;
    pcl::removeNaNFromPointCloud(*full_model, *full_model, idx_ff);
    
    pcl::PointCloud<NormalT>::Ptr full_model_normals (new pcl::PointCloud<NormalT>());
    computeNormals(full_model, full_model_normals, 0.02);
    
    AdjustCloudNormal(full_model, full_model_normals);
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    //viewer->addPointCloud(full_model,  "full_model");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "full_model");
    //viewer->addPointCloudNormals<PointT, NormalT> (full_model, full_model_normals, 5, 0.01, "normals");
    //viewer->spin();
    
    pcl::PointCloud<pcl::PointNormal>::Ptr joint_model (new pcl::PointCloud<pcl::PointNormal>());
    
    pcl::copyPointCloud(*full_model, *joint_model);
    pcl::copyPointCloud(*full_model_normals, *joint_model);
    
    //std::cerr << full_model->size() << " " << full_model_normals->size() << " " << joint_model->size() << std::endl;
    
    //pcl::concatenateFields (*full_model, *full_model_normals, *joint_model);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (joint_model);
    
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh model_mesh;

    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setSearchRadius(0.03);
    gp3.setMaximumSurfaceAngle(M_PI/4);         // 45 degrees
    gp3.setMinimumAngle(M_PI/18);               // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3);              // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (joint_model);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (model_mesh);
    
    //viewer->addPointCloud(full_model, "model");
    viewer->addPolygonMesh(model_mesh, "full_model_mesh");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, "full_model_mesh");
    viewer->spin();
    */
    
    return 0;
}

void AdjustCloudNormal(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<NormalT>::Ptr cloud_normals)
{
    pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
    ComputeCentroid(cloud, center);
    
    myPointXYZ origin = center->at(0);
    std::cerr << origin.x << " " << origin.y << " " << origin.z << std::endl;
    pcl::transformPointCloud(*cloud, *cloud, Eigen::Vector3f(-origin.x, -origin.y, -origin.z), Eigen::Quaternion<float> (1,0,0,0));
    
    int num = cloud->points.size();
    float diffx, diffy, diffz, dist, theta;
    for( int j = 0; j < num ; j++ )
    {
        PointT temp = cloud->at(j);
        NormalT temp_normals = cloud_normals->at(j);
        diffx = temp.x;
        diffy = temp.y;
        diffz = temp.z;
        dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz );
		
        theta = acos( (diffx*temp_normals.normal_x + diffy*temp_normals.normal_y + diffz*temp_normals.normal_z)/dist );
        if( theta > PI/2)
        {
            cloud_normals->at(j).normal_x = -cloud_normals->at(j).normal_x;
            cloud_normals->at(j).normal_y = -cloud_normals->at(j).normal_y;
            cloud_normals->at(j).normal_z = -cloud_normals->at(j).normal_z;
        }
    }
}

pcl::CorrespondencesPtr matchSIFT(const cv::Mat &descr1, const cv::Mat &descr2)
{
    pcl::CorrespondencesPtr corrs (new pcl::Correspondences());
    
    cv::flann::Index sift_tree;
    cv::flann::LinearIndexParams params(descr1, params);

    for( int i = 0 ; i < descr2.rows ; i++ )
    {
        std::vector<int> idxs;
        std::vector<float> dists;
        sift_tree.knnSearch(descr2.row(i), idxs, dists, 2, cv::flann::SearchParams());
        if( dists[0] / dists[1] < SIFT_RATIO )
        {
            pcl::Correspondence cur_corr (idxs[0], i, 0);
            //std::cerr << idxs[0] << " " << i << std::endl;
            corrs->push_back (cur_corr);
        }
    }
    return corrs;
}

pcl::PointCloud<PointT>::Ptr Meshing(const std::vector< pcl::PointCloud<PointT>::Ptr > &cloud_set,
                                     const std::vector< KEYSET > &keypt_set,
                                     const std::vector< cv::Mat > &keydescr_set)
{
    int frame_num = cloud_set.size();
    if( frame_num <= 0 )
        return pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
    
    PointT dummy;
    dummy.x = 0;dummy.y = 0;dummy.z = 0;
    std::vector< pcl::PointCloud<PointT>::Ptr > keypt_cloud(frame_num);
    std::vector< pcl::PointCloud<PointT>::Ptr > cloud_f_set(frame_num);
    std::vector< cv::Mat > key_active_set(frame_num);
    for(int i = 0 ; i < frame_num ; i++ )
    {
        int w = cloud_set[i]->width;
        int h = cloud_set[i]->height;
        keypt_cloud[i] = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>());
        
        std::vector< cv::Mat > mat_tmp;
        for( size_t j = 0 ; j < keypt_set[i].size() ; j++ )
        {
            int cur_y = round(keypt_set[i][j].pt.y);
            int cur_x = round(keypt_set[i][j].pt.x);
            if( cur_y < h && cur_y >= 0 && cur_x < w && cur_x >= 0)
            {
                int idx = cur_y*w + cur_x;
                if( pcl_isfinite(cloud_set[i]->at(idx).z) == true )
                {
                    keypt_cloud[i]->push_back(cloud_set[i]->at(idx));
                    mat_tmp.push_back(keydescr_set[i].row(j));
                }
            }
            else
                std::cerr << cur_y << " " << cur_x << " " << h << " " << w << std::endl;
        }
        cv::vconcat(mat_tmp, key_active_set[i]);
        
        cloud_f_set[i] = FilterBoundary(cloud_set[i]);
        //std::cerr << keypt_cloud[i]->size() << "***" << key_active_set[i].rows << std::endl; 
    }
            
    std::vector< Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > tran_set(frame_num);
    tran_set[0] = Eigen::Matrix4f::Identity();
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
    
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> crsc;
    crsc.setInlierThreshold( 0.005 );
    crsc.setMaximumIterations( 2000 );
    
    std::vector< pcl::PointCloud<PointT>::Ptr > tran_model_set(frame_num);
    pcl::PointCloud<PointT>::Ptr full_model(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cloud_f_set[0], *full_model);
    tran_model_set[0] = cloud_f_set[0];
    
    viewer->removePointCloud("full");
    viewer->addPointCloud(full_model, "full");
    viewer->spin();    
    for(int i = 1 ; i < frame_num ; i++ )
    {
        std::cerr << "Frame --- " << i << std::endl;
        pcl::CorrespondencesPtr cur_corrs = matchSIFT(key_active_set[i], key_active_set[i-1]);
        
        crsc.setInputSource( keypt_cloud[i] );
	crsc.setInputTarget( keypt_cloud[i-1] );
        
	crsc.setInputCorrespondences( cur_corrs ); 
        pcl::CorrespondencesPtr in_corr (new pcl::Correspondences());
	crsc.getCorrespondences( *in_corr );
    
        Eigen::Matrix4f init_guess = crsc.getBestTransformation();
        init_guess = tran_set[i-1] * init_guess;
                
        tran_set[i] = DenseColorICP(cloud_f_set[i], tran_model_set[i-1], init_guess);
        
        pcl::PointCloud<PointT>::Ptr Final (new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*cloud_f_set[i], *Final, tran_set[i]);
        
        tran_model_set[i] = Final;
        
        full_model->insert(full_model->end(), Final->begin(), Final->end());
        
        viewer->removePointCloud("full");
        viewer->addPointCloud(full_model, "full");
        viewer->spinOnce(1);
    }
    
    std::cerr << "Registration Done!!!" << std::endl; 
    
    pcl::PointCloud<PointT>::Ptr down_model(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(full_model);
    sor.setLeafSize(0.001, 0.001, 0.001);
    sor.filter(*down_model);
    
    /*
    //*
    pcl::search::KdTree<PointT>::Ptr mls_tree (new pcl::search::KdTree<PointT>);
    pcl::MovingLeastSquares<PointT, PointT> mls;
    // Set parameters
    mls.setInputCloud (down_model);
    mls.setComputeNormals(false);
    mls.setPolynomialFit(true);
    mls.setSearchMethod (mls_tree);
    mls.setSearchRadius (0.01);
    // Reconstruct
    pcl::PointCloud<PointT>::Ptr down_model_f(new pcl::PointCloud<PointT>());
    mls.process (*down_model_f);
    */
    //std::cerr << "Smoothing Done!!!" << std::endl; 
    
    //*/
    //viewer->removePointCloud("full");
    //viewer->addPointCloud(down_model_f, "down");
    //viewer->spin();
    
    return down_model;
}


pcl::PointCloud<PointT>::Ptr cropCloud(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::ModelCoefficients::Ptr planeCoef, float elev)
{
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    pcl::copyPointCloud(*cloud, *cloud_f);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud_f);
    proj.setModelCoefficients (planeCoef);

    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>());
    proj.filter (*cloud_projected);
    for(size_t i = 0 ; i < cloud_f->size() ; i++ )
    {
        if( pcl_isfinite(cloud_f->at(i).z) == true )
        {
            float diffx = cloud_f->at(i).x-cloud_projected->at(i).x;
            float diffy = cloud_f->at(i).y-cloud_projected->at(i).y;
            float diffz = cloud_f->at(i).z-cloud_projected->at(i).z;

            float dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz);
            if ( dist <= elev )
            {
                cloud_f->at(i).x = NAN;
                cloud_f->at(i).y = NAN;
                cloud_f->at(i).z = NAN;
            }
        }
    }
    cloud_f->is_dense = false;
    return cloud_f;
}

pcl::PointCloud<PointT>::Ptr FilterBoundary(const pcl::PointCloud<PointT>::Ptr cloud)
{
    float threshold = 0.04;
    int BLen = 5;
    
    int w = cloud->width;
    int h = cloud->height;
    int num = cloud->size();
    cv::Mat depthmap = cv::Mat::ones(h+2*BLen, w+2*BLen, CV_32FC1)*-1000;
    
    for(int i = 0 ; i < num; i++ )
    {
        if( pcl_isfinite(cloud->at(i).z) == true )
        {
            int r_idx = i / w + BLen;
            int c_idx = i % w + BLen;
            depthmap.at<float>(r_idx, c_idx) = cloud->at(i).z;
        }
    }
    
    pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());
    for(int i=0 ; i<num; i++ ){
        
        int row = i / w + BLen;
        int col = i % w + BLen;
        if( pcl_isfinite(cloud->at(i).z) == true )
        {
        
            float zCur = depthmap.at<float>(row, col);

            if( fabs(depthmap.at<float>(row-BLen, col)-zCur) > threshold || fabs(depthmap.at<float>(row+BLen, col)-zCur) > threshold
                || fabs(zCur-depthmap.at<float>(row, col-BLen)) > threshold || fabs(zCur-depthmap.at<float>(row, col+BLen)) > threshold)
                        ;//Boundary->push_back(cloud_rgb->points[i]);
            else
                cloud_f->push_back(cloud->at(i));
        }
    }
    return cloud_f;
}

//
Eigen::Matrix4f DenseColorICP(const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &initial_guess)
{
    //if the ICP fail to converge, return false. Otherwise, return true!
    size_t Iter_num = 50;
    float icp_inlier_threshold = 0.01;
    pcl::PointCloud<PointT>::Ptr init_source(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*source, *init_source, initial_guess);
    
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr source_hue(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr target_hue(new pcl::PointCloud<pcl::PointXYZHSV>);
    ExtractHue(init_source, source_hue);
    ExtractHue(target, target_hue);

    Eigen::Matrix4f final_tran = initial_guess;
    pcl::search::KdTree<pcl::PointXYZHSV> target_tree;
    target_tree.setInputCloud(target_hue);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZHSV, pcl::PointXYZHSV> SVD;
    
    double last_error = 100000;
    for( size_t iter = 0 ; iter < Iter_num ; iter++ )
    {
        //In each round of ICP, transform the model_hue cloud
        float diffh, diffs, diffv, diffsum;
        pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
        for( size_t i = 0 ; i < source_hue->size() ; i++ )
        {
            std::vector<int> pointIdx;
            std::vector<float> pointDistance;
            if ( pcl_isfinite(source_hue->at(i).z) && target_tree.radiusSearch (source_hue->at(i), 0.01, pointIdx, pointDistance) > 0 )
            {
                float diffmin = 1000;
                int diffmin_idx = -1;
                for (size_t j = 0; j < pointIdx.size (); j++)
                {
                    //std::cerr<<"Finding..."<<scene_hue->points[pointIdx[j]].h <<" "<<model_hue->points[i].h<<std::endl;
                    if( pointDistance.at(j) < icp_inlier_threshold )
                    {
                        diffh = std::min( fabs(target_hue->points[pointIdx[j]].h  - source_hue->points[i].h), 
                                std::min(fabs(target_hue->points[pointIdx[j]].h - 1 - source_hue->points[i].h), fabs(target_hue->points[pointIdx[j]].h + 1 - source_hue->points[i].h)));
                        diffs = fabs(target_hue->points[pointIdx[j]].s - source_hue->points[i].s);
                        diffv = fabs(target_hue->points[pointIdx[j]].v - source_hue->points[i].v);
                        
                        diffsum = 2*diffh + 2*diffs + diffv;
                        if( diffmin > diffsum )
                        {
                            diffmin = diffsum;
                            diffmin_idx = j;
                        }
                    }	
                }
                //std::cerr<<diffcurvature<<" ";
                //std::cerr<<diffmin<<" ";
                if( diffmin <= 0.1 )
                {
                    pcl::Correspondence temp;
                    temp.index_query = i;
                    temp.index_match = pointIdx[diffmin_idx];
                    temp.distance = pointDistance.at(diffmin_idx);
                    model_scene_corrs->push_back(temp);
                }
            }
        }
        Eigen::Matrix4f svdRt;

        SVD.estimateRigidTransformation(*source_hue, *target_hue, *model_scene_corrs, svdRt);
        
        pcl::transformPointCloud(*source_hue, *source_hue, svdRt);
        
        final_tran = svdRt * final_tran ;
        std::cerr<<"Ratio "<<(model_scene_corrs->size()+0.0) / source_hue->size()<<std::endl;
        if( (model_scene_corrs->size()+0.0) / source_hue->size() >= 0.2 ) //sufficient inlier found
        {
            size_t corrs = model_scene_corrs->size();
            double this_error=0;
            for( size_t j = 0; j < corrs; j++ )
            {
                pcl::PointXYZHSV source_pt = source_hue->points[model_scene_corrs->at(j).index_query];
                pcl::PointXYZHSV target_pt = target_hue->points[model_scene_corrs->at(j).index_match];
                double diffx = source_pt.x - target_pt.x, diffy = source_pt.y - target_pt.y, diffz = source_pt.z - target_pt.z;
                double dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz );

                this_error += dist;
            }
            this_error = this_error / corrs;
            if( fabs(this_error - last_error) < 1e-6 )  //Convergence reach
            {
                std::cerr<<"Convergence Reached. Error: "<<this_error<<std::endl;
                std::cerr<<"Iter Num: "<<iter<<std::endl;
                return final_tran;
            }
            else
                last_error = this_error;
        }
    }

    return final_tran;
}

//*/



