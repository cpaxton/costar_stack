#include "../include/utility.h"
#include <pcl/surface/bilateral_upsampling.h>

void get2DImg(const pcl::PointCloud<PointT>::Ptr cloud, cv::Mat &rgb, cv::Mat &depth, float ss)
{
    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++ )
    {
        float x = (*it).x, y = (*it).y, z = (*it).z;
        if( max_x < x ) max_x = x;
        if( max_y < y ) max_y = y;
        
        if( min_x > x ) min_x = x;
        if( min_y > y ) min_y = y;
    }
    size_t rows, cols;
    rows = ceil((max_y+EPS-min_y)/ss);
    cols = ceil((max_x+EPS-min_x)/ss);
    std::cerr << rows << " " << cols << std::endl;
    rgb = cv::Mat::zeros(rows, cols, CV_8UC3);
    depth = -100 * cv::Mat::ones(rows, cols, CV_32FC1);
    
    // go through all points
    int count = 0;
    for( pcl::PointCloud<PointT>::const_iterator it = cloud->begin() ; it < cloud->end() ; it++, count++ )
    {
        int x = floor(((*it).x - min_x) / ss);
        int y = floor(((*it).y - min_y) / ss);
        if( (*it).z > depth.at<float>(y, x) )
        {
            depth.at<float>(y, x) = (*it).z;
            // bgr
            int x_idx = 3 * x; 
            //uint32_t tmp = ;
            //uint32_t rgb_tmp = *reinterpret_cast<int*>(&(tmp));
            rgb.at<uchar>(y, x_idx) = (*it).rgba & 0x0000ff;
            rgb.at<uchar>(y, x_idx+1) = ((*it).rgba >> 8) & 0x0000ff;
            rgb.at<uchar>(y, x_idx+2) = ((*it).rgba >> 16) & 0x0000ff;
        }
    }
}

void genViews1(const pcl::PointCloud<PointT>::Ptr full_cloud, const poseVec &rot_set, 
        CloudSet &partial_set, poseVec &partial_ground_tran, float s1, float s2)
{
    float model_resol = 0;;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    if( s1 > 0 )
    {
        pcl::VoxelGrid<PointT> sor;
        sor.setInputCloud(full_cloud);
        sor.setLeafSize(s1, s1, s1);
        sor.filter(*cloud);
        std::cerr << full_cloud->size() << "->" << cloud->size() << std::endl;
        model_resol = s1;
    }
    else
    {
        pcl::copyPointCloud(*full_cloud, *cloud);
        model_resol = computeCloudResolution(cloud);
    }
    std::cerr << "Model Resolution: " << model_resol << std::endl;
   
    //shift the centroid of the cloud to origin
    pcl::PointCloud<myPointXYZ>::Ptr center(new pcl::PointCloud<myPointXYZ>());
    ComputeCentroid(cloud, center);
    Eigen::Matrix4f toOrigin = Eigen::Matrix4f::Identity();
    toOrigin(0, 3) = -center->at(0).x;
    toOrigin(1, 3) = -center->at(0).y;
    toOrigin(2, 3) = -center->at(0).z;
    pcl::transformPointCloud(*cloud, *cloud, toOrigin);
    
    int num = rot_set.size();
    partial_set.resize(num);
    partial_ground_tran.resize(num);
    
    //#pragma omp parallel for schedule(dynamic,1)
    for( int i = 0 ; i < num ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr tran_cloud(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*cloud, *tran_cloud, rot_set[i]);
        
        pcl::PointCloud<PointT>::Ptr partial_cloud = createPartialView(tran_cloud, s2);
        cv::Mat rgb, depth;
        get2DImg(tran_cloud, rgb, depth, s2);
        cv::imshow("rgb", rgb);
        cv::waitKey();
        
        // compute centers for initialization
        pcl::PointCloud<myPointXYZ>::Ptr partial_center(new pcl::PointCloud<myPointXYZ>());
        ComputeCentroid(partial_cloud, partial_center);
        Eigen::Matrix4f toOrigin1 = Eigen::Matrix4f::Identity();
        toOrigin1(0, 3) = -partial_center->at(0).x;
        toOrigin1(1, 3) = -partial_center->at(0).y;
        toOrigin1(2, 3) = -partial_center->at(0).z;
        pcl::transformPointCloud(*partial_cloud, *partial_cloud, toOrigin1); //push center to (0, 0, 0)
        partial_set[i] = partial_cloud;
        
        partial_ground_tran[i] = rot_set[i];
        partial_ground_tran[i](0, 3) = toOrigin1(0, 3);
        partial_ground_tran[i](1, 3) = toOrigin1(1, 3);
        partial_ground_tran[i](2, 3) = toOrigin1(2, 3);
        

    }
    std::cerr<<partial_set.size()<<std::endl;
    
}

int main(int argc, char** argv)
{
    std::string path(argv[1]);
    
    std::string ply_file(path+"/textured_meshes/optimized_tsdf_textured_mesh.ply");
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointT>::Ptr up_cloud(new pcl::PointCloud<PointT>());
    
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(ply_file, triangles);
    std::cerr << triangles.polygons.size() << std::endl;
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);  
    
    std::cerr << cloud->size() << std::endl;
    float s1 = 0.001;
    pcl::BilateralUpsampling<PointT,PointT> bb;
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(s1, s1, s1);
    sor.filter(*down_cloud);
    std::cerr << down_cloud->size() << std::endl;
    
    bb.setInputCloud(down_cloud);
    bb.process(*up_cloud);
    std::cerr << up_cloud->size() << std::endl;
    
    /*
    poseVec rot_set = readRots("uniRot_txt/SemiRot_1392.txt");
    
    CloudSet partial_set;
    poseVec partial_ground_tran;
    genViews1(cloud, rot_set, partial_set, partial_ground_tran, -1, 0.001);
    */
    
    /*      
    std::string path;
    std::vector<std::string> pcd_files;
    if( pcl::console::parse_argument(argc, argv, "--p", path) > 0 )
    {
        std::vector<std::string> temp_files;
        getNonNormalPCDFiles(path, temp_files);
        for( std::vector<std::string>::iterator itt = temp_files.begin() ; itt < temp_files.end() ; itt++ )
            pcd_files.push_back(path+"/"+(*itt));
    }
    
    cv::namedWindow("rgb");
    for( size_t i = 0 ; i < pcd_files.size() ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        
        cv::Mat rgb, depth;
        get2DImg(cloud, rgb, depth, 0.003);
        cv::imshow("rgb", rgb);
        cv::waitKey();
    }
    */
    return 1;
}

/*
int main(int argc, char ** argv) 
{
    vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
    readerQuery->SetFileName (argv[1]);
    vtkSmartPointer<vtkPolyData> polydata = readerQuery->GetOutput();
    polydata->Update();

    pcl::visualization::PCLVisualizer vis("Visualizer");
    vis.initCameraParameters();
    
    vis.addModelFromPolyData (polydata, "mesh1", 0);
    vis.setCameraPosition(0.0, 0.0, 1.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0, 0);
    
    vis.addCoordinateSystem(0.1);
    vis.setRepresentationToSurfaceForAllActors();
    vis.spin();
    
    std::vector< pcl::PointCloud< pcl::PointXYZ >, Eigen::aligned_allocator< pcl::PointCloud< pcl::PointXYZ > > > clouds;
    std::vector< Eigen::Matrix4f, Eigen::aligned_allocator< Eigen::Matrix4f > >  poses;
    std::vector< float > enthropies;
    vis.renderViewTesselatedSphere(256, 256, clouds, poses, enthropies, 0);
    
    pcl::visualization::PCLVisualizer vis1("Visualizer1");
    vis1.initCameraParameters();
    vis1.addCoordinateSystem(0.1);
    std::cerr << clouds.size() << std::endl;
    for( int i = 0 ; i < clouds.size() ; i++ )
    {
        pcl::PointCloud< pcl::PointXYZ >::Ptr temp(new pcl::PointCloud< pcl::PointXYZ >());
        pcl::copyPointCloud(clouds[i], *temp);
        vis1.removeAllPointClouds();
        vis1.addPointCloud(temp, "cloud_out");
        vis1.spin();
    }
    
    
    
    return 0;
}

*/