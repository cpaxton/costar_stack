#include "sp_segmenter/utility/utility.h"

pcl::visualization::PCLVisualizer::Ptr viewer;//(new pcl::visualization::PCLVisualizer ());
    
pcl::PointCloud<PointT>::Ptr CropCloud(const pcl::PointCloud<PointT>::Ptr scene, float elev = 0.03, float z_max = 1.3, float z_min = 0.1, float x_span = 0.5, float y_span = 0.5)
{   
    int w = scene->width;
    
    std::vector<int> idx_f;
    pcl::PointCloud<PointT>::Ptr scene_f(new pcl::PointCloud<PointT>());
    pcl::removeNaNFromPointCloud(*scene, *scene_f, idx_f);
    
    // do the plane segmentation
    pcl::ModelCoefficients::Ptr coef(new pcl::ModelCoefficients);
    
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(scene_f);
    seg.segment(*inliers, *coef);

    /*
    pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());
    pcl::ExtractIndices<PointT> extract;
    
    extract.setInputCloud (scene_f);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane);
    viewer->addPointCloud(plane, "plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane");
    viewer->spin();
    viewer->removePointCloud("plane");
    // crop certain spatial region above the plane
    */
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (scene_f);
    proj.setModelCoefficients (coef);
    
    pcl::PointCloud<PointT>::Ptr scene_f_projected(new pcl::PointCloud<PointT>());
    proj.filter (*scene_f_projected);

    pcl::PointCloud<PointT>::iterator it_ori = scene_f->begin();
    pcl::PointCloud<PointT>::iterator it_proj = scene_f_projected->begin();
    int max_r = -1000, min_r = 1000, max_c = -1000, min_c = 1000;
    for( int base = 0 ; it_ori < scene_f->end(), it_proj < scene_f_projected->end() ; it_ori++, it_proj++, base++ )
    {
        float diffx = (*it_ori).x-(*it_proj).x;
        float diffy = (*it_ori).y-(*it_proj).y;
        float diffz = (*it_ori).z-(*it_proj).z;
        
        //distance from the point to the plane
        float dist = sqrt( diffx*diffx + diffy*diffy + diffz*diffz);
        //std::cerr << dist << " ";
        if ( dist > elev && (*it_ori).z <= z_max && (*it_ori).z >= z_min && fabs((*it_ori).x) <= x_span && fabs((*it_ori).y) <= y_span )
        {
            int idx_ori = idx_f[base];
            int r = idx_ori / w;
            int c = idx_ori % w;
            if( max_r < r ) max_r = r;
            if( max_c < c ) max_c = c;
            
            if( min_r > r ) min_r = r;
            if( min_c > c ) min_c = c; 
        }
    }
    std::cerr << min_r << " " << max_r << " " << min_c << " " << max_c << std::endl;
    
    pcl::PointCloud<PointT>::Ptr scene_cropped(new pcl::PointCloud<PointT>());
    int c_range = max_c - min_c + 1;
    for( int r = min_r ; r <= max_r ; r++ )
    {
       int s_idx = r*w + min_c;
       scene_cropped->insert(scene_cropped->end(), scene->begin()+s_idx, scene->begin()+s_idx+c_range);
    }
    
    scene_cropped->width = scene->width;
    scene_cropped->height = scene->height;
    scene_cropped->is_dense = false;
    
    //viewer->addPointCloud(scene_cropped, "cropped_cloud");
    //viewer->spin();
    return scene_cropped;
}
/*
int main(int argc, char** argv)
{
    viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::io::loadPCDFile(argv[1], *cloud);
    
    //viewer->addPointCloud(cloud, "cloud");
    double t1, t2;
    t1 = get_wall_time();    
    pcl::PointCloud<PointT>::Ptr cropped_cloud = CropCloud(cloud, 0.03, 1.3, 0.2, 0.5, 0.5);
    t2 = get_wall_time();
    std::cerr << t2 - t1 << std::endl;
    
    viewer->addPointCloud(cropped_cloud, "cropped_cloud");
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cropped_cloud");
    viewer->spin();
    return 1;
}
 * */
//*
int main(int argc, char** argv)
{
    if( argc <= 1 )
    {
        std::cerr << "Input point cloud name!" << std::endl;
        return 0;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    
    bool show_normals = false;
    if( pcl::console::find_switch(argc, argv, "-n") == true )
        show_normals = true;
    
    float radius = 0.03;
    pcl::console::parse_argument(argc, argv, "--r", radius);
          
    std::string path;
    std::vector<std::string> pcd_files;
    std::vector<std::string> normal_files;
    if( pcl::console::parse_argument(argc, argv, "--p", path) > 0 )
    {
        std::vector<std::string> temp_files;
        getNonNormalPCDFiles(path, temp_files);
        for( std::vector<std::string>::iterator itt = temp_files.begin() ; itt < temp_files.end() ; itt++ )
        {
            pcd_files.push_back(path+"/"+(*itt));
            normal_files.push_back(path+"/normal_"+(*itt));
        }
    }
    else
    {
        std::string name(argv[1]);
        pcd_files.push_back(name);
    }
   
    for( size_t i = 0 ; i < pcd_files.size() ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        
        viewer->removeAllPointClouds();
        viewer->addPointCloud(cloud, "cloud");
        
        if( show_normals == true )
        {
            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(normal_files[i], *cloud_normals);
            viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 3, 0.02, "cloud_normals");
        }
        
        if( pcl::console::find_switch(argc, argv, "-w") == true )
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");
        
        pcl::search::KdTree<PointT> tree;
	tree.setInputCloud (cloud);
        for( pcl::PointCloud<PointT>::iterator it = cloud->begin() ; it < cloud->end() ; it++ )
        {
            std::vector<int> idxs;
            std::vector<float> dist;
            tree.radiusSearch(*it, radius, idxs, dist, cloud->size());
	    
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
            inliers->indices = idxs;

            pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
            // Create the filtering object
            pcl::ExtractIndices<PointT> extract;
             // Extract the inliers
            extract.setInputCloud (cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*down_cloud);
            
            viewer->removePointCloud("down_cloud");
            viewer->addPointCloud(down_cloud, "down_cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "down_cloud");
            viewer->spin();
        }
        
        viewer->spin();
    }
    
    return 1;
}
// */
