#ifndef SP_SEGMENTER_PLANE
#define SP_SEGMENTER_PLANE

inline
pcl::PointCloud<PointT>::Ptr getPlane (const pcl::PointCloud<PointT>::Ptr &inputCloud, const pcl::PointCloud<PointT>::Ptr & scene, const float &T)
{
    pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(T);

    seg.setInputCloud(inputCloud);
    seg.segment(*inliers, *plane_coef);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (scene);
    proj.setModelCoefficients (plane_coef);

    pcl::PointCloud<PointT>::Ptr scene_projected(new pcl::PointCloud<PointT>());
    proj.filter (*scene_projected);
    return scene_projected;
}

inline
pcl::PointCloud<PointT>::Ptr removePlane(const pcl::PointCloud<PointT>::Ptr & scene, const double aboveTable=0.025,const float T = 0.02)
{
    //Downsample the point cloud for faster plane segmentation
    pcl::PointCloud<PointT>::Ptr downsampledPointCloud (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (scene);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*downsampledPointCloud); 

    pcl::PointCloud<PointT>::Ptr scene_projected = getPlane(downsampledPointCloud, scene, T);

    pcl::PointCloud<PointT>::iterator it_ori = scene->begin();
    pcl::PointCloud<PointT>::iterator it_proj = scene_projected->begin();
    
    pcl::PointCloud<PointT>::Ptr scene_f(new pcl::PointCloud<PointT>());
    for( int base = 0 ; it_ori < scene->end() && it_proj < scene_projected->end() ; it_ori++, it_proj++, base++ )
    {
        float diffx = it_ori->x-it_proj->x;
        float diffy = it_ori->y-it_proj->y;
        float diffz = it_ori->z-it_proj->z;

        if( diffx * it_ori->x + diffy * it_ori->y + diffz * it_ori->z >= 0 )
            continue;
        //distance from the point to the plane
        float dist = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
        
        if ( dist >= aboveTable )//fabs((*it_ori).x) <= 0.1 && fabs((*it_ori).y) <= 0.1 )
            scene_f->push_back(*it_ori);
    }
    
    return scene_f;
}


#endif /* end of include guard: SP_SEGMENTER_PLANE */
