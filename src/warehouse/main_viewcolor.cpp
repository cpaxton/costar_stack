#include "../include/utility.h"
#include "../include/features.h"

int color[1000][3] = {0};
float hs_cell_scale = 0.125;
int len_h = 8, len_s = 8, len_i = 8;
int getHSPoolIdx(float hsi[3])
{
    
    float h = hsi[0];
    float s = hsi[1];
    float i = hsi[2];
    if( h == 1.0 )  h = 0.9999;
    if( s == 1.0 )  s = 0.9999;
    if( i == 1.0 )  i = 0.9999;
    
    int idx_h = floor( h / hs_cell_scale );
    int idx_s = floor( s / hs_cell_scale );
    int idx_i = floor( i / hs_cell_scale );
    //return idx_h*len_s*len_i + idx_s*len_i+idx_i;
    return idx_i;
}

pcl::PointCloud<PointT>::Ptr mapHSI(const pcl::PointCloud<PointT>::Ptr cloud)
{
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud);
    
    pcl::PointCloud<PointT>::Ptr show_cloud(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < cloud->size() ; i++ )
    {
        std::vector<int> ind;
        std::vector<float> dist;
        kdtree->radiusSearch(cloud->at(i), 0.01, ind, dist, cloud->size());
        uint32_t rgb = *reinterpret_cast<int*>(&(cloud->at(i).rgba));
        int rgba[3];
        rgba[0] = (rgb >> 16) & 0x0000ff;
        rgba[1] = (rgb >> 8)  & 0x0000ff;
        rgba[2] = (rgb)       & 0x0000ff;
        
        float pivot_sum = sqrt(rgba[0]*rgba[0]+rgba[1]*rgba[1]+rgba[2]*rgba[2]);
        float pivot_norm[3];
        pivot_norm[0] = rgba[0] / pivot_sum;
        pivot_norm[1] = rgba[1]/ pivot_sum;
        pivot_norm[2] = rgba[2]/ pivot_sum;
        
        float max_val = 0, min_val = 1000;
        float max_dist = 0;
        for( int j = 1 ; j < ind.size() ; j++ )
        {
            uint32_t cur_rgb = *reinterpret_cast<int*>(&(cloud->at(ind[j]).rgba));
            int cur_rgba[3];
            cur_rgba[0] = (cur_rgb >> 16) & 0x0000ff;
            cur_rgba[1] = (cur_rgb >> 8)  & 0x0000ff;
            cur_rgba[2] = (cur_rgb)       & 0x0000ff;
            float sum = sqrt(cur_rgba[0]*cur_rgba[0]+cur_rgba[1]*cur_rgba[1]+cur_rgba[2]*cur_rgba[2]);
            float cur_norm[3];
            cur_norm[0] = cur_rgba[0] / sum;
            cur_norm[1] = cur_rgba[1]/ sum;
            cur_norm[2] = cur_rgba[2]/ sum;
            
            //float val = cur_rgba[0]+cur_rgba[1]+cur_rgba[2];
            //float cur_dist = (rgba[0]-cur_rgba[0])*(rgba[0]-cur_rgba[0])+(rgba[1]-cur_rgba[1])*(rgba[1]-cur_rgba[1])+(rgba[2]-cur_rgba[2])*(rgba[2]-cur_rgba[2]);
            float cur_dist = (pivot_norm[0]-cur_norm[0])*(pivot_norm[0]-cur_norm[0])+(pivot_norm[1]-cur_norm[1])*(pivot_norm[1]-cur_norm[1])+(pivot_norm[2]-cur_norm[2])*(pivot_norm[2]-cur_norm[2]);
            //if( max_val < val ) max_val = val;
            //if( min_val > val ) min_val = val;        
            if( cur_dist > max_dist )
                max_dist = cur_dist;
        }
        
        /*
        uint32_t rgb = *reinterpret_cast<int*>(&(cloud->at(i).rgba));
        int rgba[3];
        float hsi[3];
        rgba[0] = (rgb >> 16) & 0x0000ff;
        rgba[1] = (rgb >> 8)  & 0x0000ff;
        rgba[2] = (rgb)       & 0x0000ff;
        RGBToHSI(rgba,hsi);
        */
        
        
        //int idx = floor((max_val - min_val)/3 / 7.0);
        int idx = floor(max_dist / 0.05);
        //int idx = getHSPoolIdx(hsi);
        PointT tmp = cloud->at(i);
        
        tmp.rgba = color[idx][0] << 16 | color[idx][1] << 8 | color[idx][2];
        show_cloud->push_back(tmp);
    }
    return show_cloud;
}

void genColor(int c[][3], int len)
{
    c[0][0] = 255;c[0][1] = 0;c[0][2] = 0;
    c[1][0] = 0;c[1][1] = 255;c[1][2] = 0;
    c[2][0] = 0;c[2][1] = 0;c[2][2] = 255;
    c[3][0] = 255;c[3][1] = 255;c[3][2] = 0;
    c[4][0] = 255;c[4][1] = 0;c[4][2] = 255;
    c[5][0] = 0;c[5][1] = 255;c[5][2] = 255;
    c[6][0] = 255;c[6][1] = 128;c[6][2] = 0;
    c[7][0] = 255;c[7][1] = 0;c[7][2] = 128;
    
    for(size_t i = 8 ; i < len ; i++ )
    {
        c[i][0] = rand() % 255;
        c[i][1] = rand() % 255;
        c[i][2] = rand() % 255;
        //std::cerr << c[i][0] << " " << c[i][1] << " " << c[i][2] << std::endl;
    }
}

pcl::PointCloud<PointT>::Ptr colorCloud(const pcl::PointCloud<PointT>::Ptr cloud, const cv::Mat &cloud_idx)
{
    if( cloud->size() != cloud_idx.rows )
    {
        std::cerr << "cloud->size() != idx.rows!" << std::endl;
        exit(0);
    }
    pcl::PointCloud<PointT>::Ptr show_cloud(new pcl::PointCloud<PointT>());
    for(size_t i = 0 ; i < cloud->size() ; i++ )
    {
        int idx = cloud_idx.at<int>(i, 0);
        PointT tmp = cloud->at(i);
        
        tmp.rgba = color[idx][0] << 16 | color[idx][1] << 8 | color[idx][2];
        show_cloud->push_back(tmp);
    }
    return show_cloud;
}

int main(int argc, char** argv)
{
    Pooler_L0 normal_pooler(-1);
    normal_pooler.LoadSeedsPool("UW_dict/kcenter_so2_60.cvmat");
    
    genColor(color, normal_pooler.getGenericPoolLen());
    
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
         
    std::string path;
    pcl::console::parse_argument(argc, argv, "--p", path);
    
    std::vector<std::string> pcd_files;
    std::vector<std::string> normal_files;
    
    std::vector<std::string> temp_files;
    getNonNormalPCDFiles(path, temp_files);
    for( std::vector<std::string>::iterator itt = temp_files.begin() ; itt < temp_files.end() ; itt++ )
    {
        pcd_files.push_back(path+"/"+(*itt));
        normal_files.push_back(path+"/normal_"+(*itt));
    }
    
    for( size_t i = 0 ; i < pcd_files.size() ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        std::cerr << "Loading: " << pcd_files[i] << std::endl;
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        
        //viewer->removeAllPointClouds();
        //viewer->addPointCloud(cloud, "cloud");
        
        pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
        pcl::io::loadPCDFile(normal_files[i], *cloud_normals);
        //viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 3, 0.02, "cloud_normals");
        
        MulInfoT down1 = convertPCD(cloud, cloud_normals);
        cv::Mat normal_idx = normal_pooler.getGenericPoolMat(down1.normal);
        
        pcl::PointCloud<PointT>::Ptr show_cloud = colorCloud(cloud, normal_idx);
        
        viewer->removeAllPointClouds();
        viewer->addPointCloud(show_cloud, "show_cloud");
        viewer->spin();
    }
    
    return 1;
}

/*
int main(int argc, char** argv)
{
    genColor(color, 100);
    if( argc <= 1 )
    {
        std::cerr << "Input point cloud name!" << std::endl;
        return 0;
    }
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ());
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(0.1);
    
    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer ());
    viewer1->initCameraParameters();
    viewer1->addCoordinateSystem(0.1);
    
    bool show_normals = false;
    if( pcl::console::find_switch(argc, argv, "-n") == true )
        show_normals = true;
          
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
    
    pcl::PointCloud<PointT>::Ptr mesh(new pcl::PointCloud<PointT>());
    std::string ply_file(path+"/../textured_meshes/optimized_tsdf_textured_mesh.ply");
    
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(ply_file, triangles);
    pcl::fromPCLPointCloud2(triangles.cloud, *mesh);
    pcl::PointCloud<PointT>::Ptr mesh_hsi = mapHSI(mesh);
    viewer1->addPointCloud(mesh_hsi, "mesh_hsi");
    viewer1->spin();
    
    for( size_t i = 0 ; i < pcd_files.size() ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
        pcl::io::loadPCDFile(pcd_files[i], *cloud);
        pcl::PointCloud<PointT>::Ptr cloud_hsi = mapHSI(cloud);
        
        viewer->removePointCloud("cloud_hsi");
        viewer->addPointCloud(cloud_hsi, "cloud_hsi");
        
        if( show_normals == true )
        {
            pcl::PointCloud<NormalT>::Ptr cloud_normals(new pcl::PointCloud<NormalT>());
            pcl::io::loadPCDFile(normal_files[i], *cloud_normals);
            viewer->removePointCloud("cloud_normals");
            viewer->addPointCloudNormals<PointT, NormalT>(cloud, cloud_normals, 3, 0.02, "cloud_normals");
        }
        
        if( pcl::console::find_switch(argc, argv, "-w") == true )
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud_hsi");
        viewer->spin();
    }
    
    return 1;
}
*/