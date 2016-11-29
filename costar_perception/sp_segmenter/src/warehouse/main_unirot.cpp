#include "sp_segmenter/utility/utility.h"

std::string path("/media/DATA1/BigBIRD/processed");
std::string model("3m_high_tack_spray_adhesive");

std::string num_rot("1392"); //228 1392...
#define SAMPLE_RATE 0.005

pcl::visualization::PCLVisualizer::Ptr viewer;

//#define DEBUG_VIEW

float distCloud(const pcl::PointCloud<PointT>::Ptr cloud1, const pcl::PointCloud<PointT>::Ptr cloud2)
{
    pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>());
    kdtree->setInputCloud(cloud2);
    float avg_error = 0;
    pcl::PointCloud<PointT>::iterator it1;
    for( it1 = cloud1->begin() ; it1 < cloud1->end() ; it1++ )
    {
        std::vector<int> idx;
        std::vector<float> dist;
        kdtree->nearestKSearch(*it1, 1, idx, dist);
        
        avg_error += sqrt(dist[0]);
    }
    return avg_error / cloud1->size();
}

float distCloud_corr(const pcl::PointCloud<PointT>::Ptr cloud1, const pcl::PointCloud<PointT>::Ptr cloud2)
{
    float avg_error = 0;
    pcl::PointCloud<PointT>::iterator it1, it2;
    for( it1 = cloud1->begin(), it2 = cloud2->begin() ; it1 < cloud1->end() ; it1++, it2++ )
    {
        float diffx = (*it1).x - (*it2).x;
        float diffy = (*it1).y - (*it2).y;
        float diffz = (*it1).z - (*it2).z;
        
        avg_error += sqrt(diffx*diffx+diffy*diffy+diffz*diffz);
    }
    return avg_error / cloud1->size();
}

//both clouds and partial clouds all centered at origin
std::vector< std::vector<int> > poseClustering(const CloudSet &partial_set, const CloudSet &partial_ground, float T = 5e-6)
{
    if( partial_ground.size() != partial_set.size() )
    {
        std::cerr << "partial grounds != partial views!" << std::endl;
        exit(1);
    }
    int num = partial_set.size();
    
    //viewer->removeAllPointClouds();
    
    Eigen::MatrixXi flag = Eigen::MatrixXi::Identity(num, num);
    #pragma omp parallel for schedule(dynamic, 1) shared(flag)
    for( int i = 0 ; i < num ; i++ ){ 
        pcl::PointCloud<PointT>::Ptr main_temp(new pcl::PointCloud<PointT>());        
        pcl::IterativeClosestPoint<PointT, PointT> main_icp;
        main_icp.setMaxCorrespondenceDistance(0.05);
        main_icp.setTransformationEpsilon (T);
        main_icp.setMaximumIterations(50);
        main_icp.setInputTarget(partial_ground[i]);
        
        pcl::PointCloud<PointT>::Ptr aux_temp(new pcl::PointCloud<PointT>());        
        pcl::IterativeClosestPoint<PointT, PointT> aux_icp;
        aux_icp.setMaxCorrespondenceDistance(0.05);
        aux_icp.setTransformationEpsilon (T);
        aux_icp.setMaximumIterations(50);
        aux_icp.setInputSource(partial_set[i]);
       
#ifdef DEBUG_VIEW        
        viewer->removePointCloud("source");
        viewer->addPointCloud(partial_ground[i], "source");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "source");
#endif
        for(int j = i+1 ; j < num ; j++ ){
            main_icp.setInputSource(partial_set[j]);
            main_icp.align(*main_temp);
            
            float icp_score = main_icp.getFitnessScore();
#ifdef DEBUG_VIEW
            std::cerr << icp_score << std::endl;
            Eigen::Matrix4f tran0 = icp.getFinalTransformation();
            Eigen::Matrix4f final_tran = tran0.inverse();
            
            temp->clear();
            pcl::transformPointCloud(*partial_ground[i], *temp, final_tran);

            viewer->removePointCloud("target");
            viewer->addPointCloud(partial_ground[j], "target");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "target");
            viewer->removePointCloud("aligned");
            viewer->addPointCloud(temp, "aligned");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "aligned");
            viewer->spin();
#endif            
            if( icp_score < T )
            {
                aux_icp.setInputTarget(partial_ground[j]);
                aux_icp.align(*aux_temp);
                float aux_icp_score = aux_icp.getFitnessScore();
                if( aux_icp_score < T)
                {
                    flag(i,j) = 1;
                    flag(j,i) = 1;
                }
            }
        }
    }
    
    //save undirected graph
    saveUndirectedGraph(flag, path+"/"+model+"/pose_graph.txt");
    
    //maximal clique finding
    std::vector< std::vector<int> > cluster_idx = maximalClique(flag);
    
    return cluster_idx;
}

///*
// run one model test
int main(int argc, char** argv)
{   
    pcl::console::parse_argument(argc, argv, "--p", path);
    pcl::console::parse_argument(argc, argv, "--m", model);
    //pcl::console::parse_argument(argc, argv, "--t", thread_num);    

    std::cerr << "Parsing Model: " << model << std::endl;
    std::string ply_file(path+"/"+model+"/textured_meshes/optimized_tsdf_textured_mesh.ply");
    std::string rot_file("uniRot_txt/SemiRot_"+num_rot+".txt");
    
    poseVec rot_set = readRots(rot_file);
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(ply_file, triangles);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);
    
    //viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("viewer"));    
    //viewer->initCameraParameters();
    //viewer->addCoordinateSystem(0.1);
    
    CloudSet partial_set, partial_ground_set;
    poseVec partial_trans;
    genViews(cloud, rot_set, partial_set, partial_trans, viewer, 0.005, 0.01);
    
    int num = partial_set.size();
    partial_ground_set.resize(num);
    
    pcl::PointCloud<PointT>::Ptr down_cloud(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.005, 0.005, 0.005);
    sor.filter(*down_cloud);
    #pragma omp parallel for schedule(dynamic,1)
    for( int i = 0 ; i < num ; i++ )
    {
        pcl::PointCloud<PointT>::Ptr tran_cloud(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*down_cloud, *tran_cloud, partial_trans[i]);
        partial_ground_set[i] = tran_cloud;
    }
    
    clock_t t1, t2;
    t1 = clock();
    std::vector< std::vector<int> > cluster_idx = poseClustering(partial_set, partial_ground_set, 5e-6);
    t2 = clock();
    std::cerr<<"Clustering time: "<<((double)t2-(double)t1) / CLOCKS_PER_SEC << std::endl;
    
    //saving cluster idx
    std::ofstream fp((path+"/"+model+"/pose_clusters.txt").c_str(), std::ios::out);
    if( fp.is_open() == true )
    {
        fp << cluster_idx.size() << std::endl;
        for( int i = 0 ; i < cluster_idx.size() ; i++ )
        {
            fp << cluster_idx[i].size();
            for( int j = 0 ; j < cluster_idx[i].size() ; j++ )
                fp << " " << cluster_idx[i][j];
            fp << std::endl;
        }
        fp.close();
    }
    return 1;
}

