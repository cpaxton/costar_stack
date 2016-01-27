#ifndef SP_SEGMENTER_REFINE_POSES
#define SP_SEGMENTER_REFINE_POSES
std::vector<poseT> RefinePoses(const pcl::PointCloud<myPointXYZ>::Ptr scene, const std::vector<ModelT> &mesh_set, const std::vector<poseT> &all_poses)
{
    int pose_num = all_poses.size();
    std::vector<ModelT> est_models(pose_num);
    pcl::PointCloud<myPointXYZ>::Ptr down_scene(new pcl::PointCloud<myPointXYZ>());
    pcl::VoxelGrid<myPointXYZ> sor;
    sor.setInputCloud(scene);
    sor.setLeafSize(0.005, 0.005, 0.005);
    sor.filter(*down_scene);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for(int i = 0 ; i < pose_num ; i++ ){
        for( int j = 0 ; j < mesh_set.size() ; j++ ){
            if( mesh_set[j].model_label == all_poses[i].model_name )
            {
                est_models[i].model_label = all_poses[i].model_name;
                est_models[i].model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*mesh_set[j].model_cloud, *est_models[i].model_cloud, all_poses[i].shift, all_poses[i].rotation);
                break;
            }
        } 
    }
    
    std::vector< pcl::search::KdTree<myPointXYZ>::Ptr > tree_set(est_models.size());
    #pragma omp parallel for schedule(dynamic, 1)
    for( int i = 0 ; i < pose_num ; i++ )
    {
        tree_set[i] = pcl::search::KdTree<myPointXYZ>::Ptr (new pcl::search::KdTree<myPointXYZ>());
        tree_set[i]->setInputCloud(est_models[i].model_cloud);
    }   
    
    std::vector<int> votes(pose_num, 0);
    std::vector< std::vector<int> > adj_graph(pose_num);
    for( int i = 0 ; i < pose_num ; i++ )
        adj_graph[i].resize(pose_num, 0);
    float sqrT = 0.01*0.01;
    int down_num = down_scene->size();
    
    std::vector< std::vector<int> > bin_vec(down_num);
    #pragma omp parallel for
    for(int i = 0 ; i < pose_num ; i++ )
    {
        int count = 0;
        for( pcl::PointCloud<myPointXYZ>::const_iterator it = down_scene->begin() ; it < down_scene->end() ; it++, count++ )
        {
            std::vector<int> idx (1);
            std::vector<float> sqrDist (1);
            int nres = tree_set[i]->nearestKSearch(*it, 1, idx, sqrDist);
            if ( nres >= 1 && sqrDist[0] <= sqrT )
            {
                #pragma omp critical
                {   
                    bin_vec[count].push_back(i);
                }
                votes[i]++;
            }
        }
    }
    
    for( int it = 0 ; it < down_num ; it++ )
        for( std::vector<int>::iterator ii = bin_vec[it].begin() ; ii < bin_vec[it].end() ; ii++ )
            for( std::vector<int>::iterator jj = ii+1 ; jj < bin_vec[it].end() ; jj++ )
            {
                adj_graph[*ii][*jj]++;
                adj_graph[*jj][*ii]++;
            }
    std::vector<bool> dead_flag(pose_num, 0);
    for( int i = 0 ; i < pose_num ; i++ ){
        if( dead_flag[i] == true )
            continue;
        for( int j = i+1 ; j < pose_num ; j++ )
        {
            if( dead_flag[j] == true )
                continue;
            int min_tmp = std::min(votes[i], votes[j]);
            if( (adj_graph[i][j]+0.0) / min_tmp >= 0.3 )
            {
                std::cerr << votes[i] << " " << i << std::endl;
                std::cerr << votes[j] << " " << j << std::endl;
                if( votes[i] > votes[j] )
                    dead_flag[j] = true;
                else
                {
                    dead_flag[i] = true;
                    break;
                }
            }
        }
    }
    std::vector<poseT> refined_poses;
    for( int i = 0 ; i < pose_num ; i++ )   
        if( dead_flag[i] == false )
            refined_poses.push_back(all_poses[i]);
    
    return refined_poses;
}
#endif