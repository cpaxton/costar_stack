#include "sp_segmenter/utility/utility.h"
#include <boost/graph/undirected_graph.hpp>
#include <boost/graph/bron_kerbosch_all_cliques.hpp>

std::string path("/home/chi/BigBIRD/processed");
std::string model("3m_high_tack_spray_adhesive");

pcl::visualization::PCLVisualizer::Ptr viewer;
///*
// run one model test
std::string num_rot("1392"); //420 2700 5280 8580 15360

int main(int argc, char** argv)
{
    pcl::console::parse_argument(argc, argv, "--m", model);
    
    std::cerr << "Parsing: " << model << std::endl; 
    if( pcl::console::find_switch(argc, argv, "-v") == true )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ());
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
    }
    
    float s1 = 0.001, s2 = 0.003;
    pcl::console::parse_argument(argc, argv, "--s1", s1);
    pcl::console::parse_argument(argc, argv, "--s2", s2);
    std::string ply_file(path+"/"+model+"/textured_meshes/optimized_tsdf_textured_mesh.ply");
    
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    
    pcl::PolygonMesh triangles;
    pcl::io::loadPolygonFilePLY(ply_file, triangles);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);  
    poseVec rot_set = readRots("uniRot_txt/SemiRot_"+num_rot+".txt");
    
    CloudSet partial_set;
    poseVec partial_ground_tran;
    genViews(cloud, rot_set, partial_set, partial_ground_tran, viewer, s1, s2);
    
    //saving partial clouds
    std::string partial_path(path+"/"+model+"/partial");
    if( exists_dir(partial_path) == false )
        boost::filesystem::create_directory(partial_path);
    
    std::cerr << "Saving Partial Views..." << std::endl;
    for( int i = 0 ; i < partial_set.size() ; i++ )
    {
        std::ostringstream ss;
        ss << i;
        
        //pcl::io::savePCDFile(partial_path+"/partial_" + ss.str() + ".pcd", *partial_set[i], true);
        //writePoseTxT(partial_path+"/partial_tran_" + ss.str() + ".eigmat", partial_ground_tran[i]);
        
    }
    return 1;
}
//*/


/*
int main(int argc, char *argv[])
{
    Eigen::MatrixXi adj_mat = readUndirectedGraph("pose_graph.txt");
    saveUndirectedGraph(adj_mat, path+"/"+model+"/pose_graph.txt");
    
    std::cerr<<"Finding Maximal Cliques..."<<std::endl;
    std::vector< std::vector<int> > cluster_idx = maximalClique(adj_mat);
    
    int count = 0;
    for( int i = 0 ; i < cluster_idx.size() ; i++ ){
        for( int j = 0 ; j < cluster_idx[i].size() ; j++ )
            std::cerr<< cluster_idx[i][j]<<" ";
        std::cerr<<std::endl;
        count += cluster_idx[i].size();
    }
    
    std::cerr << "Clique Number: "<<cluster_idx.size()<<std::endl;
    std::cerr << count << std::endl;
    
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
    return 0;
}
*/

/*
struct clique_saver
{
    clique_saver(std::vector< std::vector<int> > *idx_ptr) 
                    : cluster_idx(idx_ptr){}

    template <typename Clique, typename Graph>
    void clique(const Clique& c, const Graph& g)
    {
        // Iterate over the clique and print each vertex within it.
        std::vector<int> temp_idx;
        for(typename Clique::const_iterator it = c.begin(); it < c.end(); it++)
            temp_idx.push_back(g[*it]);
        
        cluster_idx->push_back(temp_idx);
    }
    std::vector< std::vector<int> > *cluster_idx;
};
// Declare the graph type and its vertex and edge types.
typedef boost::undirected_graph<int> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;

std::vector< std::vector<int> > maximalClique(const Eigen::MatrixXi &adj_mat, std::string save_file_name = "")
{
    Graph g;
    int num = adj_mat.rows();
    std::vector<Vertex> vv(num);
    for(int i = 0 ; i < num ; i++ )
        vv[i] = g.add_vertex(i);
    
    int edge_num = 0;
    for( int i = 0 ; i < num ; i++ ){
        for( int j = i+1 ; j < num ; j++ ){
            if( adj_mat(i, j) == 1 )
            {
                g.add_edge(vv[i],vv[j]);
                edge_num++;
            }
        }
    }
    std::cerr<<"Edge Number: "<<edge_num<<std::endl;
    
    std::vector< std::vector<int> > cluster_idx;
    clique_saver vis(&cluster_idx);
    boost::bron_kerbosch_all_cliques(g, vis);
    
    // select independent cliques for optimization
    std::vector< std::pair<int, int> > cluster_size(cluster_idx.size());
    for( int i = 0 ; i < cluster_idx.size() ; i++ )
    {
        cluster_size[i].first = i;
        cluster_size[i].second = cluster_idx[i].size();
    }
    std::sort(cluster_size.begin(), cluster_size.end(), comp1);
    
    std::vector<bool> flag(num);
    flag.assign(num, false);
    std::vector< std::vector<int> > final_clusters;
    // from small clique to large
    for( int i = cluster_size.size() - 1; i >= 0 ; i-- )
    {
        int idx = cluster_size[i].first;
        std::vector<int> new_cluster;
        for( std::vector<int>::iterator it = cluster_idx[idx].begin() ; it < cluster_idx[idx].end() ; it++ )
        {
            if( flag[*it] == false )
            {
                new_cluster.push_back(*it);
                flag[*it] = true;
            }
        }
        final_clusters.push_back(new_cluster);
    }
    return final_clusters;
}*/