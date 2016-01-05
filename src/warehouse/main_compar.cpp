#include "../include/utility.h"
#include "../include/features.h"
#include "../include/BBDataParser.h"
#include "../include/UWDataParser.h"
#include "../include/JHUDataParser.h"

#include <pcl/features/our_cvfh.h>
#include <pcl/features/esf.h>


pcl::visualization::PCLVisualizer::Ptr viewer;

#define MAX_POOLER_NUM 50
#define MAX_FEA_NUM 10

cv::Mat extractVFH(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr normals)
{
    if( cloud->empty() == true || normals->empty() == true)
        return cv::Mat::zeros(1, 308, CV_32FC1);
    
    // Create the VFH estimation class, and pass the input dataset+normals to it
    pcl::VFHEstimation<PointT, NormalT, pcl::VFHSignature308> vfh;
    vfh.setInputCloud (cloud);
    vfh.setInputNormals (normals);
    //vfh.setViewPoint(0, 0, -1);
    
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT> ());
    vfh.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

    // Compute the features
    vfh.compute (*vfhs);
    
    cv::Mat vfh_fea = cv::Mat::zeros(1, 308, CV_32FC1);
    cv::Mat vfh_1 = cv::Mat::zeros(1, 200, CV_32FC1);
    cv::Mat vfh_2 = cv::Mat::zeros(1, 108, CV_32FC1);
    
    float *ptr = vfhs->at(0).histogram;
    float *ptr1 = (float *)vfh_1.data;
    float *ptr2 = (float *)vfh_2.data;
    for( int i = 0 ; i < 200 ; i++, ptr++, ptr1++ )
        *ptr1 = *ptr;
    for( int i = 0 ; i < 108 ; i++, ptr++, ptr2++ )
        *ptr2 = *ptr;
    
    //cv::normalize(vfh_1, vfh_1, 1.0, 0.0, cv::NORM_L2);
    //cv::normalize(vfh_2, vfh_2, 1.0, 0.0, cv::NORM_L2);
    cv::hconcat(vfh_1, vfh_2, vfh_fea);
    //cv::normalize(vfh_fea, vfh_fea, 1.0, 0.0, cv::NORM_L2);
    return vfh_fea;
}

cv::Mat extractOURVFH(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr normals)
{
    if( cloud->empty() == true || normals->empty() == true)
        return cv::Mat::zeros(1, 308, CV_32FC1);
    
    pcl::PointCloud<myPointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*cloud, *cloud_xyz);
    
    pcl::search::KdTree<myPointXYZ>::Ptr kdtree(new pcl::search::KdTree<myPointXYZ> ());
    
    pcl::OURCVFHEstimation<myPointXYZ, NormalT, pcl::VFHSignature308> ourcvfh;
    ourcvfh.setInputCloud(cloud_xyz);
    ourcvfh.setInputNormals(normals);
    ourcvfh.setSearchMethod(kdtree);
    ourcvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
    ourcvfh.setCurvatureThreshold(1.0);
    ourcvfh.setNormalizeBins(true);
    
    // Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
    // this will decide if additional Reference Frames need to be created, if ambiguous.
    ourcvfh.setAxisRatio(0.8);

    // Compute the features
    pcl::PointCloud<pcl::VFHSignature308>::Ptr feature(new pcl::PointCloud<pcl::VFHSignature308>);
    ourcvfh.compute(*feature);
    
    cv::Mat vfh_fea = cv::Mat::zeros(1, 308, CV_32FC1);
    
    float *ptr = feature->at(0).histogram;
    float *ptr1 = (float *)vfh_fea.data;
    
    for( int i = 0 ; i < 308 ; i++, ptr++, ptr1++ )
        *ptr1 = *ptr;
    //std::cerr << vfh_fea << std::endl;
    //std::cin.get();
    //cv::normalize(vfh_fea, vfh_fea, 1.0, 0.0, cv::NORM_L2);
    return vfh_fea;
}

cv::Mat extractESF(const pcl::PointCloud<PointT>::Ptr cloud, const pcl::PointCloud<NormalT>::Ptr normals)
{
    pcl::PointCloud<pcl::ESFSignature640>::Ptr descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
    
    // ESF estimation object.
    pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud);
    
    // Compute the features
    esf.compute(*descriptor);
    
    cv::Mat esf_fea = cv::Mat::zeros(1, 640, CV_32FC1);
    float *ptr = descriptor->at(0).histogram;
    float *ptr1 = (float *)esf_fea.data;
    
    for( int i = 0 ; i < 640 ; i++, ptr++, ptr1++ )
        *ptr1 = *ptr;
    
    //cv::normalize(vfh_fea, vfh_fea, 1.0, 0.0, cv::NORM_L2);
    return esf_fea;
}

//*
int main(int argc, char** argv)
{
    std::string in_path("/home/chi/BigBIRD/processed");
    std::string out_path("BB_vfh");
    
    int c1 = 0, c2 = BB_INST_MAX-1;
    int dataset_id = -1;
    pcl::console::parse_argument(argc, argv, "--p", in_path);
    pcl::console::parse_argument(argc, argv, "--o", out_path);
    
    pcl::console::parse_argument(argc, argv, "--c1", c1);
    pcl::console::parse_argument(argc, argv, "--c2", c2);
    
    float radius = 0.03;
    pcl::console::parse_argument(argc, argv, "--r", radius);
    
    if( pcl::console::find_switch(argc, argv, "-uw") == true )
        dataset_id = 0;
    
    if( pcl::console::find_switch(argc, argv, "-bb") == true )
        dataset_id = 1;
    
    if( pcl::console::find_switch(argc, argv, "-jhu") == true )
        dataset_id = 2;
    
    int fea_len = 33;// + fea_len_L1;// + fea_len_L2;
    std::cerr << "Total Feature Length: "<< fea_len << std::endl;
    
    if( exists_dir(out_path) == false )
        boost::filesystem::create_directories(out_path);
    for( int i = c1 ; i <= c2  ; i++ )
    {
        double t1, t2;
        t1 = get_wall_time();
        std::stringstream ss;
        ss << i;
        
        ObjectSet train_objects, test_objects;
        
        switch(dataset_id)
        {
            case 0:
                readUWInst(in_path, train_objects, test_objects, i, i);
                break;
            case 1:
                readBB(in_path, train_objects, test_objects, i, i);
                break;
            case 2:
                readJHUInst(in_path, train_objects, test_objects, i, i);
                break;
            default:
                std::cerr << "No Corresponding Dataset!" << std::endl;
                exit(0);
        }
        std::cerr << "Loading Completed... " << std::endl;
        
        int train_num = train_objects[0].size();
        if( train_num == 0 )
        {
            std::cerr << "Empty Set!" << std::endl;
            exit(0);
        }

        std::vector< sparseVec> final_train_fea(train_num);
        int final_train_fea_dim = -1;
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < train_num ; j++ )
        {
            //cv::Mat final_temp = extractVFH(train_objects[0][j].cloud, train_objects[0][j].cloud_normals);
            cv::Mat final_temp = extractOURVFH(train_objects[0][j].cloud, train_objects[0][j].cloud_normals);
            //cv::Mat final_temp = extractESF(train_objects[0][j].cloud, train_objects[0][j].cloud_normals);
            
            final_train_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_train_fea[j] = final_sparse[0];
            
        }
        saveCvMatSparse(out_path + "/train_"+ss.str()+"_L0.smat", final_train_fea, final_train_fea_dim);
        final_train_fea.clear();
        train_objects.clear();
        
        int test_num = test_objects[0].size();
        std::vector< sparseVec> final_test_fea(test_num);
        int final_test_fea_dim = -1;
        #pragma omp parallel for schedule(dynamic, 1)
        for( int j = 0 ; j < test_num ; j++ )
        {
            //cv::Mat final_temp = extractVFH(test_objects[0][j].cloud, test_objects[0][j].cloud_normals);
            cv::Mat final_temp = extractOURVFH(test_objects[0][j].cloud, test_objects[0][j].cloud_normals);
            //cv::Mat final_temp = extractESF(test_objects[0][j].cloud, test_objects[0][j].cloud_normals);
            
            final_test_fea_dim = final_temp.cols;
            std::vector< sparseVec> final_sparse;
            sparseCvMat(final_temp, final_sparse);
            final_test_fea[j] = final_sparse[0];
        }
        saveCvMatSparse(out_path + "/test_"+ss.str()+"_L0.smat", final_test_fea, final_test_fea_dim);
        final_test_fea.clear();
        test_objects.clear();        
        
        t2 = get_wall_time();
        std::cerr << t2 - t1 << std::endl;
    }
    
    return 1;
}
//*/
