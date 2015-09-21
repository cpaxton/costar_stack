#include "../include/utility.h"
#include <flann/io/hdf5.h>

std::string path("/media/DATA1/BigBIRD/processed");
std::string model("3m_high_tack_spray_adhesive");

///*
// run one model test
int main(int argc, char** argv)
{
    if ( argc == 2 )
        model = argv[1];
    
    bool view_flag = false;
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    
    pcl::visualization::PCLVisualizer::Ptr viewer;
    if( view_flag == true )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
    }
    std::string cloud_path(path+"/"+model+"/clouds/");
    std::string pose_path(path+"/"+model+"/poses/");
    std::string all_pose_path(path+"/"+model+"/all_poses/");
    
    if( exists_dir(all_pose_path) == false )
    {
        if (boost::filesystem::create_directory(all_pose_path))
            std::cerr << "Creating Path Successfully!" << std::endl;
        else
            std::cerr << "Creating Path Failed!" << std::endl;
    }
    
    flann::Matrix<double> temp1, temp2, temp3, temp4;
    flann::load_from_file(temp1, path+"/"+model+"/calibration.h5", "H_NP1_from_NP5");
    flann::load_from_file(temp2, path+"/"+model+"/calibration.h5", "H_NP2_from_NP5");
    flann::load_from_file(temp3, path+"/"+model+"/calibration.h5", "H_NP3_from_NP5");
    flann::load_from_file(temp4, path+"/"+model+"/calibration.h5", "H_NP4_from_NP5");
    Eigen::Matrix4f tran_HP5_HP1, tran_HP5_HP2, tran_HP5_HP3, tran_HP5_HP4;
    Eigen::Matrix4f tran_HP1_HP5, tran_HP2_HP5, tran_HP3_HP5, tran_HP4_HP5;
    for( int i = 0 ; i < 4 ; i++ )
        for(int j = 0 ; j < 4 ; j++ )
        {
            tran_HP5_HP1(i, j) = temp1[i][j];
            tran_HP5_HP2(i, j) = temp2[i][j];
            tran_HP5_HP3(i, j) = temp3[i][j];
            tran_HP5_HP4(i, j) = temp4[i][j];
        }
    tran_HP1_HP5 = tran_HP5_HP1.inverse();
    tran_HP2_HP5 = tran_HP5_HP2.inverse();
    tran_HP3_HP5 = tran_HP5_HP3.inverse();
    tran_HP4_HP5 = tran_HP5_HP4.inverse();
    
    pcl::PointCloud<PointT>::Ptr model(new pcl::PointCloud<PointT> ());
    for( int i = 0 ; i < 360 ; i+=3 )
    {
        std::ostringstream ss;
        ss << i;
        
        pcl::PointCloud<PointT>::Ptr cloud5(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr cloud4(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr cloud3(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr cloud2(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr cloud1(new pcl::PointCloud<PointT> ());
        pcl::io::loadPCDFile(cloud_path+"NP5_"+ss.str()+".pcd", *cloud5);
        pcl::io::loadPCDFile(cloud_path+"NP4_"+ss.str()+".pcd", *cloud4);
        pcl::io::loadPCDFile(cloud_path+"NP3_"+ss.str()+".pcd", *cloud3);
        pcl::io::loadPCDFile(cloud_path+"NP2_"+ss.str()+".pcd", *cloud2);
        pcl::io::loadPCDFile(cloud_path+"NP1_"+ss.str()+".pcd", *cloud1);
         
        std::cerr << "Instance: " << i << std::endl;
        
        flann::Matrix<double> pose5;
        flann::load_from_file(pose5, pose_path+"NP5_"+ss.str()+"_pose.h5", "H_table_from_reference_camera");
        Eigen::Matrix4f tran_HP5_table;
        for( int i = 0 ; i < 4 ; i++ )
            for(int j = 0 ; j < 4 ; j++ )
                tran_HP5_table(i, j) = pose5[i][j];
        
        pcl::PointCloud<PointT>::Ptr tran_cloud5(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr tran_cloud4(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr tran_cloud3(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr tran_cloud2(new pcl::PointCloud<PointT> ());
        pcl::PointCloud<PointT>::Ptr tran_cloud1(new pcl::PointCloud<PointT> ());
        
        pcl::transformPointCloud(*cloud5, *tran_cloud5, tran_HP5_table);
        writePoseTxT(all_pose_path + "/NP5_"+ss.str()+"_pose.txt", tran_HP5_table);
        pcl::transformPointCloud(*cloud4, *tran_cloud4, tran_HP5_table*tran_HP4_HP5);
        writePoseTxT(all_pose_path + "/NP4_"+ss.str()+"_pose.txt", tran_HP5_table*tran_HP4_HP5);
        pcl::transformPointCloud(*cloud3, *tran_cloud3, tran_HP5_table*tran_HP3_HP5);
        writePoseTxT(all_pose_path + "/NP3_"+ss.str()+"_pose.txt", tran_HP5_table*tran_HP3_HP5);
        pcl::transformPointCloud(*cloud2, *tran_cloud2, tran_HP5_table*tran_HP2_HP5);
        writePoseTxT(all_pose_path + "/NP2_"+ss.str()+"_pose.txt", tran_HP5_table*tran_HP2_HP5);
        pcl::transformPointCloud(*cloud1, *tran_cloud1, tran_HP5_table*tran_HP1_HP5);
        writePoseTxT(all_pose_path + "/NP1_"+ss.str()+"_pose.txt", tran_HP5_table*tran_HP1_HP5);
        
        model->insert(model->end(), tran_cloud5->begin(), tran_cloud5->end());
        model->insert(model->end(), tran_cloud4->begin(), tran_cloud4->end());
        model->insert(model->end(), tran_cloud3->begin(), tran_cloud3->end());
        model->insert(model->end(), tran_cloud2->begin(), tran_cloud2->end());
        model->insert(model->end(), tran_cloud1->begin(), tran_cloud1->end());
        //viewer->spin();
    }
    if( view_flag == true )
    {
        viewer->addPointCloud(model, "model");
        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0 , 0.0, 0.0, "model");
        viewer->spin();
    }
    return 1;
}

