#include <iostream>
#include <termios.h>
// For removing point cloud outside certain distance
// based on http://pointclouds.org/documentation/tutorials/passthrough.php tutorial
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

// for pcl segmentation
// based on http://www.pointclouds.org/documentation/tutorials/cluster_extraction.php
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// // displaying image and reading key input
// #include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
// #include <cv_bridge/cv_bridge.h>

std::string POINTS_IN;
std::string save_directory, object_name;
int cloud_save_index;
ros::Subscriber pc_sub;
pcl::PCDWriter writer;

// function getch is from http://answers.ros.org/question/63491/keyboard-key-pressed/
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filter_distance(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input)
{
  std::cerr << "Cloud points before distance filtering: " << cloud_input->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud_input);

  pass.setFilterFieldName ("x");
  pass.setFilterLimits (0.2, 0.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pass.setInputCloud (cloud_filtered);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.5, 1.25);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after distance filtering: " << cloud_filtered->points.size() << std::endl;
  return cloud_filtered;
}

void cloud_segmenter_and_save(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud)
{
  std::cerr << "Segmentation process begin \n";
  // Remove point outside certain distance
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>),cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud_filtered = cloud_filter_distance(input_cloud);
  //writer.write<pcl::PointXYZRGBA> (save_directory+"distance_filtered.pcd", *cloud_filtered, false);

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGBA> ());
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
  ec.setClusterTolerance (0.03); // 5cm
  ec.setMinClusterSize (3000);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cerr << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << save_directory << object_name << cloud_save_index << ".pcd";
    writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_cluster, false); //*
    std::cerr << "Saved " << save_directory << object_name << cloud_save_index << ".pcd" << std::endl;
    cloud_save_index++;
  }

  std::cerr << "Segmentation done \n. Wait for new data \n";
}

void callback(const sensor_msgs::PointCloud2 &pc)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  // convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA
  pcl::fromROSMsg(pc, *cloud);
  cloud_segmenter_and_save(cloud);
}

int main (int argc, char** argv)
{
  ros::init(argc,argv,"object_on_table_segmenter_Node");    
  ros::NodeHandle nh("~");
  //getting subscriber/publisher parameters
  nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
  nh.param("save_directory",save_directory,std::string("./result"));
  nh.param("object",object_name,std::string("cloud_cluster_"));
  cloud_save_index = 0;

  pc_sub = nh.subscribe(POINTS_IN,1,callback);
  std::cerr << "Press 'q' key to exit \n";
  std::cerr << "Press 's' key to do object on table segmentation \n";

  ros::Rate r(10); // 10 hz
  int key = 0;

  while (ros::ok())
  {
    key = getch();
    if ((key == 's') || (key == 'S'))
      ros::spinOnce();
    else if ((key == 'q') || (key == 'Q'))
      break;
    r.sleep();
  }
  ros::shutdown();
  return (0);
}
