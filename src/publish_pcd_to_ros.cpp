#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <Eigen/Dense>
// #include <Eigen/Geometry>

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr loadPointCloudFromFile(const std::string &pcd_path)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PCDReader reader;
  if( reader.read (pcd_path, *point_cloud) == 0)
  {
    std::cerr << "Point cloud loaded successfully.\n";
  }
  else
  {
    std::cerr << "Point cloud fail to load.\n";
  }
  return point_cloud;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv,"sample_pcd_publisher");
    ros::NodeHandle nh ("~");
    ros::Rate r(1); //1Hz
    
    std::string pcd_path, topic_name, header_frame;
    nh.param("pcd_path", pcd_path, std::string("sample.pcd"));
    nh.param("pcl_topic_name", topic_name, std::string("sample_point_cloud"));
    nh.param("pcl_header_frame",header_frame, std::string("/world"));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl_data = loadPointCloudFromFile(pcd_path);

    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name,1000);
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*pcl_data,output_msg);

    output_msg.header.frame_id = header_frame;

    while (ros::ok())
    {
    	pc_pub.publish(output_msg);
    	r.sleep();
    }

	return 0;
}