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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudLabelTransferToRedChannel(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr input_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
  result->width = input_cloud->width;
  result->height = input_cloud->height;
  for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator it = input_cloud->begin(); it != input_cloud->end(); ++it)
    {
      pcl::PointXYZRGB point;
      point.x = it->x;
      point.y = it->y;
      point.z = it->z;
      // point.r = it->r;
      // point.g = it->g;
      // point.b = it->b;
      point.r = it->label;
      point.g = 0;
      point.b = 0;
      result->push_back(point);
    }
    return result;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv,"sample_pcd_publisher");
    ros::NodeHandle nh ("~");
    ros::Rate r(1); //1Hz
    
    std::string pcd_path, topic_name, header_frame;
    bool labelled_pcd;
    nh.param("pcd_path", pcd_path, std::string("sample.pcd"));
    nh.param("pcl_topic_name", topic_name, std::string("sample_point_cloud"));
    nh.param("pcl_header_frame",header_frame, std::string("/world"));
    nh.param("pcl_labelled",labelled_pcd, false);
    sensor_msgs::PointCloud2 output_msg;

    if (!labelled_pcd)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_data(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PCDReader reader;
      if( reader.read (pcd_path, *pcl_data) == 0)
      {
        std::cerr << "Unlabellled Point cloud loaded successfully.\n";
      }
      else
      {
        std::cerr << "Point cloud fail to load.\n";
      }
      toROSMsg(*pcl_data,output_msg);
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZRGBL>::Ptr pcl_data(new pcl::PointCloud<pcl::PointXYZRGBL>);
      pcl::PCDReader reader;
      if( reader.read (pcd_path, *pcl_data) == 0)
      {
        std::cerr << "Labelled Point cloud loaded successfully.\n";
      }
      else
      {
        std::cerr << "Point cloud fail to load.\n";
      }
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr red_channel_label = PointCloudLabelTransferToRedChannel(pcl_data);
      toROSMsg(*red_channel_label,output_msg);
    }

    ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name,1000);
    output_msg.header.frame_id = header_frame;

    while (ros::ok())
    {
    	pc_pub.publish(output_msg);
    	r.sleep();
    }

	return 0;
}