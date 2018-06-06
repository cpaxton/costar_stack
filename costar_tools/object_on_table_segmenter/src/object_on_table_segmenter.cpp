#include <iostream>
#include <termios.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// for using tf surface
#include <pcl_ros/transforms.h>

// for creating directory automatically
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

// for pcl segmentation
// #include "object_on_table_segmenter/plane_segmenter.h"
#include "object_on_table_segmenter/ros_plane_segmenter.h"


std::string input_point_cloud_topic;
std::string save_directory, object_name, original_directory, ground_truth_directory;
int cloud_save_index;
ros::Subscriber pc_sub;
pcl::PCDWriter writer;
double time_step;
bool keypress = false;
int key_value = 0;
bool auto_capture_segmentation;
int num_to_capture = 0;
bool do_cluster;

RosPlaneSegmenter segmenter;
boost::posix_time::ptime time_to_save;
    
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

void saveCloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud_input, std::string dir, std::string additional_text = std::string(""))
{
    std::stringstream ss;
    
    try{
      using namespace boost::posix_time;
      using namespace std;

      ss.imbue(std::locale(ss.getloc(), new boost::posix_time::time_facet("%Y_%m_%d_%H_%M_%S_")));
      ss << dir << time_to_save << object_name << "_" << cloud_save_index << additional_text << ".pcd";
      writer.write<pcl::PointXYZRGBA> (ss.str (), cloud_input, true);
      std::cerr << "Saved: " << ss.str() << "\n";
    } catch (pcl::IOException e) {
      ROS_ERROR("could not write to %s!",ss.str().c_str());
    }
}

void cloud_segmenter_and_save(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_filtered)
{
  std::cerr << "Object Segmentation process begin \n";
  std::vector<CloudXYZRGBA::Ptr> clustered_cloud = segmenter.clusterPointCloud(*cloud_filtered, 500);

  unsigned int initialIndex = cloud_save_index;
  // save detected cluster data
  for (size_t i = 0; i < clustered_cloud.size(); i++)
  {
    std::stringstream ss;
    ss << "_cluster_" << i+1 << "_ground_truth";
    saveCloud(*clustered_cloud[i],ground_truth_directory, ss.str());
  }

  std::cerr << "Segmented object: " << clustered_cloud.size() << ". Segmentation done.\n Waiting for keypress to get new data \n";
}

void callback(const sensor_msgs::PointCloud2 &pc)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
  // convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA
  // update the time which will become the leading string of collected data
  time_to_save = boost::posix_time::second_clock::local_time();
  if (segmenter.ready and keypress)
  {
    pcl::fromROSMsg(pc, *cloud);
    if(key_value == 't'){
      // force update the table plane surface segmentation
      segmenter.enableSavingTableFile();
      segmenter.segmentPlane(pc);
      
    } else {
      // get the object segmented from above the table surface
      cloud_save_index++;
      saveCloud(*cloud,original_directory,"_original");
      cloud = segmenter.segmentAbovePlane(*cloud);
      if (do_cluster) cloud_segmenter_and_save(cloud);
      else saveCloud(*cloud,ground_truth_directory,"_ground_truth");

    }
  }
  else
  {
    segmenter.segmentPlaneIfNotExist(pc);

    if (segmenter.ready)
    {
      std::cerr << "Add new object to the table \n";
      std::cerr << "Press 's' key to do object on table segmentation. \n";
      std::cerr << "Press 't' key to update the table itself and overwrite the table file. \n";
      std::cerr << "Press 'q' key to exit. \n";
    }
    else
    {
      std::cerr << "Failed to perform plane segmentation.\n";
      std::cerr << "Press the 's' key to retry.\n";
      std::cerr << "Press the 'q' key to exit.\n";
    }
  }
}

void callbackCaptureEnvironment(const sensor_msgs::PointCloud2 &pc)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
  // convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA
  pcl::fromROSMsg(pc, *cloud);
  std::stringstream ss;
  ss << save_directory << "environment" << ++cloud_save_index << ".pcd";
  writer.write<pcl::PointXYZRGBA> (ss.str(), *cloud, true);
  std::cerr << "Saved " << ss.str() << std::endl;
}

int main (int argc, char** argv)
{
  ros::init(argc,argv,"object_on_table_segmenter_Node");    
  ros::NodeHandle nh("~");
  
  //getting subscriber parameters
  nh.param("input_point_cloud_topic", input_point_cloud_topic,std::string("/camera/depth_registered/points"));

  //getting save parameters
  nh.param("save_directory",save_directory,std::string("./"));
  nh.param("original_directory",original_directory,std::string("original/"));
  nh.param("ground_truth_directory",ground_truth_directory,std::string("ground_truth/"));
  nh.param("object",object_name,std::string("cloud_cluster_"));
  nh.param("save_index",cloud_save_index,0);
  nh.param("num_to_capture",num_to_capture,200);

  nh.param("time_step",time_step,0.1);
  nh.param("auto_capture_segmentation",auto_capture_segmentation,false);
  
  nh.param("do_cluster",do_cluster,true);

  bool just_capture_environment;
  nh.param("environment_only",just_capture_environment,false);
  boost::filesystem::create_directories(save_directory);
  boost::filesystem::create_directories(original_directory);
  boost::filesystem::create_directories(ground_truth_directory);

  // initialize plane segmenter
  segmenter.initialize(nh);

  if (just_capture_environment)
  { 
    pc_sub = nh.subscribe(input_point_cloud_topic,1,callbackCaptureEnvironment);
    ros::Rate r(1);
    while (ros::ok()){
      r.sleep();
      ros::spinOnce();
    }
  }
  else
  {
    pc_sub = nh.subscribe(input_point_cloud_topic,1,callback);
    if (!segmenter.ready)
      std::cerr << "1) Remove all objects from the table\n2) make sure the AR tag you specified is visible\n3) press the 's' key to save the segmentation plane, or 't' to create and save a new table segmentation." << std::endl;
    else
    {
      std::cerr << "Press 'q' key to exit \n";
      std::cerr << "Press 's' key to start object on table segmentation, or 't' to segment the table itself \n";
    }

    ros::Rate r(10); // 10 hz
    int key = 0;

    if (not auto_capture_segmentation) {
      keypress = true;
      while (ros::ok())
      {
        key = getch();
        if ((key == 's') || (key == 'S')){
          key_value = 's';
          ros::spinOnce();
        }
        else if ((key == 't') || (key == 'T')){
          key_value = 't';
          ros::spinOnce();
        }
        else if ((key == 'q') || (key == 'Q'))
          break;
        r.sleep();
      }
    } else {
      // run loop with time
      int count = 0;
      while (ros::ok()) {

        if (not segmenter.ready) {
          key = getch();
          if ((key == 's') || (key == 'S'))
            ros::spinOnce();
          else if ((key == 'q') || (key == 'Q'))
            break;
        } 
        else if (not keypress) {
          std::cout << "Press any key to start data collection...\n";
          key = getch();
          keypress = true;
        }

        ++count;
        
        if (num_to_capture > 0 && count >= num_to_capture) {
          break;
        }

        ros::spinOnce();
        ros::Duration(time_step).sleep();
      }
    }
  }
  
  ros::shutdown();
  return (0);
}
