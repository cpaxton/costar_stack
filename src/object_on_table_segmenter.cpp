#include <iostream>
#include <termios.h>
// For removing point cloud outside certain distance
// based on http://pointclouds.org/documentation/tutorials/passthrough.php tutorial
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

// for pcl segmentation
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// for segment object above table
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

bool dist_viewer, distance_limit_enable[3],haveTable,update_table;
double limitX[2],limitY[2],limitZ[2];
std::string POINTS_IN;
std::string save_directory, object_name, load_directory;
int cloud_save_index;
ros::Subscriber pc_sub;
pcl::PCDWriter writer;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tableHull(new pcl::PointCloud<pcl::PointXYZRGBA>);

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
  //std::cerr << "Cloud points before distance filtering: " << cloud_input->points.size() << std::endl;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  *cloud_filtered = *cloud_input;

  pass.setInputCloud (cloud_filtered);
  pass.setKeepOrganized(true);
  
  if (distance_limit_enable[0])
  {
	pass.setFilterFieldName ("x");
	pass.setFilterLimits (limitX[0], limitX[1]);
	pass.filter (*cloud_filtered);
  }
  if (distance_limit_enable[1])
  {
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (limitY[0], limitY[1]);
	pass.filter (*cloud_filtered);
  }
  if (distance_limit_enable[2])
  {
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (limitZ[0], limitZ[1]);
	pass.filter (*cloud_filtered);
  }
  //std::cerr << "Cloud after distance filtering: " << cloud_filtered->points.size() << std::endl;
  return cloud_filtered;
}

void segmentCloudAboveTable(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convexHull)
{

  std::cerr << "\nSegment object above table \n";
  // Prism object.
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
  prism.setInputCloud(cloud_input);
  prism.setInputPlanarHull(convexHull);

  // from 1 cm above table to 50 cm above table
  prism.setHeightLimits(0.01f, 0.5f);
  pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

  prism.segment(*objectIndices);
  // Get and show all points retrieved by the hull.
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(cloud_input);
  extract.setIndices(objectIndices);
  extract.setKeepOrganized(true);
  extract.filter(*objects);
  *cloud_input = *objects;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getTableConvexHull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr negative)
{

  // Get Normal Cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(negative);
  ne.compute(*normals);

  // Segment planes
  pcl::OrganizedMultiPlaneSegmentation< pcl::PointXYZRGBA, pcl::Normal, pcl::Label > mps;

  mps.setMinInliers (15000);
  mps.setAngularThreshold (0.017453 * 2.0); // 2 degrees
  mps.setDistanceThreshold (0.02); // 2cm
  mps.setInputNormals (normals);
  mps.setInputCloud (negative);
  std::vector< pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
  mps.segmentAndRefine (regions);
  
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZRGBA>);
  boundary->points = regions[0].getContour();

  // Retrieve the convex hull.
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::ConvexHull<pcl::PointXYZRGBA> hull;
  hull.setInputCloud(boundary);
  // Make sure that the resulting hull is bidimensional.
  hull.setDimension(2);
  hull.reconstruct(*convexHull);
  return convexHull;
}

void cloud_segmenter_and_save(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr input_cloud)
{
  std::cerr << "Object Segmentation process begin \n";
  // Remove point outside certain distance
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
  cloud_filtered = cloud_filter_distance(input_cloud);
  if (dist_viewer)
  {
  	std::cerr << "See distance filtered cloud. Press q to quit viewer. \n";
  	pcl::visualization::CloudViewer viewer ("Distance Filtered Cloud Viewer");
  	viewer.showCloud (cloud_filtered);
  	while (!viewer.wasStopped ())
    	{
    	}
  	dist_viewer = false;
  	writer.write<pcl::PointXYZRGBA> (save_directory+"distance_filtered.pcd", *cloud_filtered, true);
  }

  // Get Normal Cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.03f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud_filtered);
  ne.compute(*normals);

  // Segment planes
  pcl::OrganizedMultiPlaneSegmentation< pcl::PointXYZRGBA, pcl::Normal, pcl::Label > mps;
  std::vector<pcl::ModelCoefficients> model_coefficients;
  std::vector<pcl::PointIndices> inlier_indices;  
  pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
  std::vector<pcl::PointIndices> label_indices;
  std::vector<pcl::PointIndices> boundary_indices;

  mps.setMinInliers (10000);
  mps.setAngularThreshold (0.017453 * 3.0); // 2 degrees
  mps.setDistanceThreshold (0.03); // 2cm
  mps.setInputNormals (normals);
  mps.setInputCloud (cloud_filtered);
  std::vector< pcl::PlanarRegion<pcl::PointXYZRGBA>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGBA> > > regions;
  mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

  // Remove detected planes (table) and segment the object
  // int planar_region_number = 0;
  std::vector<bool> plane_labels;
  plane_labels.resize (label_indices.size (), false);
  // for (size_t i = 0; i < label_indices.size (); i++)
  // {
  //   if (label_indices[i].indices.size () > 10000)
  //   {
  //     plane_labels[i] = true;
  //     planar_region_number ++;
  //   }
  // }
  // std::cerr << "Detected planar region: " << planar_region_number << std::endl;

  pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_(new pcl::EuclideanClusterComparator<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> ());
  euclidean_cluster_comparator_->setInputCloud (cloud_filtered);
  euclidean_cluster_comparator_->setLabels (labels);
  euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
  euclidean_cluster_comparator_->setDistanceThreshold (0.03f, false);

  pcl::PointCloud<pcl::Label> euclidean_labels;
  std::vector<pcl::PointIndices> euclidean_label_indices;
  pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGBA,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
  euclidean_segmentation.setInputCloud (cloud_filtered);
  euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

  // save detected cluster data
  for (size_t i = 0; i < euclidean_label_indices.size (); i++)
  {
  	if (euclidean_label_indices[i].indices.size () > 1000)
    {
	    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
	    pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

  		pcl::PointIndices::Ptr object_cloud_indices (new pcl::PointIndices);
  		*object_cloud_indices = euclidean_label_indices[i];
  		extract.setInputCloud(cloud_filtered);
  		extract.setIndices(object_cloud_indices);
  		extract.setKeepOrganized(true);
  		extract.filter(*cloud_cluster);

	    std::stringstream ss;
	    ss << save_directory << object_name << cloud_save_index << ".pcd";
	    writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_cluster, true);
	    std::cerr << "Saved " << save_directory << object_name << cloud_save_index << ".pcd" << std::endl;
	    cloud_save_index++;
  	}
  }

  std::cerr << "Segmentation done.\n Waiting keypress to get new data \n";
}

void callback(const sensor_msgs::PointCloud2 &pc)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  // convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA
  pcl::fromROSMsg(pc, *cloud);
  if (haveTable)
  {
    segmentCloudAboveTable(cloud,tableHull);
    cloud_segmenter_and_save(cloud);
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    cloud_filtered = cloud_filter_distance(cloud);
    if (dist_viewer)
    {
      std::cerr << "See distance filtered cloud. Press q to quit viewer. \n";
      pcl::visualization::CloudViewer viewer ("Distance Filtered Cloud Viewer");
      viewer.showCloud (cloud_filtered);
      while (!viewer.wasStopped ())
        {
        }
    }
    tableHull = getTableConvexHull(cloud_filtered);
    if (update_table)
      writer.write<pcl::PointXYZRGBA> (load_directory+"/table.pcd", *tableHull, true);
    haveTable = true;

    std::cerr << "Add new object to the table \n";
    std::cerr << "Press 's' key to do object on table segmentation \n";
    std::cerr << "Press 'q' key to exit \n";
  }
}

int main (int argc, char** argv)
{
  ros::init(argc,argv,"object_on_table_segmenter_Node");    
  ros::NodeHandle nh("~");
  bool load_table;
  //getting subscriber parameters
  nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));

  //getting save parameters
  nh.param("save_directory",save_directory,std::string("./result"));
  nh.param("object",object_name,std::string("cloud_cluster_"));
  nh.param("pcl_viewer",dist_viewer,false);
  nh.param("save_index",cloud_save_index,0);

  nh.param("load_table",load_table,false);
  nh.param("update_table",update_table,false);
  nh.param("load_directory",load_directory,std::string("./data"));

  //getting other parameters
  nh.param("limit_X",distance_limit_enable[0],false);
  nh.param("limit_Y",distance_limit_enable[1],false);
  nh.param("limit_Z",distance_limit_enable[2],true);
  nh.param("xMin",limitX[0],0.0d);
  nh.param("yMin",limitY[0],0.0d);
  nh.param("zMin",limitZ[0],0.0d);
  nh.param("xMax",limitX[1],0.0d);
  nh.param("yMax",limitY[1],0.0d);
  nh.param("zMax",limitZ[1],1.5d);

  if (load_table)
  {
    pcl::PCDReader reader;
    reader.read (load_directory+"/table.pcd", *tableHull);
    haveTable = true;
  }
  else
    haveTable = false;

  pc_sub = nh.subscribe(POINTS_IN,1,callback);
  if (!haveTable)
    std::cerr << "Remove all object on table and press 's' key" << std::endl;
  else
  {
    std::cerr << "Press 'q' key to exit \n";
    std::cerr << "Press 's' key to do object on table segmentation \n";
  }

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
