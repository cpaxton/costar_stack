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
#include <tf/transform_listener.h>
// for using tf surface
#include <pcl_ros/transforms.h>
// for segment object above table
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>

// for creating directory automatically
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

bool dist_viewer ,haveTable,update_table;
std::string POINTS_IN;
std::string save_directory, object_name, load_directory, original_directory, ground_truth_directory;
std::string tableTFname;
int cloud_save_index;
ros::Subscriber pc_sub;
pcl::PCDWriter writer;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tableHull(new pcl::PointCloud<pcl::PointXYZRGBA>);
tf::TransformListener * listener;
double aboveTableMin;
double aboveTableMax;
double time_step;
bool keyPress = false;
bool run_auto;
int num_to_capture = 0;
bool useTFsurface;
bool useRosbag;
bool doCluster;

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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filter_distance(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input, const tf::StampedTransform &transform)
{
  //std::cerr << "Cloud points before distance filtering: " << cloud_input->points.size() << std::endl;
  double region = 0.5;
  for (unsigned int i = 0; i < 3; i++)
  {
    pcl::PassThrough<pcl::PointXYZRGBA> pass;
    std::cerr << "cloud input organized" << cloud_input->isOrganized() << std::endl;
    pass.setInputCloud (cloud_input);
    pass.setKeepOrganized(true);
    std::string axisName;
    double positionToSegment;

    switch (i){
      case 0:
        axisName = "x";
        positionToSegment = transform.getOrigin().getX();
        break;
      case 1: 
        axisName = "y"; 
        positionToSegment = transform.getOrigin().getY();
        break;
      default: 
        axisName = "z";
        positionToSegment = transform.getOrigin().getZ();
        break;
    }
    std::cerr << "Segmenting axis: " << axisName << "max: " << positionToSegment + region << " min: " << positionToSegment - region << std::endl;
    pass.setFilterFieldName (axisName);
    pass.setFilterLimits (positionToSegment - region, positionToSegment + region);
    pass.filter (*cloud_input);
    std::cerr << "cloud output organized" << cloud_input->isOrganized() << std::endl;
  }
  return cloud_input;
}

void segmentCloudAboveTable(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_input, const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &convexHull)
{
  std::cerr << "cloud input organized" << cloud_input->isOrganized() << std::endl;
  std::cerr << "\nSegment object above table \n";
  // Prism object.
  pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBA> prism;
  prism.setInputCloud(cloud_input);
  prism.setInputPlanarHull(convexHull);

  // from 1 cm above table to 50 cm above table
  //prism.setHeightLimits(0.135f, 0.5f);
  prism.setHeightLimits(aboveTableMin,aboveTableMax);
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
  std::cerr << "cloud output organized" << cloud_input->isOrganized() << std::endl;
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

void saveCloud(pcl::PointCloud<pcl::PointXYZRGBA> cloud_input, std::string dir, std::string additional_text = std::string(""))
{
    using namespace boost::posix_time;
    using namespace std;

  std::stringstream ss;
  boost::shared_ptr<time_facet>facet(new boost::posix_time::time_facet("%Y_%m_%d_%H_%M_%S_"));
  ss.imbue(std::locale(ss.getloc(), facet.get()));
  ss << dir << boost::posix_time::second_clock::local_time() << object_name << "_" << cloud_save_index << additional_text << ".pcd";
  writer.write<pcl::PointXYZRGBA> (ss.str (), cloud_input, true);
  std::cerr << "Saved " << ss.str();
  cloud_save_index++;
}

void cloud_segmenter_and_save(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud_filtered)
{
  std::cerr << "Object Segmentation process begin \n";
  if (dist_viewer)
  {
    std::cerr << "See distance filtered cloud. Press q to quit viewer. \n";
    pcl::visualization::CloudViewer viewer ("Distance Filtered Cloud Viewer");
    viewer.showCloud (cloud_filtered);
    while (!viewer.wasStopped ())
    {
    }
    dist_viewer = false;
    writer.write<pcl::PointXYZRGBA> (load_directory+"distance_filtered.pcd", *cloud_filtered, true);
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
  unsigned int initialIndex = cloud_save_index;
  // save detected cluster data
  for (size_t i = 0; i < euclidean_label_indices.size (); i++)
  {
    if (euclidean_label_indices[i].indices.size () > 500)
    {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

      pcl::PointIndices::Ptr object_cloud_indices (new pcl::PointIndices);
      *object_cloud_indices = euclidean_label_indices[i];
      extract.setInputCloud(cloud_filtered);
      extract.setIndices(object_cloud_indices);
      extract.setKeepOrganized(true);
      extract.filter(*cloud_cluster);

      saveCloud(*cloud_cluster,ground_truth_directory, "_ground_truth");
      std::cerr << "\tcluster size: "<< euclidean_label_indices[i].indices.size () << std::endl;
    }
  }

  std::cerr << "Segmented object: " << cloud_save_index - initialIndex <<". Segmentation done.\n Waiting for keypress to get new data \n";
}



pcl::PointCloud<pcl::PointXYZRGBA>::Ptr useTFConvexHull(tf::StampedTransform transform, double distance = 0.5)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tableTmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int i = -1; i < 2; i+=2)
    for (int j = -1; j < 2; j+=2)
    {
      pcl::PointXYZRGBA tmp;
      tmp.x = i * distance/2;
      tmp.y = j * distance/2;
      tmp.z = 0;
      tableTmp->push_back(tmp);
    }
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tfTable(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl_ros::transformPointCloud (*tableTmp, *tfTable, transform);
  writer.write<pcl::PointXYZRGBA> (load_directory+"TF_boundary.pcd", *tfTable, true);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGBA>);

  pcl::ConvexHull<pcl::PointXYZRGBA> hull;
  hull.setInputCloud(tfTable);
  // Make sure that the resulting hull is bidimensional.
  hull.setDimension(2);
  hull.reconstruct(*convexHull);
  writer.write<pcl::PointXYZRGBA> (load_directory+"TFconvexHull.pcd", *tfTable, true);
  return convexHull;
}

void callback(const sensor_msgs::PointCloud2 &pc)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  // convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA
  pcl::fromROSMsg(pc, *cloud);
  if (haveTable and keyPress)
  {
    std::stringstream ss;
    ss << original_directory << object_name << cloud_save_index << "_original_cloud.pcd";
    std::cout << "original saved to :" << ss.str() << "\n";
    try {
      saveCloud(*cloud,original_directory,"_original");
    } catch (pcl::IOException e) {
      ROS_ERROR("could not write to %s!",ss.str().c_str());
    }
    segmentCloudAboveTable(cloud,tableHull);
    if (doCluster)
	    cloud_segmenter_and_save(cloud);
	else saveCloud(*cloud,ground_truth_directory,"_ground_truth");
  }
  else
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::string tableTFparent;
    tf::StampedTransform transform;
    listener->getParent(tableTFname,ros::Time(0),tableTFparent);
    if (useRosbag || listener->waitForTransform(tableTFparent,tableTFname,ros::Time::now(),ros::Duration(1.5)))
    {
      std::cerr << "Table TF with name: '" << tableTFname << "' found with parent frame: " << tableTFparent << std::endl;
      listener->lookupTransform(tableTFparent,tableTFname,ros::Time(0),transform);
      cloud_filtered = cloud_filter_distance(cloud, transform);
    }
    else{
      std::cerr << "Fail to get TF: "<< tableTFname << std::endl;
      return;
    }

    if (dist_viewer)
    {
      std::cerr << "See distance filtered cloud. Press q to quit viewer. \n";
      pcl::visualization::CloudViewer viewer ("Distance Filtered Cloud Viewer");
      viewer.showCloud (cloud_filtered);
      while (!viewer.wasStopped ())
      {
      }
    }
    if (useTFsurface)
      tableHull = useTFConvexHull(transform);
    else
      tableHull = getTableConvexHull(cloud_filtered);

    if (update_table)
      writer.write<pcl::PointXYZRGBA> (load_directory+"/table.pcd", *tableHull, true);
    haveTable = true;

    std::cerr << "Add new object to the table \n";
    std::cerr << "Press 's' key to do object on table segmentation \n";
    std::cerr << "Press 'q' key to exit \n";
  }
}

void callbackCaptureEnvironment(const sensor_msgs::PointCloud2 &pc)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
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
  bool load_table;
  //getting subscriber parameters
  nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));

  //getting save parameters
  nh.param("save_directory",save_directory,std::string("./"));
  nh.param("original_directory",original_directory,std::string("original/"));
  nh.param("ground_truth_directory",ground_truth_directory,std::string("ground_truth/"));
  nh.param("object",object_name,std::string("cloud_cluster_"));
  nh.param("pcl_viewer",dist_viewer,false);
  nh.param("save_index",cloud_save_index,0);
  nh.param("num_to_capture",num_to_capture,200);

  nh.param("time_step",time_step,0.1);
  nh.param("run_auto",run_auto,false);

  nh.param("load_table",load_table,false);
  nh.param("update_table",update_table,false);
  nh.param("load_directory",load_directory,std::string("./data"));
  nh.param("tableTF", tableTFname,std::string("/tableTF"));
  nh.param("useTFsurface",useTFsurface,false);
  nh.param("useRosbag",useRosbag,false);
  nh.param("doCluster",doCluster,true);

  nh.param("aboveTableMin",aboveTableMin,0.135);
  nh.param("aboveTableMax",aboveTableMax,0.50);

  listener = new (tf::TransformListener);

  bool justCaptureEnvironment;
  nh.param("environment_only",justCaptureEnvironment,false);
  boost::filesystem::create_directories(load_directory);
  boost::filesystem::create_directories(save_directory);
  boost::filesystem::create_directories(original_directory);
  boost::filesystem::create_directories(ground_truth_directory);

  if (justCaptureEnvironment)
  { 
    pc_sub = nh.subscribe(POINTS_IN,1,callbackCaptureEnvironment);
    ros::Rate r(1);
    while (ros::ok()){
      r.sleep();
      ros::spinOnce();
    }
  }
  else
  {
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

    if (not run_auto) {
      while (ros::ok())
      {
        key = getch();
        if ((key == 's') || (key == 'S'))
          ros::spinOnce();
        else if ((key == 'q') || (key == 'Q'))
          break;
        r.sleep();
      }
    } else {
      // run loop with time
      int count = 0;
      while (ros::ok()) {

        if (not haveTable) {
          key = getch();
          if ((key == 's') || (key == 'S'))
            ros::spinOnce();
          else if ((key == 'q') || (key == 'Q'))
            break;
        } else if (not keyPress) {
          std::cout << "Press any key to start data collection...\n";
          key = getch();
          keyPress = true;
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
  delete (listener);
  ros::shutdown();
  return (0);
}
