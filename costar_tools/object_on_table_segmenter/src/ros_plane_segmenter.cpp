#include "object_on_table_segmenter/ros_plane_segmenter.h"

RosPlaneSegmenter::~RosPlaneSegmenter()
{ 
	if (listener_)
	{
		delete (listener_);
	}
}

void RosPlaneSegmenter::initialize(const ros::NodeHandle &nh)
{
	this->initialized_ = true;
	listener_ = new (tf::TransformListener);

	bool load_table;
	bool visualization;
	nh.param("plane_segmenter_viewer",visualization,false);
	nh.param("load_table",load_table,false);
	nh.param("load_table_path",load_table_path_,std::string("./data/table.pcd"));
	
	namespace fs = boost::filesystem;
	fs::path p(load_table_path_);
	if (!fs::exists(p))
	{
		ROS_INFO("Cannot find plane data in %s.",load_table_path_.c_str());
		fs::path plane_data_directory = p.parent_path();
		if (!fs::exists(plane_data_directory))
		{
			if (fs::create_directories(plane_data_directory))
			{
				ROS_INFO("Created plane data save directory %s.",plane_data_directory.string().c_str());
			}
			else
			{
				ROS_ERROR("Fail to generate plane data save directory %s.",plane_data_directory.string().c_str());
			}
		}
	}
	else
	{
		ROS_INFO("Found plane data in %s.",load_table_path_.c_str());
	}
	
	if (load_table)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr table_hull(new pcl::PointCloud<pcl::PointXYZRGBA>());
		if (reader_.read(load_table_path_, *table_hull) < 0)
		{
			ROS_WARN("Fail to load plane data from %s.",load_table_path_.c_str());
		}
		else
		{
			ROS_INFO("Plane data loaded.");
			this->ready = true;
		}
		this->setPlaneConvexHull(*table_hull);
	}

	nh.param("use_rosbag",use_rosbag_,false);
	nh.param("update_table",update_table_,false);
	nh.param("table_tf", table_tf_name_,std::string("/table_tf"));
	nh.param("use_tf_surface",use_tf_surface_,false);
	
	double above_table_min, above_table_max;
	nh.param("above_table_min",above_table_min,0.01);
	nh.param("above_table_max",above_table_max,0.50);

	double table_distance_threshold, table_angular_threshold, table_minimal_inliers;
	nh.param("table_distance_threshold",table_distance_threshold,0.02);
	nh.param("table_angular_threshold",table_angular_threshold,2.0);
	nh.param("table_minimal_inliers",table_minimal_inliers,5000.0);

	this->setPlaneSegmentationHeight(above_table_min,above_table_max);
	this->setPlaneSegmentationParameters(table_distance_threshold,table_angular_threshold,table_minimal_inliers);
	this->setVisualizationFlag(visualization);

}

void RosPlaneSegmenter::segmentPlaneIfNotExist(const sensor_msgs::PointCloud2 &input_cloud)
{
	if (!initialized_)
	{
		std::cerr << "ERROR, RosPlaneSegmenter is not initialized yet.\n";
		return;
	}
	else if (!this->ready)
	{
		ROS_INFO("Performing plane segmentation");
		// perform table segmentation
		CloudXYZRGBA::Ptr cloud(new CloudXYZRGBA());
		fromROSMsg(input_cloud,*cloud);
		CloudXYZRGBA::Ptr cloud_filtered (new CloudXYZRGBA());
		std::string table_tf_parent;
		tf::StampedTransform transform;

		if (!listener_->frameExists(table_tf_name_))
		{
			ROS_WARN("Fail to find table frame [%s]",table_tf_name_.c_str());
			return;
		}

		listener_->getParent(table_tf_name_,ros::Time(0),table_tf_parent);
		if (use_rosbag_ || listener_->waitForTransform(table_tf_parent,table_tf_name_,ros::Time::now(),ros::Duration(1.5)))
		{
			ROS_INFO("Found table frame [%s]",table_tf_name_.c_str());
			listener_->lookupTransform(table_tf_parent,table_tf_name_,ros::Time(0),transform);
			Eigen::Affine3d box_pose;
			tf::transformTFToEigen(transform, box_pose);
			Eigen::Affine3f box_pose_float(box_pose);
			cloud_filtered = this->cropBox(*cloud, box_pose_float, Eigen::Vector3f(0.5,0.5,0.5));
		}
		else
		{
			ROS_WARN("Fail to find table frame [%s]",table_tf_name_.c_str());
			return;
		}

		CloudXYZRGBA::Ptr table_hull;

		if (use_tf_surface_)
		{
			Eigen::Affine3d plane_pose;
			tf::transformTFToEigen(transform, plane_pose);
			Eigen::Affine3f plane_pose_float(plane_pose);
			table_hull = this->generatePlaneConvexHullFromPoseXYplane(plane_pose_float,0.5);
		}
		else
		{
			table_hull = this->generatePlaneConvexHull(*cloud_filtered);
		}

		if (table_hull->size() > 0)
		{
			this->setPlaneConvexHull(*table_hull);

			if (update_table_) writer_.write<pcl::PointXYZRGBA> (load_table_path_, *table_hull, true);
		}
	}
}