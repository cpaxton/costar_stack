#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <object_on_table_segmenter/ros_plane_segmenter.h>

#include "color_nn_segmenter.h"

sensor_msgs::PointCloud2 cached_cloud;
ros::Publisher pc_pub, pc_vis_pub, seg_done;
ColorNnSegmenter color_based_segmenter;

RosPlaneSegmenter plane_segmenter;
bool use_table, have_table, update_table;
std::string table_tf_name, load_table_path;
pcl::PCDWriter writer;
tf::TransformListener * listener;
bool use_tf_surface;
bool label_as_rgb;

uchar color_label[11][3] =
{
    {255, 255, 255},
    {255, 0, 0},
    {0, 255, 0},
    {0, 0, 255},
    {255, 255, 0},
    {255, 0, 255},
    {0, 255, 255},
    {255, 128, 0},
    {255, 0, 128},
    {0, 128, 255},
    {128, 0, 255}
};

pcl::PointCloud<pcl::PointXYZRGBA> convertPointCloudLabelToRGBA(const PointCloudXYZL &input)
{
    pcl::PointCloud<pcl::PointXYZRGBA> result;
    for (pcl::PointCloud<pcl::PointXYZL>::const_iterator it = input.begin(); it != input.end(); ++it)
    {
        pcl::PointXYZRGBA point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        point.r = color_label[it->label][0];
        point.b = color_label[it->label][1];
        point.g = color_label[it->label][2];
        point.a = 255;
        result.push_back(point);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZRGBA> convertPointCloudLabelToRChannel(const PointCloudXYZL &input)
{
	std::map<std::size_t,std::size_t> label_counter;
    pcl::PointCloud<pcl::PointXYZRGBA> result;
    for (pcl::PointCloud<pcl::PointXYZL>::const_iterator it = input.begin(); it != input.end(); ++it)
    {
        pcl::PointXYZRGBA point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        point.r = it->label;
        point.b = 0;
        point.g = 0;
        point.a = 255;
        result.push_back(point);
        if (label_counter.find(it->label)==label_counter.end())
        {
        	label_counter[it->label] = 1;
        }
        else
        {
        	label_counter[it->label]++;
        }
    }

    for (std::map<std::size_t, std::size_t>::const_iterator it = label_counter.begin(); it!= label_counter.end(); ++it)
    {
    	std::cerr << "Number of points in label: " << it->first << " = " << it->second << std::endl; 
    }
    return result;
}

void cacheCloudInput(const sensor_msgs::PointCloud2 &input_cloud)
{
	// std::cerr << "Updated cached cloud.\n";
	cached_cloud = input_cloud;

	if (use_table)
	{
		plane_segmenter.segmentPlaneIfNotExist(input_cloud);
	}
}

bool colorSegmenter(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	PointCloudXYZRGB::Ptr input_cloud(new PointCloudXYZRGB());

	if (use_table)
	{
		ROS_INFO("Removing background with plane segmentation.");
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
		fromROSMsg(cached_cloud,*filtered_cloud);
		filtered_cloud = plane_segmenter.segmentAbovePlane(*filtered_cloud);
		pcl::copyPointCloud(*filtered_cloud,*input_cloud);
	}
	else
	{
		fromROSMsg(cached_cloud,*input_cloud);
	}
	ROS_INFO("Performing color based segmentation.");

	// std::cerr << "Input cloud size: " << input_cloud->size() << std::endl;
	PointCloudXYZL::Ptr seg_cloud = color_based_segmenter.segment(*input_cloud);

	sensor_msgs::PointCloud2 output_msg;
	if (label_as_rgb)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr r_as_label_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
		*r_as_label_cloud = convertPointCloudLabelToRChannel(*seg_cloud);
		toROSMsg(*r_as_label_cloud,output_msg);
	}
	else
	{
		toROSMsg(*seg_cloud,output_msg);
	}
	output_msg.header.frame_id = cached_cloud.header.frame_id;
	pc_pub.publish(output_msg);
	
	pcl::PointCloud<pcl::PointXYZRGBA> seg_cloud_vis = convertPointCloudLabelToRGBA(*seg_cloud);
	sensor_msgs::PointCloud2 output_msg2;
	toROSMsg(seg_cloud_vis,output_msg2);
	output_msg2.header.frame_id = cached_cloud.header.frame_id;

	pc_vis_pub.publish(output_msg2);
	seg_done.publish(std_msgs::Empty());
	ROS_INFO("Done.");
	return true;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv, "color_based_segmenter");
	ros::NodeHandle nh("~");
	ros::Rate r(60); // 60 Hz

	bool load_existing_model;
	nh.param("load_existing_model",load_existing_model,false);
	std::string model_name;
	nh.param("model_name",model_name,std::string("model"));

    // load all the models, they are retrieved in lexicographic order
	if (load_existing_model)
	{
		std::string model_directory;
		nh.param("model_directory",model_directory,std::string(""));
		boost::filesystem::path p(model_directory);
		p /= model_name + ".dat";

		if (!color_based_segmenter.loadModel(p.string()))
		{
			ROS_ERROR("Fail to load the color model.");
			return -1;
		}
		else
		{
			ROS_INFO("Successfully load the color model.");
		}
	}
	else
	{
		std::string training_data_directory;
		int kmeans_point_per_model;

		nh.param("training_data_directory",training_data_directory,std::string(""));
		nh.param("kmeans_point_per_model",kmeans_point_per_model,5);
		
		ROS_INFO("Generating a new color model");
		if (!color_based_segmenter.trainModel(training_data_directory, kmeans_point_per_model))
		{
			ROS_ERROR("Fail to generate a new color model.");
			return -1;
		}

		ROS_INFO("New color model generated");

		bool save_new_model;
		nh.param("save_new_model",save_new_model,true);
		if (save_new_model)
		{
			std::string save_directory;
			nh.param("save_directory",save_directory,std::string(""));
			
			if (color_based_segmenter.saveModel(save_directory,model_name))
			{
				ROS_INFO("New color model saved");	
			}
			else
			{
				ROS_WARN("Fail to save the new color model");
			}
		}
		else
		{
			ROS_INFO("Skipped saving the new color model");
		}
	}

	std::string cloud_in, cloud_out, cloud_out_vis, segment_done;
	nh.param("cloud_input_topic",cloud_in,std::string("/camera/depth_registered/points"));
	nh.param("segmented_cloud_topic",cloud_out,std::string("segmented_cloud"));
	nh.param("visualized_cloud_topic",cloud_out_vis,std::string("visualized_cloud"));
	nh.param("segmentation_done_topic",segment_done,std::string("segment_done"));

	nh.param("use_plane_segmentation",use_table,false);
	nh.param("send_label_as_rgb",label_as_rgb,true);

	std::string background_labels, foreground_labels;
	nh.param("background_labels",background_labels,std::string(""));
	nh.param("foreground_labels",foreground_labels,std::string(""));

    // update the remapped labels
	color_based_segmenter.setColorLabels(foreground_labels,background_labels);

	if (use_table)
	{
		plane_segmenter.initialize(nh);
	}

	ros::Subscriber cloud_subscriber = nh.subscribe(cloud_in,1,cacheCloudInput);
	pc_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_out,1000);
	pc_vis_pub = nh.advertise<sensor_msgs::PointCloud2>(cloud_out_vis,1000);
	seg_done = nh.advertise<std_msgs::Empty>(segment_done,1);
	ros::ServiceServer segmenter_service = nh.advertiseService("segmenter",colorSegmenter);

	ros::spin();
	
	return 0;
}