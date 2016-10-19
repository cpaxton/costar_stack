#include <iostream>

#include "ros_sequential_scene_parsing.h"

int main(int argc, char* argv[])
{
	ros::init(argc,argv, "scene_graph_test");
    ros::NodeHandle nh ("~");

	RosSceneGraph test(nh);
	ros::spin();
	return 0;
}