#include <iostream>

#include "ros_sequential_scene_parsing.h"

int main(int argc, char* argv[])
{
	ros::init(argc,argv, "scene_graph_test");
    ros::NodeHandle nh ("~");
    ros::Rate r(10); //10Hz
	RosSceneGraph test(nh);
	
    while (ros::ok())
    {
        test.publishTf();
        r.sleep();
        ros::spinOnce();
    }
	return 0;
}