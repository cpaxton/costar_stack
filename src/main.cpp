#include <iostream>

#include "ros_sequential_scene_parsing.h"
#include <boost/thread.hpp>

void rosMainloop(ros::Rate &r, RosSceneGraph &test)
{
    while (ros::ok())
    {
        test.publishTf();
        r.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv, "scene_graph_test");
    ros::NodeHandle nh ("~");
    ros::Rate r(10); //10Hz
	RosSceneGraph test(nh);
	
    // Run this on separate thread
    rosMainloop(r,test);
    
	return 0;
}