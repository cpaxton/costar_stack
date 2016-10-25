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
        if (test.exit_) break;
    }
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv, "scene_graph_test");
    ros::NodeHandle nh ("~");
    ros::Rate r(10); //10Hz
	RosSceneGraph test(nh);
	
    // Run this on separate thread
    // boost::thread * thr = new boost::thread(boost::bind(rosMainloop, r,boost::ref(test))); 
    // ros::spin();
    rosMainloop(r,test);
    return glutmain(argc,argv, 1024,600,"Test",&test.physics_engine_);
    // Run the physics simulation
    // test.callGlutMain(argc,argv);
    
	return 0;
}