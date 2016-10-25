#include <iostream>
#include "scene_physics_engine_w_rendering.h"
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
#if 0
    PhysicsEngineWRender test;
    // test.initPhysics();
    return glutmain(argc,argv, 1024,600,"Test",&test);
#else
    ros::init(argc,argv, "scene_graph_test");
    ros::NodeHandle nh ("~");
    ros::Rate r(10); //10Hz
    RosSceneGraph test(nh);
    // Run this on separate thread
    boost::thread * thr = new boost::thread(boost::bind(rosMainloop, r,boost::ref(test))); 
    // ros::spin();
    // Run the physics simulation
    test.callGlutMain(argc,argv);
#endif
    
	return 0;
}