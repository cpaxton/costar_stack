#include <iostream>
#include "scene_physics_engine.h"
#include "ros_sequential_scene_parsing.h"
#include <boost/thread.hpp>

void rosMainloop(ros::Rate &r, RosSceneHypothesisAssessor &test)
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
    PhysicsEngine test;
    // test.initPhysics();
    return glutmain(argc,argv, 1024,600,"Test",&test);
#else
    ros::init(argc,argv, "scene_graph_test");
    ros::NodeHandle nh ("~");
    ros::Rate r(10); //10Hz
    RosSceneHypothesisAssessor test(nh);

    bool render_scene;
    nh.param("render_scene",render_scene, true);
    if (!render_scene)
    {
        // rosMainloop(r,test);
        while (ros::ok())
        {
            test.publishTf();
            r.sleep();
            ros::spinOnce();
        }
    }
    else
    {
        // Run the physics simulation on separate thread
        boost::thread * thr = new boost::thread(boost::bind(rosMainloop, r,boost::ref(test))); 
        test.callGlutMain(argc,argv);
        ros::spin();
    }
    
#endif
    
	return 0;
}