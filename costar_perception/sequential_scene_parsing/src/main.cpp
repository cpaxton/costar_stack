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

void publishTF(ros::Rate &r, RosSceneHypothesisAssessor &test)
{
    while (ros::ok())
    {
        test.publishTf();
        r.sleep();
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
    ros::MultiThreadedSpinner spinner(4);

    bool render_scene;
    nh.param("render_scene",render_scene, true);
    boost::thread * thr = new boost::thread(boost::bind(publishTF, r,boost::ref(test)));
    boost::thread * thr2;
    if (render_scene)
    {
        // Run the physics simulation on separate thread
        // boost::thread * thr = new boost::thread(boost::bind(rosMainloop, r,boost::ref(test)));
        // boost::thread * thr = new boost::thread(boost::bind(publishTF, r,boost::ref(test)));
        // test.callGlutMain(argc,argv);

        // boost::thread * thr = new boost::thread(boost::bind(&RosSceneHypothesisAssessor::publishTf,boost::ref(test)));
        thr2 = new boost::thread(boost::bind(&RosSceneHypothesisAssessor::callGlutMain,boost::ref(test),argc,argv));
    }
    ros::spin();
    thr->join();
    if (thr2) 
    {
        test.exitGlutMain();
        thr2->join();
    }
#endif
    
	return 0;
}