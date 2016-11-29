#include <ros/ros.h>
#include "sp_segmenter/ros_semantic_segmentation.h"
// #include "sp_segmenter/semantic_segmentation.h"

int main(int argc, char** argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    ros::init(argc,argv,"sp_segmenter_server");    
    ros::NodeHandle nh("~");
    ros::Rate r(10); //10Hz
    // SemanticSegmentation test;
    RosSemanticSegmentation segmenter(nh);
    
    while (ros::ok())
    {
        segmenter.publishTF();
        r.sleep();
        ros::spinOnce();
    }
    
    return 1;
}
