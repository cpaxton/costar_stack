//#include <ros/ros.h>
#include "sp_segmenter/semanticSegmentation.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sp_segmenter_server");    
    ros::NodeHandle nh("~");
    ros::Rate r(10); //10Hz
    semanticSegmentation segmenter(argc, argv, nh);
    
    while (ros::ok())
    {
        segmenter.publishTF();
        r.sleep();
        ros::spinOnce();
    }
    
    return 1;
}
