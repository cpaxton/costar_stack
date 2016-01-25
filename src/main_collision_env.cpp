#include "collision_env.h"

int main(int argc, char** argv)
{
	// initialize ros node
	ros::init(argc,argv, "collision_environment");
    
    ros::NodeHandle nh ("~");
	// tf::TransformListener listener2;
	// std::cerr << "List frame \n" << listener2.allFramesAsString() << std::endl;
	// ros::Rate r(10); // 10 hz
	collision_environment environment(nh);
	ros::Rate r (10);

	while (ros::ok())
	{
        environment.addAllCollisionObject();
        r.sleep();
		// std::cerr << "List frame \n" << listener2.allFramesAsString() << std::endl;
		// environment.addAllCollisionObject();
		// ros::spin();
	}
	// load all mesh
	return 1;
}
