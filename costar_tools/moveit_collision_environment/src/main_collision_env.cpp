#include "planning_scene_generator.h"

int main(int argc, char** argv)
{
	// initialize ros node
	ros::init(argc,argv, "collision_environment");
    ros::NodeHandle nh ("~");
    
    // setting up moveit planning scene
    moveitPlanningSceneGenerator planningScene(nh);
    
    // advertise the planning scene generator service
    ros::ServiceServer planningSceneGenerator = nh.advertiseService("planningSceneGenerator", &moveitPlanningSceneGenerator::updateCollisionObject, &planningScene);

    ros::ServiceServer updatePlanningSceneBackground = nh.advertiseService("updateWithBackground", &moveitPlanningSceneGenerator::updateWithBackground, &planningScene);
    ros::spin();
	return 1;
}
