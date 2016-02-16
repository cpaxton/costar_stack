#include "collision_env.h"

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/move_group_interface/move_group.h>

//#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <std_srvs/Empty.h>

class moveitPlanningSceneGenerator
{
    protected:
        ros::NodeHandle nh;
        ros::Publisher planning_scene_diff_publisher;
    ;
    private:
        moveit::planning_interface::MoveGroup * group;
        ros::Publisher display_publisher;
//        moveit_msgs::DisplayTrajectory display_trajectory;
        moveit_msgs::PlanningScene planning_scene;
        collision_environment collisionObjectGenerator;
    ;
    
    public:
        bool updateCollisionObject (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
        void addCollisionObjects (const std::vector<moveit_msgs::CollisionObject>& collision_objects);
        moveitPlanningSceneGenerator(const ros::NodeHandle &nh);
};

void moveitPlanningSceneGenerator::addCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collision_objects)
{
    for (unsigned i = 0; i < collision_objects.size(); i++)
    {
        this->planning_scene.world.collision_objects.push_back(collision_objects.at(i));
    }
}

bool moveitPlanningSceneGenerator::updateCollisionObject (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    // remove old collision objects
    planning_scene.world.collision_objects.clear();
    this->addCollisionObjects(collisionObjectGenerator.generateOldObjectToRemove());
    
    // add new collision objects
    collisionObjectGenerator.updateCollisionObjects();
    this->addCollisionObjects(collisionObjectGenerator.getCollisionObjects());
    
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    return true;
}

moveitPlanningSceneGenerator::moveitPlanningSceneGenerator(const ros::NodeHandle &nh)
{
    this->nh = nh;
    this->group = new moveit::planning_interface::MoveGroup("robot_environment");
//    display_publisher = this->nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = this->nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    collisionObjectGenerator.setNodeHandle(this->nh);
}

int main(int argc, char** argv)
{
	// initialize ros node
	ros::init(argc,argv, "collision_environment");
    ros::NodeHandle nh ("~");
    
    // setting up moveit
    moveitPlanningSceneGenerator planningScene(nh);
	ros::Rate r (10);
    
	while (ros::ok())
	{
        r.sleep();
		// std::cerr << "List frame \n" << listener2.allFramesAsString() << std::endl;
		// environment.addAllCollisionObject();
		// ros::spin();
	}
	// load all mesh
	return 1;
}
