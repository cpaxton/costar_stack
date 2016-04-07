#ifndef PLANNINGSCENE_GENERATOR_H
#define PLANNINGSCENE_GENERATOR_H

#include <ros/ros.h>

// MoveIt!
#include "collision_env.h"
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
// Empty args service
#include <std_srvs/Empty.h>

// use detected object list instead of tf name convention
#include <costar_objrec_msgs/DetectedObject.h>
#include <costar_objrec_msgs/DetectedObjectList.h>

class moveitPlanningSceneGenerator
{
protected:
    ros::NodeHandle nh;
    ros::Publisher planning_scene_diff_publisher;
    ;
private:
    moveit_msgs::PlanningScene planning_scene;
    collision_environment collisionObjectGenerator;
    bool useDetectedObjectMsgs;
    ros::Subscriber getDetectedObject; 
    ;
    
public:
    void autoUpdateScene(const costar_objrec_msgs::DetectedObjectList &detectedObject);
    bool updateCollisionObject (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    void addCollisionObjects (const std::vector<moveit_msgs::CollisionObject>& collision_objects);
    moveitPlanningSceneGenerator(const ros::NodeHandle &nh);
    ;
};


#endif