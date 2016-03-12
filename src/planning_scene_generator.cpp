#include "planning_scene_generator.h"

void moveitPlanningSceneGenerator::addCollisionObjects(const std::vector<moveit_msgs::CollisionObject>& collision_objects)
{
    // This function will add the collision objects from collision environment to the planning scene
    std::cerr << collision_objects.size() << std::endl;
    if (collision_objects.size() < 1)
        return;
    
    for (unsigned i = 0; i < collision_objects.size(); i++)
        this->planning_scene.world.collision_objects.push_back(collision_objects.at(i));
}

bool moveitPlanningSceneGenerator::updateCollisionObject (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    // This service function will update the planning scene collision objects
    
    // remove old collision objects
    planning_scene.world.collision_objects.clear();
    std::cerr << "Number of old object to remove: ";
    this->addCollisionObjects(collisionObjectGenerator.generateOldObjectToRemove());
    
    // add new collision objects
    std::cerr << "Updating objects.\n";
    collisionObjectGenerator.updateCollisionObjects();
    
    std::cerr << "Number of new object to add: ";
    this->addCollisionObjects(collisionObjectGenerator.getCollisionObjects());
    
    bool anyUpdate = planning_scene.world.collision_objects.size() > 0;
    if (anyUpdate) {
        planning_scene.is_diff = true;
        planning_scene_diff_publisher.publish(planning_scene);
        std::cerr << "Update done\n";
    }
    else
        std::cerr << "No update done since there is no object TF detected\n";
    std::cerr << std::endl;
    return anyUpdate;
}

moveitPlanningSceneGenerator::moveitPlanningSceneGenerator(const ros::NodeHandle &nh)
{
    // This will set the nodehandle and publisher of the class
    this->nh = nh;
    planning_scene_diff_publisher = this->nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    collisionObjectGenerator.setNodeHandle(this->nh);
}