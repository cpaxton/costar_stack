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

    // Keep the node from updating collision object with autoUpdate if the service call is running first
    mtx.lock();

    // remove old collision objects
    planning_scene.world.collision_objects.clear();
    std::cerr << "Number of old object to remove: ";
    this->addCollisionObjects(collisionObjectGenerator.generateOldObjectToRemove());
    
    // add new collision objects
    std::cerr << "Updating objects.\n";
    if(useDetectedObjectMsgs)
    {
        collisionObjectGenerator.getAllObjectTFfromDetectedObjectMsgs(detectedObjectList);
        collisionObjectGenerator.updateCollisionObjects(false);
    }
    else
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
    mtx.unlock();
    return anyUpdate;
}

void moveitPlanningSceneGenerator::autoUpdateScene(const costar_objrec_msgs::DetectedObjectList &detectedObject)
{
    // this function will automatically update scene when it got detectedObject msgs
    // remove old collision objects

    // Keep the node from updating collision object with service call if the autoupdate is running first
    mtx.lock();
    std::cerr << "Received DetectedObjectList, automatically update planning scene now. \n";
    this->detectedObjectList = detectedObject;

    planning_scene.world.collision_objects.clear();
    std::cerr << "Number of old object to remove: ";
    this->addCollisionObjects(collisionObjectGenerator.generateOldObjectToRemove());

    // Get update of tf names from detected object msgs
    std::cerr << "Updating objects.\n";
    collisionObjectGenerator.getAllObjectTFfromDetectedObjectMsgs(detectedObjectList);

    // Update collision objects without updating frame (because we already update it from the msgs)
    collisionObjectGenerator.updateCollisionObjects(false);

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
    mtx.unlock();
}

moveitPlanningSceneGenerator::moveitPlanningSceneGenerator(const ros::NodeHandle &nh)
{
    // This will set the nodehandle and publisher of the class
    this->nh = nh;
    planning_scene_diff_publisher = this->nh.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);
    collisionObjectGenerator.setNodeHandle(this->nh);

    this->nh.param("useDetectedObjectMsgs", useDetectedObjectMsgs, false);
    if(useDetectedObjectMsgs){
        std::cerr << "This node will update the planning scene automatically when it receieved the costar object msgs\n";
        std::string detectedObjectTopic;
        this->nh.param("detectedObjectTopic",detectedObjectTopic,std::string("/SPServer/detected_object_list"));
        getDetectedObject = this->nh.subscribe(detectedObjectTopic,1,&moveitPlanningSceneGenerator::autoUpdateScene,this);
    }
    else std::cerr << "This node need service call to update the planning scene.\n";
}

bool moveitPlanningSceneGenerator::updateWithBackground(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    this->nh.setParam("renewTable", true);
    bool result = this->updateCollisionObject(request, response);
    this->nh.setParam("renewTable", false);
    return result;
}