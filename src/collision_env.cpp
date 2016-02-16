#include "collision_env.h"

std::vector<std::string> stringVectorSeparator (const std::string &input,
                                                const std::string &charToFind, const std::string &breakChar, const int &loopInterval, const bool &debug)
{
    // This function will break the string to vector and alternate between finding input char and
    // break char every n value. If loopInterval = 3, it will try to find 2 charToFind and a breakChar before finding charToFind again
    std::vector<std::string> result;
    std::size_t found = input.find(charToFind);
    std::size_t pos = 0;
    std::size_t index = 1;
    std::string lastTmp = charToFind;
    
    bool findSomething = (found!=std::string::npos);
    if (!findSomething) {
        return result;
    }
    
    while (found!=std::string::npos)
    {
        std::string buffer;
        buffer.assign(input, pos, found - pos);
        result.push_back(buffer);
        if (debug)
            std::cerr << "Found '" << lastTmp << "' result: " << buffer << " " << index <<std::endl;
        pos = found + lastTmp.length();
        index++;
        if (index % loopInterval == 0 && loopInterval > 1)
            lastTmp = breakChar;
        else lastTmp = charToFind;
        found = input.find(lastTmp,pos);
    }
    
    std::string buffer;
    buffer.assign(input, pos, input.length() - pos);
    result.push_back(buffer);
    return result;
}

collision_environment::collision_environment()
{
    classReady = false; // not ready to be used
}

collision_environment::collision_environment(const ros::NodeHandle &nh)
{
    classReady = true;
    this->nh = nh;
    nh.param("loopInterval", loopInterval,3);
    nh.param("objectNameIdx", objectNameFormatIndex,2);
    nh.param("debug", debug,false);
    nh.param("breakChar", breakChar,std::string(" "));
    nh.param("charToFind", charToFind,std::string("::"));
    nh.param("mesh_source", mesh_source,std::string("data/mesh"));
    
    this->segmentedObjects = new std::vector<moveit_msgs::CollisionObject>();
    
	std::cerr << "Started \n";
//	add_collision_object_publisher = this->nh.advertise<moveit_msgs::CollisionObject>("collision_objects",1000);
	sleep(1.0);
	getAllObjectTF();
	// listOfTF = listener.allFramesAsString();
	// std::cerr << "Initial List of all frames:\n" << listOfTF << std::endl;
};

void collision_environment::setNodeHandle(const ros::NodeHandle &nh)
{
    classReady = true;
    this->nh = nh;
    nh.param("loopInterval", loopInterval,3);
    nh.param("objectNameIdx", objectNameFormatIndex,2);
    nh.param("debug", debug,false);
    nh.param("breakChar", breakChar,std::string(" "));
    nh.param("charToFind", charToFind,std::string("::"));
    nh.param("mesh_source", mesh_source,std::string("data/mesh"));
    
    this->segmentedObjects = new std::vector<moveit_msgs::CollisionObject>();
    
    std::cerr << "Started \n";
    sleep(1.0);
    getAllObjectTF();
}

void collision_environment::updateCollisionObjects(const bool &updateFrame)
{
    if (!classReady)
    {
        std::cerr << "ERROR, class has no nodehandle to subscribe.\n";
        return;
    }
    
	// update list of object
    if (updateFrame) {
        getAllObjectTF();
    }
    
    segmentedObjects->clear();
//    co.meshes.resize(detectedObjectsTF.size());
//    co.mesh_poses.resize(detectedObjectsTF.size());
    
    for (int i = 0; i < detectedObjectsTF.size(); i++) {
        moveit_msgs::CollisionObject co;
        co.id = detectedObjectsTF.at(i).frame_id;
        co.header.frame_id = parentFrame;
        std::stringstream ss;
        ss << "file://" << mesh_source << "/" << detectedObjectsTF.at(i).name << ".obj";
        
        shapes::Mesh * tmpMesh = shapes::createMeshFromResource(ss.str());
        shape_msgs::Mesh co_mesh;
        shapes::ShapeMsg co_mesh_msg;
        shapes::constructMsgFromShape(tmpMesh,co_mesh_msg);
        co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
        co.meshes.push_back(co_mesh);
        co.mesh_poses.push_back(detectedObjectsTF.at(i).pose);
        co.operation = co.ADD;
        segmentedObjects->push_back(co);
    }
    
//    add_collision_object_publisher.publish(co);
    if (debug) std::cerr << "Collision Object Published\n";
    
	// load the mesh of all detected object

	// add the collision object based on TF position and mesh
	
};

void collision_environment::getAllObjectTF()
{
	// get the TF list
    listener.getFrameStrings(listOfTF);
    
    if (debug) {
        for (unsigned int i = 0; i < listOfTF.size(); i++) {
            std::cerr << "Frame " << i << ": " << listOfTF.at(i) <<std::endl;
        }
    }
    
    // get parent of objectTF
    listener.getParent(listOfTF.at(0),ros::Time(0),parentFrame);
    std::cerr << "parentFrame: " << parentFrame << std::endl;
    
	detectedObjectsTF.clear();
    std::vector<std::string> newestListOfTF;
    
    ros::Time now = ros::Time::now();
    // get the name of TFs and its object name
    for (unsigned int i = 0; i < listOfTF.size(); i++) {
        std::vector<std::string> tmp = stringVectorSeparator(listOfTF.at(i), charToFind, breakChar, loopInterval, debug);
        if (tmp.size() > objectNameFormatIndex)
        {
            if (listener.waitForTransform(listOfTF.at(i),parentFrame,now,ros::Duration(3.0)))
            {
                objectTF detectedObject;
                detectedObject.name = tmp.at(objectNameFormatIndex-1);
                detectedObject.frame_id = listOfTF.at(i);
                newestListOfTF.push_back(detectedObject.frame_id);
                tf::StampedTransform transform;
                listener.lookupTransform(listOfTF.at(i),parentFrame,ros::Time(0),transform);
                geometry_msgs::TransformStamped msg;
                transformStampedTFToMsg(transform, msg);
                
                detectedObject.pose.position.x = msg.transform.translation.x;
                detectedObject.pose.position.y = msg.transform.translation.y;
                detectedObject.pose.position.z = msg.transform.translation.z;
                detectedObject.pose.orientation = msg.transform.rotation;
                
                detectedObjectsTF.push_back(detectedObject);
            }
        }
	}
    
    listOfTF = newestListOfTF; // update list of TF to contain only newest set of object TF frames
    
	// filter the TF list to display the object list
    if (debug) {
        std::cerr <<"Num of Objects: " << detectedObjectsTF.size() <<std::endl;
        for (unsigned int i = 0; i < detectedObjectsTF.size(); i++)
            std::cerr << "name: " << detectedObjectsTF.at(i).name << ", frame id: " << detectedObjectsTF.at(i).frame_id << std::endl;
    }
};

const std::vector<moveit_msgs::CollisionObject> collision_environment::generateOldObjectToRemove() const
{
    std::vector<moveit_msgs::CollisionObject> objectsToRemove;
    for (unsigned int i = 0; i < listOfTF.size(); i++) {
        moveit_msgs::CollisionObject co;
        co.id = detectedObjectsTF.at(i).frame_id;
        co.header.frame_id = parentFrame;
        co.operation = co.REMOVE;
        objectsToRemove.push_back(co);
    }
    return objectsToRemove;
}

const std::vector<moveit_msgs::CollisionObject> collision_environment::getCollisionObjects() const
{
    return *segmentedObjects;
}

std::vector<std::string> collision_environment::getListOfTF() const
{
    return listOfTF;
}