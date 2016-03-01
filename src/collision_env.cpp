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
    this -> setNodeHandle(nh);
};

void collision_environment::setNodeHandle(const ros::NodeHandle &nh)
{
    classReady = true;
    hasParent = false;
    hasTableTF = false;
    hasObjects = false;
    this->nh = nh;
    nh.param("loopInterval", loopInterval,3);
    nh.param("objectNameIdx", objectNameFormatIndex,2);
    nh.param("debug", debug,false);
    nh.param("breakChar", breakChar,std::string(" "));
    nh.param("charToFind", charToFind,std::string("::"));
    nh.param("mesh_source", mesh_source,std::string("data/mesh"));
    nh.param("tableTFname",tableTFname,std::string("tableTF"));
    nh.param("defineParent",defineParent,false);
    if (defineParent) {
        nh.param("parentFrameName",definedParent,std::string("base_link"));
    }
    
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
    
    segmentedObjects->clear();
    
    bool updateTable;
    nh.param("renewTable", updateTable, true);
    if (updateTable) {
        if (getTable())
            segmentedObjects->push_back(this->tableObject);
    }
    else
        if (hasTableTF) {
            segmentedObjects->push_back(this->tableObject);
        }
    
    if (debug)
        std::cerr << "Number of object after add table: " << segmentedObjects->size() <<std::endl;

	// update list of object
    if (updateFrame) {
        getAllObjectTF();
        if (listOfTF.size() == 0) {
            hasObjects = false;
            std::cerr << "No Object TF available.\n";
            return;
        }
    }
    else hasObjects = true;
    
//    co.meshes.resize(detectedObjectsTF.size());
//    co.mesh_poses.resize(detectedObjectsTF.size());
    
    for (int i = 0; i < detectedObjectsTF.size(); i++) {
        moveit_msgs::CollisionObject co;
        co.id = detectedObjectsTF.at(i).frame_id;
        co.header.frame_id = parentFrame;
        std::stringstream ss;
        ss << "file://" << mesh_source << "/" << detectedObjectsTF.at(i).name << ".stl";
        
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
    
    if (debug) {
        std::cerr << "Number of object after add objects: ";
        std::cerr << segmentedObjects->size() <<std::endl;
        std::cerr << "Collision Object Published\n";
    }
};

void collision_environment::getAllObjectTF()
{
	// get the TF list
    listener.getFrameStrings(listOfTF);
    if (listOfTF.size() == 0) {
        std::cerr << "No Frame available yet.\n";
        return;
    }
    
    if (debug) {
        for (unsigned int i = 0; i < listOfTF.size(); i++) {
            std::cerr << "Frame " << i << ": " << listOfTF.at(i) <<std::endl;
        }
    }
    
    if (!hasParent) //searching for object tf parent
        for (unsigned int i = 0; i < listOfTF.size(); i++) {
            std::vector<std::string> tmp = stringVectorSeparator(listOfTF.at(i), charToFind, breakChar, loopInterval, debug);
            if (tmp.size() > objectNameFormatIndex)
            {
                // get parent of objectTF
                if (!defineParent) {
                    listener.getParent(listOfTF.at(i),ros::Time(0),parentFrame);
                    if (debug)
                        std::cerr << "parentFrame of ObjectTF ->" << parentFrame << std::endl;
                }
                else
                    parentFrame = definedParent;
                hasParent = true;
                break;
            }
        }
    
	detectedObjectsTF.clear();
    std::vector<std::string> newestListOfTF;
    
    ros::Time now = ros::Time::now();
    
    if (!hasParent) {
        //something wrong going on here, probably no object frame found
        std::cerr << "TF fail to any Object frames\n";
        listOfTF.clear();
        return;
    }

    // get the name of TFs and its object name
    for (unsigned int i = 0; i < listOfTF.size(); i++) {
        std::vector<std::string> tmp = stringVectorSeparator(listOfTF.at(i), charToFind, breakChar, loopInterval, debug);
        if (tmp.size() > objectNameFormatIndex)
        {
            if (listener.waitForTransform(listOfTF.at(i),parentFrame,now,ros::Duration(0.5)))
            {
                objectTF detectedObject;
                detectedObject.name = tmp.at(objectNameFormatIndex-1);
                detectedObject.frame_id = listOfTF.at(i);
                newestListOfTF.push_back(detectedObject.frame_id);
                tf::StampedTransform transform;
                listener.lookupTransform(parentFrame,listOfTF.at(i),ros::Time(0),transform);
                tf::poseTFToMsg(transform,detectedObject.pose);
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
    for (unsigned int i = 0; i < segmentedObjects->size(); i++) {
        moveit_msgs::CollisionObject co;
        co.id = segmentedObjects->at(i).id;
        co.header.frame_id = segmentedObjects->at(i).header.frame_id;
        co.operation = co.REMOVE;
        objectsToRemove.push_back(co);
    }
    
//    std::cerr << "Object to remove: " << objectsToRemove.size() << std::endl;
    return objectsToRemove;
}

const std::vector<moveit_msgs::CollisionObject> collision_environment::getCollisionObjects() const
{
//    std::cerr << "Number of object: " << segmentedObjects->size() <<std::endl;
    return *segmentedObjects;
}

std::vector<std::string> collision_environment::getListOfTF() const
{
    return listOfTF;
}

bool collision_environment::getTable()
{
    if (!classReady)
    {
        std::cerr << "ERROR, class has no nodehandle to subscribe.\n";
        return false;
    }
    
    moveit_msgs::CollisionObject co;
    std::string parentTableTF;
    
    if (!defineParent) {
        listener.getParent(tableTFname,ros::Time(0),parentTableTF);
    }
    else parentTableTF = definedParent;

    if (!listener.waitForTransform(tableTFname,parentTableTF,ros::Time::now(),ros::Duration(0.5)))
    {
        std::cerr << "Fail to get tf for table: " << tableTFname << std::endl;
        hasTableTF = false;
    }
    else
    {
        co.id = tableTFname;
        co.header.frame_id = parentTableTF;
        if (debug)
            std::cerr << "parentFrame of: " << tableTFname << "-> " << parentTableTF << std::endl;
        tf::StampedTransform transform;
        listener.lookupTransform(parentTableTF,tableTFname,ros::Time(0),transform);
        geometry_msgs::TransformStamped msg;
        transformStampedTFToMsg(transform, msg);
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.75;
        primitive.dimensions[1] = 0.75;
        primitive.dimensions[2] = 0.02;
        
        geometry_msgs::Pose box_pose;
        tf::poseTFToMsg(transform,box_pose);
        box_pose.position.z -= primitive.dimensions[2];
        co.primitives.push_back(primitive);
        co.primitive_poses.push_back(box_pose);
        hasTableTF = true;
    }
    tableObject = co;
    return hasTableTF;
}
