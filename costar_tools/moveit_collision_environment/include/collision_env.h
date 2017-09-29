#ifndef COLLISION_ENV_H
#define COLLISION_ENV_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>
#include <moveit_msgs/CollisionObject.h>

// use detected object list instead of tf name convention
#include <costar_objrec_msgs/DetectedObject.h>
#include <costar_objrec_msgs/DetectedObjectList.h>

struct objectTF {
    std::string name, frame_id;
    geometry_msgs::Pose pose;
};

class collision_environment
{
	protected:
		ros::NodeHandle nh;
		tf::TransformListener listener;
	;

	private:
		bool debug;
		std::vector<std::string> listOfTF;
		std::vector<objectTF> detectedObjectsTF;
        std::vector<moveit_msgs::CollisionObject>* segmentedObjects;
        moveit_msgs::CollisionObject tableObject;
        std::vector<moveit_msgs::CollisionObject> retained_obstacle_objects;
        
        std::string tableTFname, parentTableTF, baseLinkName;
		std::string mesh_source, file_extension;
        std::string charToFind,objTFsignature;
        std::string parentFrame; //parent of the TF frame that objects attached to
		int objectNameFormatIndex;
        bool classReady,hasParent, hasTableTF,hasObjects;
        bool insertTableWalls, insertBaseLinkGround, useBaseLinkWall;
        bool defineParent;
        std::string definedParent;
        double tableSize, baseLinkWallDistance;

        bool getTable();
        void addSurroundingWalls(
        	moveit_msgs::CollisionObject &targetCollisionObject, const tf::Transform &centerOfObject,
        	const double &distanceToWalls,const double &wallHeights, bool flippedBackWall = false);
		// after breaking the list of TF to several items, which item define the object name
		// Example: with format Obj::NameOfObject::index, objectNameFormatIndex = 2. 
		// If format Obj::index::NameOfObject = 3, objectNameFormatIndex = 3.
	;

	public:
		collision_environment();
        collision_environment(const ros::NodeHandle &nh);
        void setNodeHandle(const ros::NodeHandle &nh);
		void getAllObjectTF();
        void getAllObjectTFfromDetectedObjectMsgs(const costar_objrec_msgs::DetectedObjectList &detectedObject);

        bool addObjectAsRetainedObstacle(const std::string &frame_id);
        void removeAllRetainedObstacles();
        
        moveit_msgs::CollisionObject generateCollisionObjectFromObjectTF(const objectTF &object_info);
        
		void updateCollisionObjects(const bool &updateFrame = true);
        std::vector<std::string> getListOfTF() const;
        const std::vector<moveit_msgs::CollisionObject> getCollisionObjects() const;
        const std::vector<moveit_msgs::CollisionObject> generateOldObjectToRemove() const;
};

std::vector<std::string> stringVectorSeparator (const std::string &input,
                                                const std::string &charToFind, const std::string &breakChar, const bool &debug);

#endif