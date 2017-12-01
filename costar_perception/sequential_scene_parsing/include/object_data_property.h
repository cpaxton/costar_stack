#ifndef OBJECT_DATA_PROPERTY_H
#define OBJECT_DATA_PROPERTY_H

#include <vector>
#include <map>

// Contains the datatype used for this library
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <Eigen/Geometry>
#include <btBulletDynamicsCommon.h>

// Contains function for loading an object from file
#include "bcs_loader.h"
#include "physics_world_parameters.h"
// Contains variety of tools for handling conversion and getting content from one datatype to another
#include "utility.h"
#include "scene_physics_penalty.h"

typedef btCollisionShape* objectShapePtr;
typedef btCollisionShape objectShape;

enum background_mode {BACKGROUND_PLANE, BACKGROUND_HULL, BACKGROUND_MESH};

struct Obb
{
	// oriented bounding box
	btTransform pose_;
	btVector3 min_point_;
	btVector3 max_point_;

	Obb() {}
	Obb(std::vector<btVector3> corner_points);
	void generateObb(std::vector<btVector3> corner_points);
};

struct PhysicalProperties
{
	btScalar mass_;
	btScalar friction_;
	btScalar rolling_friction_;
	btVector3 inertia_;

	PhysicalProperties() : mass_(1.0), friction_(1.0), rolling_friction_(1.0) {};

	// template constructor
	template <typename NumericStandard>
	PhysicalProperties(const NumericStandard &mass, const NumericStandard &friction, const NumericStandard &rolling_friction): 
		mass_(mass), friction_(friction), rolling_friction_(rolling_friction) {};

	// copy assignment
	PhysicalProperties& operator=(const PhysicalProperties& other)
	{
		this->mass_ = other.mass_;
		this->friction_ = other.friction_;
		this->rolling_friction_ = other.rolling_friction_;
		this->inertia_ = other.inertia_;
		return *this;
	}

	void extractPhysicalProperty(btScalar &mass_out, btScalar &friction_out, btScalar &rolling_friction_out, btVector3 &inertia_out)
	{
		mass_out = this->mass_;
		friction_out = this->friction_;
		rolling_friction_out = this->rolling_friction_;
		inertia_out = this->inertia_;
	}
};

class Object
{
public:
	// NOTE: Object class and its' inheritance Has a potential memory leak since it allocates mesh using standard pointer.
	// Since every mesh pointed by this Object is copied to ObjectDatabase class, it does not leak.

	Object() : physical_data_ready_(false) {};
	// ~Object();
	void setPhysicalProperties(const objectShapePtr mesh, const PhysicalProperties &physical_properties);
	void shallowCopyPhysicalProperties(objectShapePtr &mesh_out, PhysicalProperties &physical_properties_out) const;
	
	void shallowCopy(const Object& other);
	void deepCopy(const Object& other);
	// free memory that pointed by objectSHapePtr pointer
	void deleteMeshContent();

	// generate rigid body data that can be added to the bullet physics world
	btRigidBody* generateRigidBody(const btTransform &transform) const;
	btVector3 getInertiaVector() const;
	objectShapePtr getCollisionShape() const;

	void getObbProperty(Obb &entire_shape_obb, std::vector<Obb> &child_shape_obb) const;
	void copyObbProperty(const Object& other);
	// void addSpecialProperty();
;
protected:
	bool computeObb();
	bool physical_data_ready_;
	objectShapePtr mesh_;

	// object physical parameters in SI units
	PhysicalProperties physical_properties_;

	Obb entire_shape_obb_;
	std::vector<Obb> child_shape_obb_;
};

// Object class with id information.
// Useful for keeping track on multiple object of same type
class ObjectWithID : public Object{
public:
	void assignPhysicalPropertyFromObject(const Object &input);
	void assignData(const std::string &object_id, const btTransform &transform, const std::string &object_class);
	std::string getID() const;
	std::string getObjectClass() const;
	btRigidBody* generateRigidBodyForWorld() const;
	btTransform getTransform() const;
private:
	std::string id_;
	std::string object_class_;
	btTransform  transform_;
};

// class ObjectWithPCloud : public Object{
// public:
// 	void addPointCloud();
// 	void getMeshPointCloud();
// ;
// private:
// 	void generateMeshPointCloud();

// 	bool has_generated_pcl_mesh;
// };

class ObjectDatabase
{
public:
	ObjectDatabase() : debug_messages_(false) {};
	// delete all object mesh data
	~ObjectDatabase();

	// set the folder location that contains the object .bcs file
	void setObjectFolderLocation(const std::string &file_location);
	void setPhysicalPropertyDatabase(const std::map<std::string, PhysicalProperties> &physical_properties_database);

	// add one object to the database. The object file must exist in file_location/object_name.bcs. If true, adding object is successful.
	bool addObjectToDatabase(const std::string &object_name);

	void setDebugMode(bool debug);

	// add multiple objects to the database. Returns number of unsuccesful adding object operation
	std::size_t loadDatabase(const std::map<std::string, PhysicalProperties> &physical_properties_database);
	Object getObjectProperty(const std::string &object_name) const;
	bool objectExistInDatabase(const std::string &object_name) const;
	std::map<std::string, ObjectPenaltyParameters> * getObjectPenaltyDatabase();

private:
	bool debug_messages_;
	std::map<std::string, Object> database_;
	std::map<std::string, PhysicalProperties> physical_properties_database_;
	std::map<std::string, ObjectPenaltyParameters> object_penalty_parameter_database_;
	std::string file_location_;
};

struct ObjectPairProperty
{
	btTransform relative_transform_;
	btVector3 linear_limit_;
	btVector3 angular_limit_;

	ObjectPairProperty() : relative_transform_(btTransform::getIdentity() ) {};
	ObjectPairProperty(btTransform relative_transform) : relative_transform_(relative_transform) {};
	ObjectPairProperty(btTransform relative_transform, btVector3  linear_limit, btVector3 angular_limit) : 
		relative_transform_(relative_transform), linear_limit_(linear_limit), angular_limit_(angular_limit) {};
	
	ObjectPairProperty getInverse() const;
};

class ObjectPairDatabase
{
	std::map< std::pair<std::string,std::string>, btTransform > pairRelativeTransform;
};

#endif
