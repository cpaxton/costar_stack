#ifndef OBJECT_DATA_PROPERTY_H
#define OBJECT_DATA_PROPERTY_H

#include <vector>
#include <map>

// Contains the datatype used for this library
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <btBulletDynamicsCommon.h>

// Contains function for loading an object from file
#include "bcs_loader.h"

// Contains variety of tools for handling conversion and getting content from one datatype to another
#include "utility.h"

typedef btCollisionShape* objectShapePtr;
typedef btCollisionShape objectShape;

struct PhysicalProperties
{
	btScalar mass_;
	btScalar friction_;
	btScalar rolling_friction_;

	PhysicalProperties() : mass_(1.0), friction_(1.0), rolling_friction_(1.0) {};

	// template constructor
	template <typename numericStandard>
	PhysicalProperties(const numericStandard &mass, const numericStandard &friction, const numericStandard &rolling_friction): 
		mass_(mass), friction_(friction), rolling_friction_(rolling_friction) {};

	// copy assignment
	PhysicalProperties& operator=(const PhysicalProperties& other)
	{
		this->mass_ = other.mass_;
		this->friction_ = other.friction_;
		this->rolling_friction_ = other.rolling_friction_;
		return *this;
	}

	void extractPhysicalProperty(btScalar &mass_out, btScalar &friction_out, btScalar &rolling_friction_out)
	{
		mass_out = this->mass_;
		friction_out = this->friction_;
		rolling_friction_out = this->rolling_friction_;
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
	// void addSpecialProperty();
;
protected:
	// list of supporting force that could be generated from the object and its effective range
	// std::vector<std::pair<std::string, double> > support_property_;
	// std::string object_fit_match_;

	bool physical_data_ready_;

	// object physical parameters in SI units (manually set)
	objectShapePtr mesh_;
	PhysicalProperties physical_properties_;

	// btScalar mass_;
	// btScalar friction_;
	// btScalar rolling_friction_;
	
	// object physical parameters in SI units (automatically generated)
	btVector3 inertia_;

};

// Object class with id information.
// Useful for keeping track on multiple object of same type
class ObjectWithID : public Object{
public:
	void assignPhysicalPropertyFromObject(const Object &input);
	void assignData(const std::string &object_id, const btTransform &transform);
	std::string getID() const;
	btRigidBody* generateRigidBodyForWorld() const;
private:
	std::string id_;
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

struct ObjectDatabase
{
public:
	ObjectDatabase() : debug_messages_(false) {};
	// delete all object mesh data
	~ObjectDatabase();

	// set the folder location that contains the object .bcs file
	void setObjectFolderLocation(std::string file_location);
	void setPhysicalPropertyDatabase(const std::map<std::string, PhysicalProperties> &physical_properties_database);

	// add one object to the database. The object file must exist in file_location/object_name.bcs. If true, adding object is successful.
	bool addObjectToDatabase(const std::string &object_name);

	void setDebugMode(bool debug);

	// add multiple objects to the database. Returns number of unsuccesful adding object operation
	std::size_t loadDatabase(const std::map<std::string, PhysicalProperties> &physical_properties_database);
	Object getObjectProperty(std::string object_name) const;
	bool objectExistInDatabase(std::string) const;

private:
	bool debug_messages_;
	std::map<std::string, Object> database_;
	std::map<std::string, PhysicalProperties> physical_properties_database_;
	std::string file_location_;
};

#endif
