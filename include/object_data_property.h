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

typedef btCollisionShape* objectShapePtr;

class Object
{
public:
	Object() : physical_data_ready_(false) {};
	void setPhysicalProperties(const objectShapePtr mesh, const double &mass);
	void copyPhysicalProperties(objectShapePtr mesh_output, btScalar &mass_output, btVector3 &inertia_output) const;
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
	btScalar mass_;

	// object physical parameters in SI units (automatically generated)
	btVector3 inertia_;

};

// Object class with id information.
// Useful for keeping track on multiple object of same type
class ObjectWithID : public Object{
public:
	void assignPhysicalPropertyFromObject(const Object &input);
	void assignData(std::string object_id, btTransform  transform);
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

	// set the folder location that contains the object .bcs file
	void setObjectDatabaseLocation(std::string file_location);

	// add one object to the database. The object file must exist in file_location/object_name.bcs. If true, adding object is successful.
	bool addObjectToDatabase(std::string object_name, double mass = 0.2);

	void setDebugMode(bool debug);

	// add multiple objects to the database. Returns number of unsuccesful adding object operation
	std::size_t loadDatabase(std::vector<std::string> object_names);
	std::size_t loadDatabase(std::vector<std::string> object_names, std::vector<double> mass);
	Object getObjectProperty(std::string object_name) const;
	bool objectExistInDatabase(std::string) const;
private:
	bool debug_messages_;
	std::map<std::string, Object> database_;
	std::string file_location_;
};

#endif
