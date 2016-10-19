#include "object_data_property.h"

void Object::setPhysicalProperties(const objectShapePtr mesh, const double &mass)
{
	this->mesh_ = mesh;
	this->mass_ = mass;
	this->mesh_->calculateLocalInertia(this->mass_, this->inertia_);
	this->physical_data_ready_ = true;
}

void Object::copyPhysicalProperties(objectShapePtr mesh_output, btScalar &mass_output, btVector3 &inertia_output) const
{
	mesh_output = this->mesh_;
	mass_output = this->mass_;
	inertia_output = this->inertia_; 
}

btRigidBody* Object::generateRigidBody(const btTransform &transform) const
{
	btDefaultMotionState* object_motion_state = 
		new btDefaultMotionState(transform);
	btRigidBody::btRigidBodyConstructionInfo object_RigidBodyCI(this->mass_, 
		object_motion_state, this->mesh_, this->inertia_);
	btRigidBody* object_RigidBody = new btRigidBody(object_RigidBodyCI);
	return object_RigidBody;
}

void ObjectWithID::assignPhysicalPropertyFromObject(const Object &input)
{
	objectShapePtr mesh_tmp;
	btScalar mass_tmp;
	btVector3 inertia_tmp;
	input.copyPhysicalProperties(mesh_tmp,mass_tmp,inertia_tmp);
	this->mesh_ = mesh_tmp;
	this->mass_ = mass_tmp;
	this->inertia_ = inertia_tmp;
	this->physical_data_ready_ = true;
}

void ObjectWithID::assignData(std::string object_id, btTransform transform)
{
	this->id_ = object_id;
	this->transform_ = transform;
}

std::string ObjectWithID::getID() const
{
	return this->id_;
}

btRigidBody* ObjectWithID::generateRigidBodyForWorld() const
{
	return this->generateRigidBody(this->transform_);
}

void ObjectDatabase::setObjectDatabaseLocation(std::string file_location)
{
	this->file_location_ = file_location;
}

bool ObjectDatabase::addObjectToDatabase(std::string object_name, double mass)
{
	std::string object_file_location = this->file_location_ + "/" + object_name + ".bcs";
	btCollisionShape* simplified_mesh = load_bcs(object_file_location.c_str(), false);
	if (simplified_mesh != NULL)
	{
		Object new_object;
		new_object.setPhysicalProperties(simplified_mesh, mass);
		this->database_[object_name] = new_object;
		return true;
	}
	else
	{
		return false;
	}
}

std::size_t ObjectDatabase::loadDatabase(std::vector<std::string> object_names)
{
	std::size_t unsuccessful_object_count = 0;
	for (std::size_t i = 0; i < object_names.size(); i++)
		if (!this->addObjectToDatabase(object_names[i])) unsuccessful_object_count++;
	return unsuccessful_object_count;
}

std::size_t ObjectDatabase::loadDatabase(std::vector<std::string> object_names, std::vector<double> mass)
{
	std::size_t unsuccessful_object_count = 0;
	for (std::size_t i = 0; i < object_names.size(); i++)
		if (!this->addObjectToDatabase(object_names[i])) unsuccessful_object_count++;
	return unsuccessful_object_count;
}

Object ObjectDatabase::getObjectProperty(std::string object_name) const
{
	std::map<std::string, Object>::const_iterator it(this->database_.find(object_name));
	return it != this->database_.end() ? it->second : Object();
}

bool ObjectDatabase::objectExistInDatabase(std::string object_name) const
{
	return this->database_.find(object_name) != this->database_.end();
}
