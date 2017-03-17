#include "object_data_property.h"

void Object::setPhysicalProperties(const objectShapePtr mesh, const PhysicalProperties &physical_properties)
{
	this->mesh_ = mesh;
	this->physical_properties_ = physical_properties;
	this->mesh_->calculateLocalInertia(this->physical_properties_.mass_, this->physical_properties_.inertia_);

	this->physical_data_ready_ = true;
}

void Object::shallowCopyPhysicalProperties(objectShapePtr &mesh_out, PhysicalProperties &physical_properties_out) const
{
	// perform shallowcopy of mesh
	mesh_out = this->mesh_;
	physical_properties_out = this->physical_properties_;
}

btRigidBody* Object::generateRigidBody(const btTransform &transform) const
{
	btDefaultMotionState* object_motion_state = 
		new btDefaultMotionState(transform);

	btRigidBody::btRigidBodyConstructionInfo object_RigidBodyCI(this->physical_properties_.mass_, 
		object_motion_state, this->mesh_, this->physical_properties_.inertia_);

	btRigidBody* object_RigidBody = new btRigidBody(object_RigidBodyCI);
	object_RigidBody->setFriction(this->physical_properties_.friction_);
	object_RigidBody->setRollingFriction(this->physical_properties_.rolling_friction_);
	object_RigidBody->setDamping(0.5,0.5);
	return object_RigidBody;
}

void Object::deleteMeshContent()
{
	if (this->mesh_ != NULL)
		delete this->mesh_;
}

// Object::~Object()
// {
// 	this->deleteMeshContent();
// }

void Object::shallowCopy(const Object& other)
{
	objectShapePtr mesh_tmp;
	PhysicalProperties phy_temp;
	other.shallowCopyPhysicalProperties(mesh_tmp,phy_temp);

	// mesh shallow copy
	this->mesh_ = mesh_tmp;
	
	this->physical_properties_ = phy_temp;
	this->physical_data_ready_ = true;
}

void Object::deepCopy(const Object& other)
{
	objectShapePtr mesh_tmp;
	PhysicalProperties phy_temp;
	other.shallowCopyPhysicalProperties(mesh_tmp,phy_temp);

	// mesh deep copy
	this->mesh_ = new btCompoundShape();
	*this->mesh_ = *mesh_tmp;
	
	this->physical_properties_ = phy_temp;
	this->physical_data_ready_ = true;
}

btVector3 Object::getInertiaVector() const
{
	return this->physical_properties_.inertia_;
}

void ObjectWithID::assignPhysicalPropertyFromObject(const Object &input)
{
	this->shallowCopy(input);
}

void ObjectWithID::assignData(const std::string &object_id, const btTransform &transform, const std::string &object_class)
{
	this->id_ = object_id;
	this->transform_ = transform;
	this->object_class_ = object_class;
}

// inline
std::string ObjectWithID::getID() const
{
	return this->id_;
}

std::string ObjectWithID::getObjectClass() const
{
	return this->object_class_;
}

btRigidBody* ObjectWithID::generateRigidBodyForWorld() const
{
	return this->generateRigidBody(this->transform_);
}

btTransform ObjectWithID::getTransform() const
{
	return this->transform_;
}

void ObjectDatabase::setObjectFolderLocation(std::string file_location)
{
	this->file_location_ = file_location;
}

void ObjectDatabase::setPhysicalPropertyDatabase(const std::map<std::string, PhysicalProperties> &physical_properties_database)
{
	this->physical_properties_database_ = physical_properties_database;
}


bool ObjectDatabase::addObjectToDatabase(const std::string &object_name)
{
	if (this->debug_messages_) std::cerr << "Adding object: " << object_name << " to the database.\n";
	std::string object_file_location = this->file_location_ + "/" + object_name + ".bcs";
	btCollisionShape* simplified_mesh = load_bcs(object_file_location.c_str(), false);
	simplified_mesh->setLocalScaling(btVector3(SCALING,SCALING,SCALING));

	if (simplified_mesh != NULL)
	{
		Object new_object;
		PhysicalProperties new_property = getContentOfConstantMap(object_name,this->physical_properties_database_);
		ObjectPenaltyParameters new_penalty_params;

		new_object.setPhysicalProperties(simplified_mesh, new_property);
		new_penalty_params.maximum_angular_acceleration_ = getObjectMaximumAngularAcceleration(*simplified_mesh,new_property.mass_, new_object.getInertiaVector());
		// std::cerr << "max angular acc = " << new_penalty_params.maximum_angular_acceleration_ << std::endl;
		this->database_[object_name].shallowCopy(new_object);
		this->object_penalty_parameter_database_[object_name] = new_penalty_params;
		if (this->debug_messages_) std::cerr << object_name << " successfully added to the database.\n";

		return true;
	}
	else
	{
		if (this->debug_messages_) std::cerr << object_name << " successfully added to the database.\n";
		return false;
	}
}

ObjectDatabase::~ObjectDatabase()
{
	// delete all object mesh stored in the object database
	for (std::map<std::string, Object>::iterator it = this->database_.begin(); 
		it != this->database_.end(); ++it)
	{
		it->second.deleteMeshContent();
	}
	this->database_.clear();
	this->physical_properties_database_.clear();
}

void ObjectDatabase::setDebugMode(bool debug)
{
	this->debug_messages_ = true;
}

std::size_t ObjectDatabase::loadDatabase(const std::map<std::string, PhysicalProperties> &physical_properties_database)
{
	std::size_t unsuccessful_object_count = 0;
	this->setPhysicalPropertyDatabase(physical_properties_database);

	for (std::map<std::string, PhysicalProperties>::iterator it = this->physical_properties_database_.begin(); 
		it != this->physical_properties_database_.end(); ++it)
	{
		if (!this->addObjectToDatabase(it->first)) unsuccessful_object_count++;
	}

	return unsuccessful_object_count;
}

Object ObjectDatabase::getObjectProperty(std::string object_name) const
{
	return getContentOfConstantMap(object_name,this->database_);
}

bool ObjectDatabase::objectExistInDatabase(std::string object_name) const
{
	return keyExistInConstantMap(object_name,this->database_);
}

std::map<std::string, ObjectPenaltyParameters> * ObjectDatabase::getObjectPenaltyDatabase()
{
	return &this->object_penalty_parameter_database_;
}

// ObjectPairProperty ObjectPairProperty::getInverse() const
// {
	
// }
