#include "object_data_property.h"

void Obb::generateObb(std::vector<btVector3> corner_points)
{
	pcl::PointCloud<pcl::PointXYZ> cloud = convertVectorBtVec3ToPcl(corner_points);
	pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
	feature_extractor.setInputCloud(cloud.makeShared());
	feature_extractor.compute();

	pcl::PointXYZ min_point_OBB;
	pcl::PointXYZ max_point_OBB;
	pcl::PointXYZ position_OBB;
	Eigen::Matrix3f rotational_matrix_OBB;
	feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
	Eigen::Quaternionf quaternion_obb(rotational_matrix_OBB);

	btQuaternion rot = convertEigenToBulletQuaternion(quaternion_obb);
	btVector3 pos = pclPointToBulletVector(position_OBB);

	this->min_point_ = pclPointToBulletVector(min_point_OBB);
	this->max_point_ = pclPointToBulletVector(max_point_OBB);
	this->pose_ = btTransform(rot,pos);
}

Obb::Obb(std::vector<btVector3> corner_points)
{
	this->generateObb(corner_points);
}

void Object::setPhysicalProperties(const objectShapePtr mesh, const PhysicalProperties &physical_properties)
{
	this->mesh_ = mesh;
	this->physical_properties_ = physical_properties;
	this->mesh_->calculateLocalInertia(this->physical_properties_.mass_, this->physical_properties_.inertia_);

	this->physical_data_ready_ = true;

	this->computeObb();
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
	
	this->copyObbProperty(other);
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

	this->copyObbProperty(other);
}

btVector3 Object::getInertiaVector() const
{
	return this->physical_properties_.inertia_;
}

objectShapePtr Object::getCollisionShape() const
{
	return this->mesh_;
}

void Object::getObbProperty(Obb &entire_shape_obb, std::vector<Obb> &child_shape_obb) const
{
	entire_shape_obb = this->entire_shape_obb_;
	child_shape_obb = this->child_shape_obb_;
}

void Object::copyObbProperty(const Object& other)
{	
	Obb other_entire_shape_obb;
	std::vector<Obb> other_child_shape_obb;
	other.getObbProperty(other_entire_shape_obb,other_child_shape_obb);
	this->entire_shape_obb_ = other_entire_shape_obb;
	this->child_shape_obb_ = other_child_shape_obb;
}

bool Object::computeObb()
{
	if (!mesh_)
	{
		std::cerr << "Object mesh is still empty! Fail to compute Obb\n";
		return false;
	}

    if (!mesh_->isCompound())
    {
    	std::cerr << "Object mesh is not a compound shape! Fail to compute Obb\n";
    	return false;
    }

    const btCompoundShape* compound_shape = (btCompoundShape*)mesh_;

	this->child_shape_obb_.reserve(compound_shape->getNumChildShapes());

	std::vector<btVector3> accumulated_hull_vertices;
	// assumed max 20 points per convex hull for reserving space
	accumulated_hull_vertices.reserve(compound_shape->getNumChildShapes()*20);

	for (int i = 0; i < compound_shape->getNumChildShapes(); ++i)
	{
		const btCollisionShape *child = compound_shape->getChildShape(i);
		if (child->isConvex())
		{
			const btConvexHullShape* child_hull = (btConvexHullShape*)child;
			std::vector<btVector3> child_hull_vertices;
			child_hull_vertices.reserve(child_hull->getNumPoints());
			const btVector3* points = child_hull->getUnscaledPoints();
			for (int j = 0; j < child_hull->getNumPoints(); ++j)
			{
				child_hull_vertices.push_back(points[j]);
			}

			Obb child_obb(child_hull_vertices);
			child_shape_obb_.push_back(child_obb);

			accumulated_hull_vertices.insert(accumulated_hull_vertices.end(),
				child_hull_vertices.begin(),child_hull_vertices.end());
		}
		else
		{
			// only supports convex hull object for now
			continue;
		}
	}

	this->entire_shape_obb_ = Obb(accumulated_hull_vertices);
	return true;
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

void ObjectDatabase::setObjectFolderLocation(const std::string &file_location)
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

Object ObjectDatabase::getObjectProperty(const std::string &object_name) const
{
	return getContentOfConstantMap(object_name,this->database_);
}

bool ObjectDatabase::objectExistInDatabase(const std::string &object_name) const
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
