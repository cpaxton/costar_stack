#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Geometry>
#include <btBulletDynamicsCommon.h>
#include <pcl/point_types.h>
#include <map>
#include <vector>
#include <set>
#include <sstream>
#include "physics_world_parameters.h"

template <typename container_type1, typename container_type2>
container_type2 getContentOfConstantMap(const container_type1 &key, const std::map<container_type1, container_type2> &database)
{
	typename std::map<container_type1, container_type2>::const_iterator it(database.find(key));
	return it != database.end() ? it->second : container_type2();
}

template <typename container_type1, typename container_type2>
bool keyExistInConstantMap(const container_type1 &key, const std::map<container_type1, container_type2> &database)
{
	return database.find(key) != database.end();
}


template <typename numericStandard>
btVector3 convertEigenToBulletVector(const Eigen::Matrix<numericStandard, 3, 1> &eigentype_vector)
{
	return btVector3(btScalar(eigentype_vector[0]),
		btScalar(eigentype_vector[1]),
		btScalar(eigentype_vector[2]) );
}

template <typename numericStandard>
btQuaternion convertEigenToBulletQuaternion(const Eigen::Quaternion<numericStandard> &eigentype_q)
{
	return btQuaternion(btScalar(eigentype_q.x()),
		btScalar(eigentype_q.y()),
		btScalar(eigentype_q.z()),
		btScalar(eigentype_q.w()) ) ;
}

template <typename numericStandard>
Eigen::Quaternion<numericStandard>convertBulletToEigenQuaternion(const btQuaternion &bt_q)
{
	return Eigen::Quaternion<numericStandard>(
		numericStandard(bt_q.w() ),
		numericStandard(bt_q.x() ),
		numericStandard(bt_q.y() ),
		numericStandard(bt_q.z() ) );
}

template <typename numericStandard>
Eigen::Matrix<numericStandard, 3, 1> convertBulletToEigenVector(const btVector3 &bt_vector)
{
	return Eigen::Matrix<numericStandard, 3, 1> (numericStandard(bt_vector.x()),
		numericStandard(bt_vector.y()),
		numericStandard(bt_vector.z()));
}


template <typename numericStandard>
btTransform convertEigenToBulletTransform(const Eigen::Transform< numericStandard,3,Eigen::Affine > &input_eigentype_tf)
{
	Eigen::Quaternion<numericStandard> rotation(input_eigentype_tf.rotation());
	btVector3 bt_translation = convertEigenToBulletVector(input_eigentype_tf.translation());
	btQuaternion bt_q = convertEigenToBulletQuaternion(rotation);
	return btTransform(bt_q,bt_translation);
}

template <typename numericStandard>
Eigen::Transform< numericStandard,3,Eigen::Affine > convertBulletToEigenTransform(const btTransform &input_bullettype_tf)
{
	Eigen::Quaternion<numericStandard> ei_q = convertBulletToEigenQuaternion<numericStandard>(input_bullettype_tf.getRotation());
	Eigen::Matrix<numericStandard, 3, 1> ei_vec = convertBulletToEigenVector<numericStandard>(input_bullettype_tf.getOrigin());
	Eigen::Translation<numericStandard,3> tmpTranslation (ei_vec);
    Eigen::Transform< numericStandard,3,Eigen::Affine >  result = tmpTranslation * ei_q;
    return result;
}

template <typename PointT>
btVector3 pclPointToBulletVector(const PointT &input)
{
	return btVector3(input.x,input.y,input.z);
}


inline
btTransform scaleTransformToPhysicsEngine(const btTransform &input)
{
	btTransform bt = input;
	bt.setOrigin(bt.getOrigin()*SCALING);
	return bt;
}

inline
btTransform rescaleTransformFromPhysicsEngine(const btTransform &input)
{
	btTransform bt = input;
	bt.setOrigin(bt.getOrigin()/SCALING);
	return bt;
}


static
std::string getObjectIDFromCollisionObject(const btCollisionObject* object)
{
    if (object->getUserPointer() != NULL)
        return *(std::string*)object->getUserPointer();
    else
        return std::string("unrecognized_object");
}

static
std::string printTransform(const btTransform &transform)
{
	std::stringstream ss;
	btVector3 origin = transform.getOrigin();
	btQuaternion q = transform.getRotation();
	ss << "Origin: " << origin.x() << ", " << origin.y() << ", " << origin.z() << "; ";
	ss << "Q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
	return ss.str();
}

static
std::string printVecString(const std::vector<std::string> &list_of_strings)
{
	std::stringstream ss;
	if (list_of_strings.size() == 0 ) ss << "\n";
	for (std::vector<std::string>::const_iterator it = list_of_strings.begin(); it != list_of_strings.end(); ++it)
	{
		std::string separator = (it != --list_of_strings.end()) ? ", " : ".\n";
		ss << *it << separator;
	}
	return ss.str();
}

static
std::string printSetString(const std::set<std::string> &set_of_strings)
{
	std::stringstream ss;
	if (set_of_strings.size() == 0 ) ss << "\n";
	for (std::set<std::string>::const_iterator it = set_of_strings.begin(); it != set_of_strings.end(); ++it)
	{
		std::string separator = (it != --set_of_strings.end()) ? ", " : ".\n";
		ss << *it << separator;
	}
	return ss.str();
}



static
std::ostream &operator<<(std::ostream &os, const std::vector<std::string> &list_of_strings)
{
	if (list_of_strings.size() == 0 ) return os << "\n";
	for (std::vector<std::string>::const_iterator it = list_of_strings.begin(); it != list_of_strings.end(); ++it)
	{
		std::string separator = (it != --list_of_strings.end()) ? ", " : ".\n";
		os << list_of_strings << separator;
	}
	return os;
}


static
std::ostream &operator<<(std::ostream &os, const std::set<std::string> &set_of_strings)
{
	if (set_of_strings.size() == 0 ) return os << "\n";
	for (std::set<std::string>::const_iterator it = set_of_strings.begin(); it != set_of_strings.end(); ++it)
	{
		std::string separator = (it != --set_of_strings.end()) ? ", " : ".\n";
		os << *it << separator;
	}
	return os;
}

#endif