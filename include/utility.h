#ifndef UTILITY_H
#define UTILITY_H

#include <Eigen/Geometry>
#include <btBulletDynamicsCommon.h>
#include <map>

template <class container_type>
container_type getContentOfConstantMap(const std::string &key, const std::map<std::string, container_type> &database)
{
	typename std::map<std::string, container_type>::const_iterator it(database.find(key));
	return it != database.end() ? it->second : container_type();
}

template <typename container_type>
bool keyExistInConstantMap(const std::string &key, const std::map<std::string, container_type> &database)
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

#endif