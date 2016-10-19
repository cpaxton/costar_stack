#ifndef SCENE_PHYSICS_ENGINE_H
#define SCENE_PHYSICS_ENGINE_H

#include <vector>
#include <map>

// Bullet physics engine
#include <Bullet3Common/b3FileUtils.h>

#include "object_data_property.h"

class PhysicsEngine
{
public:
	PhysicsEngine();
	~PhysicsEngine();

	// use plane as background (table)
	void addBackgroundPlane(btVector3 plane_normal, btScalar plane_constant);
	// TODO: use mesh as background (table)
	void addBackgroundMesh();
	void setGravityVectorDirection(const btVector3 &gravity);
	void addObjects(const std::vector<ObjectWithID> &objects);
	std::map<std::string, btTransform>  getUpdatedObjectPose();
	void resetObjects();

private:
	void simulate();

	bool have_background;
	// rigid body data from ObjectWithID input with ID information
	std::map<std::string, btRigidBody*> rigid_body_;
	btRigidBody* background_;

	// physics engine environment parameters
	btBroadphaseInterface* broadphase_;
	btDefaultCollisionConfiguration* collisionConfiguration_;
	btCollisionDispatcher* dispatcher_;
	btSequentialImpulseConstraintSolver* solver_;
	btDiscreteDynamicsWorld* dynamicsWorld_;

};

#endif
