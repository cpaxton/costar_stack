#ifndef SCENE_PHYSICS_ENGINE_H
#define SCENE_PHYSICS_ENGINE_H

#include <vector>
#include <map>

// Bullet physics engine
// #include <Bullet3Common/b3FileUtils.h>

#include "object_data_property.h"

class PhysicsEngine
{
public:
	PhysicsEngine();
	~PhysicsEngine();

	// use plane as background (table)
	void addBackgroundPlane(const std::vector<btVector3> &plane_points);
	// TODO: use mesh as background (table)
	void addBackgroundMesh();

	// uses a frame that has Y direction up as a guide for gravity direction
	void setGravityVectorDirectionFromTfYUp(const btTransform &transform_y_is_inverse_gravity_direction);
	void setGravityVectorDirection(const btVector3 &gravity);
	void addObjects(const std::vector<ObjectWithID> &objects);
	std::map<std::string, btTransform>  getUpdatedObjectPose();
	void resetObjects();

	void setDebugMode(bool debug);
	
private:
	void simulate();

	bool debug_messages_;
	bool have_background_;
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
