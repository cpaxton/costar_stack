#include <iostream>

#include "scene_physics_engine.h"

PhysicsEngine::PhysicsEngine()
{
	this->broadphase_ = new btDbvtBroadphase();
	this->collisionConfiguration_ = new btDefaultCollisionConfiguration();
	this->dispatcher_ = new btCollisionDispatcher(this->collisionConfiguration_);
	this->solver_  = new btSequentialImpulseConstraintSolver;
	this->dynamicsWorld_ = new btDiscreteDynamicsWorld(this->dispatcher_, 
		this->broadphase_, this->solver_, this->collisionConfiguration_);
	this->have_background = false;
}

PhysicsEngine::~PhysicsEngine()
{
	// Clean up pointers

	// delete all object contents
	this->resetObjects();
	
	// removes background
	this->dynamicsWorld_->removeRigidBody(this->background_);
	delete this->background_->getMotionState();
	delete this->background_;

	// delete physics world environment
	delete this->dynamicsWorld_;
	delete this->solver_;
	delete this->dispatcher_;
	delete this->collisionConfiguration_;
	delete this->broadphase_;
}

void PhysicsEngine::addBackgroundPlane(btVector3 plane_normal, btScalar plane_constant)
{
	btCollisionShape*  background = new btStaticPlaneShape(plane_normal, plane_constant);
	btDefaultMotionState* background_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    
    // unmovable ground object
    btRigidBody::btRigidBodyConstructionInfo
            background_RigidBodyCI(0, background_motion_state, background, btVector3(0, 0, 0));
    
    this->background_ = new btRigidBody(background_RigidBodyCI);
    this->dynamicsWorld_->addRigidBody(this->background_);
    this->have_background = true;
}

void PhysicsEngine::addBackgroundMesh()
{
	// TODO
	this->have_background = true;
}

void PhysicsEngine::setGravityVectorDirection(const btVector3 &gravity)
{
	this->dynamicsWorld_->setGravity(gravity);
}

void PhysicsEngine::addObjects(const std::vector<ObjectWithID> &objects)
{
	// Add new objects

	for (std::vector<ObjectWithID>::const_iterator it = objects.begin(); 
		it != objects.end(); ++it)
	{
		this->rigid_body_[it->getID()] = it->generateRigidBodyForWorld();
		this->dynamicsWorld_->addRigidBody(this->rigid_body_[it->getID()]);
	}
}

void PhysicsEngine::simulate()
{
	// If do not have background, do not update the scene
	if (!this->have_background)
		return;

	// TODO: early termination. Check if all object has no changes anymore.
	bool steady_state = false;

	for (int i = 0; i < 300; i++)
	{
		this->dynamicsWorld_->stepSimulation(1 / 60.f, 10);
		if (steady_state) break;
	}
}

std::map<std::string, btTransform>  PhysicsEngine::getUpdatedObjectPose()
{
	// Run simulation to get an update on object poses
	this->simulate();

	std::map<std::string, btTransform> result_pose;
	for (std::map<std::string, btRigidBody*>::const_iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		it->second->getMotionState()->getWorldTransform(result_pose[it->first]);
	}

	return result_pose;
}

void PhysicsEngine::resetObjects()
{
	// Removes all objects from the physics world then delete its' content
	for (std::map<std::string, btRigidBody*>::iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		this->dynamicsWorld_->removeRigidBody(it->second);
		delete it->second->getMotionState();
		delete it->second;
	}

	this->rigid_body_.clear();
}
