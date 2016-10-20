#include <iostream>

#include "scene_physics_engine.h"

PhysicsEngine::PhysicsEngine() : have_background_(false), debug_messages_(false)
{
	if (this->debug_messages_) std::cerr << "Setting up physics engine.\n";
	this->broadphase_ = new btDbvtBroadphase();
	this->collisionConfiguration_ = new btDefaultCollisionConfiguration();
	this->dispatcher_ = new btCollisionDispatcher(this->collisionConfiguration_);
	this->solver_  = new btSequentialImpulseConstraintSolver;
	this->dynamicsWorld_ = new btDiscreteDynamicsWorld(this->dispatcher_, 
		this->broadphase_, this->solver_, this->collisionConfiguration_);
}

PhysicsEngine::~PhysicsEngine()
{
	// Clean up pointers
	if (this->debug_messages_) std::cerr << "Shutting down physics engine.\n";
	// delete all object contents
	this->resetObjects();
	
	// removes background
	if (this->have_background_)
	{
		this->dynamicsWorld_->removeRigidBody(this->background_);
		delete this->background_->getMotionState();
		delete this->background_;
	}

	// delete physics world environment
	if (this->debug_messages_) std::cerr << "Deleting physics engine environment.\n";
	delete this->dynamicsWorld_;
	delete this->solver_;
	delete this->dispatcher_;
	delete this->collisionConfiguration_;
	delete this->broadphase_;
}

void PhysicsEngine::addBackgroundPlane(btVector3 plane_normal, btScalar plane_constant)
{
	if (this->debug_messages_) std::cerr << "Adding background(plane) to the physics engine's world.\n";
	btCollisionShape*  background = new btStaticPlaneShape(plane_normal, plane_constant);
	btDefaultMotionState* background_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
    
    // unmovable ground object
    btRigidBody::btRigidBodyConstructionInfo
            background_RigidBodyCI(0, background_motion_state, background, btVector3(0, 0, 0));
    
    this->background_ = new btRigidBody(background_RigidBodyCI);
    this->dynamicsWorld_->addRigidBody(this->background_);
    this->have_background_ = true;
}

void PhysicsEngine::addBackgroundMesh()
{
	if (this->debug_messages_) std::cerr << "Adding background(mesh).\n";
	// TODO
	this->have_background_ = true;
}

void PhysicsEngine::setGravityVectorDirectionFromTfYUp(const btTransform &transform_y_is_inverse_gravity_direction)
{
	if (this->debug_messages_) std::cerr << "Setting physics engine gravity based on a TF transform.\n";

	btMatrix3x3 rotation_matrix = transform_y_is_inverse_gravity_direction.getBasis();
	if (this->debug_messages_)
	{
		std::cerr << "Input transform:\n";
		for (int i = 0; i < 3; i++){
			for (int j = 0; j < 3; j++)
			{
				std::cerr <<  rotation_matrix[i][j] << " ";
			}
			std::cerr << std::endl;
		}

	} 

	// get Z axis direction from the transform
	btVector3 inverse_gravity(rotation_matrix[0][2],rotation_matrix[1][2],rotation_matrix[2][2]);
	
	this->setGravityVectorDirection(-inverse_gravity);
}

void PhysicsEngine::setGravityVectorDirection(const btVector3 &gravity)
{
	if (this->debug_messages_) std::cerr << "Setting physics engine gravity vector.\n";
	btVector3 gravity_corrected_magnitude = gravity / gravity.norm() * 9.807;
	if (this->debug_messages_) std::cerr << "Gravity vector::" << gravity_corrected_magnitude[0] << ", "
		<< gravity_corrected_magnitude[1] << ", " 
		<< gravity_corrected_magnitude[2] << std::endl;
	this->dynamicsWorld_->setGravity(gravity_corrected_magnitude);
}

void PhysicsEngine::addObjects(const std::vector<ObjectWithID> &objects)
{
	// Add new objects
	if (this->debug_messages_) std::cerr << "Adding scene objects to the physics engine.\n";
	
	for (std::vector<ObjectWithID>::const_iterator it = objects.begin(); 
		it != objects.end(); ++it)
	{
		if (this->debug_messages_) std::cerr << "Adding rigid body " << it->getID() << " to the physics engine's database.\n";
		this->rigid_body_[it->getID()] = it->generateRigidBodyForWorld();
		if (this->debug_messages_) std::cerr << "Adding rigid body " << it->getID() << " to the physics engine's world.\n";
		this->dynamicsWorld_->addRigidBody(this->rigid_body_[it->getID()]);
	}
	if (this->debug_messages_) std::cerr << "Scene objects added to the physics engine.\n";
}

void PhysicsEngine::simulate()
{
	if (this->debug_messages_) std::cerr << "Simulating the physics engine.\n";
	// If do not have background, do not update the scene
	
	if (!this->have_background_)
	{
		std::cerr << "Skipping simulation, scene does not has any background data yet.\n";
		return;
	}

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
	if (this->debug_messages_) std::cerr << "Getting updated scene objects poses.\n";
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
	if (this->debug_messages_) std::cerr << "Removing all scene objects.\n";
	// Removes all objects from the physics world then delete its' content
	for (std::map<std::string, btRigidBody*>::iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		this->dynamicsWorld_->removeRigidBody(it->second);
		delete it->second->getMotionState();
		delete it->second;
		if (this->debug_messages_) std::cerr << "Removed objects: "<<  it->first <<".\n";
	}
	this->rigid_body_.clear();
	if (this->debug_messages_) std::cerr << "Done removing all scene objects.\n";
}

void PhysicsEngine::setDebugMode(bool debug)
{
	this->debug_messages_ = debug;
}