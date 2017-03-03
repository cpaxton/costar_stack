#include <iostream>
#include <math.h>

#include "scene_physics_engine.h"
#include "scene_physics_penalty.h"
#include "scene_physics_support.h"

static void _worldTickCallback(btDynamicsWorld *world, btScalar timeStep)
{ 
	PhysicsEngine *physics_engine_world = static_cast<PhysicsEngine *>(world->getWorldUserInfo());
	physics_engine_world->worldTickCallback(timeStep); 
}

PhysicsEngine::PhysicsEngine() : have_background_(false), debug_messages_(false), rendering_launched_(false), use_background_normal_as_gravity_(false)
{
	if (this->debug_messages_) std::cerr << "Setting up physics engine.\n";
	simulation_step_ = 1/180.f; // 180 Hz
	this->initPhysics();
}
void PhysicsEngine::addBackgroundPlane(btVector3 plane_normal, btScalar plane_constant, btVector3 plane_center)
{
	if (this->debug_messages_) std::cerr << "Adding background(plane) to the physics engine's world.\n";
	btCollisionShape*  background = new btStaticPlaneShape(plane_normal, plane_constant);
	btDefaultMotionState* background_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    
    // unmovable ground object
    btRigidBody::btRigidBodyConstructionInfo
            background_RigidBodyCI(0, background_motion_state, background, btVector3(0, 0, 0));
    
    this->background_ = new btRigidBody(background_RigidBodyCI);
    m_dynamicsWorld->addRigidBody(this->background_);
	this->background_->setFriction(1.f);
	this->background_->setRollingFriction(1.f);
	this->have_background_ = true;

	this->camera_coordinate_ = btVector3(0,0,0);
	this->target_coordinate_ = plane_center;
	this->setCameraPositionAndTarget(camera_coordinate_,target_coordinate_);
	this->setCameraClippingPlaneNearFar(0.005f);
	this->background_surface_normal_ = plane_normal;
	if (this->use_background_normal_as_gravity_)
		this->setGravityVectorDirection(-background_surface_normal_);
    m_collisionShapes.push_back(background);
}

void PhysicsEngine::addBackgroundConvexHull(const std::vector<btVector3> &plane_points, btVector3 plane_normal)
{
	// mtx_.lock();

	if (this->debug_messages_) std::cerr << "Adding background(convex hull) to the physics engine's world.\n";
	
	btVector3 plane_center(0,0,0);

	btConvexHullShape background_convex;
	for (std::vector<btVector3>::const_iterator it = plane_points.begin(); it != plane_points.end(); ++it)
	{
		// add a point to the convex hull then recalculate the AABB
		background_convex.addPoint(*it, true);
		plane_center+= *it;
	}
	background_convex.setMargin(1); // 1 mm rescaled to 0.01 mm

	plane_center /= plane_points.size();
	// since everything is in reference to camera coordinate, camera coordinate is 0,0,0
	this->camera_coordinate_ = btVector3(0,0,0);
	this->target_coordinate_ = plane_center;
	this->setCameraPositionAndTarget(camera_coordinate_,target_coordinate_);
	this->setCameraClippingPlaneNearFar(0.005f);

	btCollisionShape*  background = new btConvexHullShape(background_convex);
	btDefaultMotionState* background_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
	
	// unmovable ground object
	btRigidBody::btRigidBodyConstructionInfo
			background_RigidBodyCI(0, background_motion_state, background, btVector3(0, 0, 0));
	
	this->background_ = new btRigidBody(background_RigidBodyCI);
	this->background_->setFriction(1.f);
	this->background_->setRollingFriction(1.f);
	
	m_dynamicsWorld->addRigidBody(this->background_);

	if (this->debug_messages_)
	{
		std::cerr << "Background: \n";
		btTransform background_tf;
		this->background_->getMotionState()->getWorldTransform(background_tf);
		btQuaternion q = background_tf.getRotation();
		btVector3 t = background_tf.getOrigin();
		std::cerr << "Quaterion: " << q[0] << ", " << q[1] << ", "  << q[2] << ", "  << q[3] << std::endl;
		std::cerr << "Translation: " << t[0]  << ", " << t[1]  << ", " << t[2] << std::endl;
	}
	this->have_background_ = true;
	this->background_surface_normal_ = plane_normal;
	if (this->use_background_normal_as_gravity_)
		this->setGravityVectorDirection(-background_surface_normal_);
	// mtx_.unlock();
    m_collisionShapes.push_back(background);
}

void PhysicsEngine::addBackgroundMesh(btTriangleMesh* trimesh, btVector3 plane_normal, btVector3 plane_center)
{
	if (this->debug_messages_) std::cerr << "Adding background(mesh).\n";
	this->camera_coordinate_ = btVector3(0,0,0);
	this->target_coordinate_ = plane_center;
	this->setCameraPositionAndTarget(camera_coordinate_,target_coordinate_);
	this->setCameraClippingPlaneNearFar(0.005f);
	this->background_surface_normal_ = plane_normal;
	if (this->use_background_normal_as_gravity_)
		this->setGravityVectorDirection(-background_surface_normal_);

	bool useQuantizedBvhTree = true;

	btCollisionShape* background  = new btBvhTriangleMeshShape(trimesh,useQuantizedBvhTree);
	btDefaultMotionState* background_motion_state = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
	
	// unmovable ground object
	btRigidBody::btRigidBodyConstructionInfo
			background_RigidBodyCI(0, background_motion_state, background, btVector3(0, 0, 0));
	
	this->background_ = new btRigidBody(background_RigidBodyCI);
	this->background_->setFriction(1.f);
	this->background_->setRollingFriction(1.f);
	
	m_dynamicsWorld->addRigidBody(this->background_);
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
	// mtx_.lock();
	if (this->debug_messages_) std::cerr << "Setting physics engine gravity vector.\n";
	this->gravity_vector_ = gravity / gravity.norm() * GRAVITY_MAGNITUDE * SCALING;
	if (this->debug_messages_) std::cerr << "Gravity vector::" << gravity_vector_[0] << ", "
		<< gravity_vector_[1] << ", " 
		<< gravity_vector_[2] << std::endl;
	m_dynamicsWorld->setGravity(gravity_vector_);
	this->gravity_magnitude_ = gravity_vector_.norm();
	// mtx_.unlock();
}

void PhysicsEngine::setGravityFromBackgroundNormal(const bool &input)
{
	this->use_background_normal_as_gravity_ = input;
	if (this->use_background_normal_as_gravity_ && this->have_background_)
		this->setGravityVectorDirection(-background_surface_normal_);
}

void PhysicsEngine::addObjects(const std::vector<ObjectWithID> &objects)
{
	// mtx_.lock();

	// Add new objects
	if (this->debug_messages_) std::cerr << "Adding scene objects to the physics engine.\n";
	
	for (std::vector<ObjectWithID>::const_iterator it = objects.begin(); 
		it != objects.end(); ++it)
	{
		if (this->debug_messages_) std::cerr << "Adding rigid body " << it->getID() << " to the physics engine's database.\n";
		this->rigid_body_[it->getID()] = it->generateRigidBodyForWorld();
		if (this->debug_messages_) std::cerr << "Adding rigid body " << it->getID() << " to the physics engine's world.\n";
		m_dynamicsWorld->addRigidBody(this->rigid_body_[it->getID()]);

		// set the name of the object in the collision object
		std::string * object_name = new std::string(it->getID());
		this->rigid_body_[it->getID()]->setUserPointer(object_name);

		object_penalty_parameter_database_by_id_[it->getID()] = (*object_penalty_parameter_database_)[it->getObjectClass()];
		
		if (this->debug_messages_)
		{
			std::cerr << "Object " << it->getID() <<", address: " << this->rigid_body_[it->getID()]  <<": \n";
			btTransform object_tf;
			this->rigid_body_[it->getID()]->getMotionState()->getWorldTransform(object_tf);
			btQuaternion q = object_tf.getRotation();
			btVector3 t = object_tf.getOrigin();
			std::cerr << "Quaterion: " << q[0] << ", " << q[1] << ", "  << q[2] << ", "  << q[3] << std::endl;
			std::cerr << "Translation: " << t[0]  << ", " << t[1]  << ", " << t[2] << std::endl;
		}
	}
	if (this->debug_messages_) std::cerr << "Scene objects added to the physics engine.\n";

	// mtx_.unlock();
}

void PhysicsEngine::simulate()
{
	// mtx_.lock();

	if (this->debug_messages_) std::cerr << "Simulating the physics engine.\n";
	// If do not have background, do not update the scene
	
	if (!this->have_background_)
	{
		std::cerr << "Skipping simulation, scene does not has any background data yet.\n";
		return;
	}
	else
	{
		// set the background name
		std::string * background_name = new std::string("background");
		this->background_->setUserPointer(background_name);
	}

	if (this->rendering_launched_)
	{
		this->counter_ = 0;
		this->object_velocity_.clear();
		this->object_acceleration_.clear();
		this->in_simulation_ = true;
		while (this->in_simulation_ && this->counter_ < 10 )
		{
			// Do nothing;

			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
			this->counter_++;
		}
		this->in_simulation_ = false;
	}
	else
	{
		for (int i = 0; i < 300; i++)
		{
			m_dynamicsWorld->stepSimulation(simulation_step_, 10);
			if (this->checkSteadyState()) break;
		}
	}

}
bool PhysicsEngine::checkSteadyState()
{
	bool steady_state = true;
	for (std::map<std::string, btRigidBody*>::const_iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		// Check if any object is still moving
		if (it->second->getActivationState() == 1)
		{
			steady_state = false;
			break;
		}
	}
	return steady_state;
}

std::map<std::string, btTransform>  PhysicsEngine::getUpdatedObjectPose()
{
	if (this->debug_messages_) std::cerr << "Getting updated scene objects poses.\n";
	// Run simulation to get an update on object poses
	this->simulate();

	// mtx_.lock();
	std::map<std::string, btTransform> result_pose;
	for (std::map<std::string, btRigidBody*>::const_iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		it->second->getMotionState()->getWorldTransform(result_pose[it->first]);

		if (this->debug_messages_)
		{
			std::cerr << "Object " << it->first << std::endl;
			btQuaternion q = result_pose[it->first].getRotation();
			btVector3 t = result_pose[it->first].getOrigin();
			std::cerr << "Quaterion: " << q[0] << ", " << q[1] << ", "  << q[2] << ", "  << q[3] << std::endl;
			std::cerr << "Translation: " << t[0]  << ", " << t[1]  << ", " << t[2] << std::endl;
		}
	}

	if (this->debug_messages_) std::cerr << "Done scene objects poses.\n";

	// mtx_.unlock();

	return result_pose;
}

void PhysicsEngine::resetObjects()
{
	// mtx_.lock();
	if (this->debug_messages_) std::cerr << "Removing all scene objects.\n";
	// Removes all objects from the physics world then delete its' content
	for (std::map<std::string, btRigidBody*>::iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		m_dynamicsWorld->removeRigidBody(it->second);
		delete (std::string*) it->second->getUserPointer();
		delete it->second->getMotionState();
		delete it->second;
		if (this->debug_messages_) std::cerr << "Removed objects: "<<  it->first <<".\n";
	}
	this->rigid_body_.clear();
	if (this->debug_messages_) std::cerr << "Done removing all scene objects.\n";

	// mtx_.unlock();
}

void PhysicsEngine::cacheObjectVelocities(const btScalar &timeStep)
{
	for (std::map<std::string, btRigidBody*>::const_iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		btVector3 current_lin_vel = it->second->getLinearVelocity(),
			current_ang_vel = it->second->getAngularVelocity();
		if (keyExistInConstantMap(it->first,this->object_velocity_))
		{
			btVector3 avg_lin_acc = (current_lin_vel - object_velocity_[it->first].linear_)/ timeStep,
				avg_ang_acc = (current_ang_vel - object_velocity_[it->first].angular_)/ timeStep;

			this->object_acceleration_[it->first].setValue(avg_lin_acc, avg_ang_acc);
		}
		this->object_velocity_[it->first].setValue(current_lin_vel, current_ang_vel);
	}
}

void PhysicsEngine::setDebugMode(bool debug)
{
	this->debug_messages_ = debug;
}


void PhysicsEngine::renderingLaunched()
{
	this->rendering_launched_ = true;
}

void PhysicsEngine::initPhysics()
{
	setTexturing(true);
	setShadows(true);
	m_broadphase = new btDbvtBroadphase();
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_solver  = new btSequentialImpulseConstraintSolver;
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, 
		m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld->setDebugDrawer(&gDebugDraw);
	
	m_dynamicsWorld->setInternalTickCallback(_worldTickCallback,static_cast<void *>(this),true);
}

void PhysicsEngine::exitPhysics()
{
	// Clean up pointers
	if (this->debug_messages_) std::cerr << "Shutting down physics engine.\n";
	// delete all object contents
	this->resetObjects();

	// removes background
	for (int i = 0; i < m_collisionShapes.size(); i++)
		delete this->m_collisionShapes.at(0);
	if (this->have_background_)
	{
		m_dynamicsWorld->removeRigidBody(this->background_);
		delete this->background_->getMotionState();
		delete this->background_;
	}

	// delete physics world environment
	if (this->debug_messages_) std::cerr << "Deleting physics engine environment.\n";
	delete m_dynamicsWorld;
	delete this->m_solver;
	delete this->m_dispatcher;
	delete this->m_collisionConfiguration;
	delete this->m_broadphase;
}

void	PhysicsEngine::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void PhysicsEngine::clientMoveAndDisplay()
{
	// mtx_.lock();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();
	
	///step the simulation
	if (m_dynamicsWorld)
	{
		if (this->in_simulation_){
			m_dynamicsWorld->stepSimulation(simulation_step_, 10);
			if (this->checkSteadyState()) this->in_simulation_ = false;
		}
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}
	
	renderme(); 
	glFlush();
	swapBuffers();
	// mtx_.unlock();
}

void PhysicsEngine::displayCallback(void) {

	// mtx_.lock();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	renderme();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
	glFlush();
	swapBuffers();

	// mtx_.unlock();
}


void PhysicsEngine::setCameraClippingPlaneNearFar(btScalar near, btScalar far)
{
	this->m_frustumZNear = near;
	this->m_frustumZFar = far;
}

void PhysicsEngine::setCameraPositionAndTarget(btVector3 cam_position, btVector3 cam_target)
{
	this->m_cameraPosition = cam_position;
	this->m_cameraTargetPosition = cam_target;
	m_cameraDistance = m_cameraTargetPosition.distance(m_cameraPosition);
	
	// calculate polar coordinate of the camera
	btVector3 target_to_cam_direction = (m_cameraPosition - m_cameraTargetPosition).normalize();
	this->m_ele = 90 - acos(target_to_cam_direction[1]) * 57.29577951308232;
	this->m_azi = atan2(-target_to_cam_direction[0],-target_to_cam_direction[2]) * 57.29577951308232;
}

void PhysicsEngine::setObjectPenaltyDatabase(std::map<std::string, ObjectPenaltyParameters> * penalty_database)
{
	this->object_penalty_parameter_database_ = penalty_database;
}

void PhysicsEngine::worldTickCallback(const btScalar &timeStep) {
	// mtx_.lock();
    // printf(“The world just ticked by %f seconds\n”, (float)timeStep);
    // calculate the scene analysis here
    this->cacheObjectVelocities(timeStep);
    for (std::map<std::string, btRigidBody*>::const_iterator it = this->rigid_body_.begin(); 
		it != this->rigid_body_.end(); ++it)
	{
		if (keyExistInConstantMap(it->first,this->object_acceleration_))
		{
			if(this->debug_messages_)
				std::cerr << it->first << " ";
			calculateStabilityPenalty(this->object_acceleration_[it->first], 
				object_penalty_parameter_database_by_id_[it->first], gravity_magnitude_);
		}
	}
	generateObjectSupportGraph(m_dynamicsWorld, timeStep, gravity_vector_, this->debug_messages_);
	// mtx_.unlock();
}


