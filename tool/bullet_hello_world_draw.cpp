#include "bullet_hello_world_draw.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include <btBulletDynamicsCommon.h>

#include <stdio.h> //printf debugging
#include <LinearMath/btAabbUtil2.h>


#include <iostream>

#include <btBulletDynamicsCommon.h>
// #include <Bullet3Common/b3FileUtils.h>
#include "bcs_loader.h"
#include <math.h>

///scaling of the objects (0.1 = 20 centimeter boxes )
#define SCALING 1.


void helloBulletDemo::clientMoveAndDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    //simple dynamics world doesn't handle fixed-time-stepping
    float ms = getDeltaTimeMicroseconds();
    
    ///step the simulation
    if (m_dynamicsWorld)
    {
        m_dynamicsWorld->stepSimulation(ms / 1000000.f);
        //optional but useful: debug drawing
        m_dynamicsWorld->debugDrawWorld();

        btVector3 aabbMin(1,1,1);
        btVector3 aabbMax(2,2,2);

        // btTransform trans, trans_node;
        // fallRigidBody->getMotionState()->getWorldTransform(trans);
        // if (node_RigidBody!= NULL)
        //     node_RigidBody->getMotionState()->getWorldTransform(trans_node);
        
        // std::cout << "sphere height: " << trans.getOrigin().getY() << ", node height: " << trans_node.getOrigin().getY() << std::endl;                
    }
        
    renderme(); 

    glFlush();

    swapBuffers();

}



void helloBulletDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    
    renderme();

    //optional but useful: debug drawing to detect problems
    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    glFlush();
    swapBuffers();
}

void helloBulletDemo::setCameraClippingPlaneNearFar(btScalar near, btScalar far)
{
    this->m_frustumZNear = near;
    this->m_frustumZFar = far;
}

void helloBulletDemo::setCameraPositionAndTarget(btVector3 cam_position, btVector3 cam_target)
{
    this->m_cameraPosition = cam_position;
    this->m_cameraTargetPosition = cam_target;
    m_cameraDistance = m_cameraTargetPosition.distance(m_cameraPosition);
    
    // calculate polar coordinate of the camera
    btVector3 target_to_cam_direction = (m_cameraPosition - m_cameraTargetPosition).normalize();
    this->m_ele = 90 - acos(target_to_cam_direction[1]) * 57.29577951308232;
    this->m_azi = atan2(-target_to_cam_direction[0],-target_to_cam_direction[2]) * 57.29577951308232;
}

void    helloBulletDemo::initPhysics()
{
    setTexturing(true);
    setShadows(true);
    this->setCameraPositionAndTarget(btVector3(1.0,1.5,1.5), btVector3(0,0,0));
    this->setCameraClippingPlaneNearFar(0.005f);
    // updateCamera();


    ///collision configuration contains default setup for memory, collision setup
    m_collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();

    ///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    m_dispatcher = new  btCollisionDispatcher(m_collisionConfiguration);

    m_broadphase = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
    m_solver = sol;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

    m_dynamicsWorld->setDebugDrawer(&gDebugDraw);

    btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
    m_collisionShapes.push_back(groundShape);
    btDefaultMotionState* groundMotionState = new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 0)));
    btRigidBody::btRigidBodyConstructionInfo
            groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
    btRigidBody*  groundRigidBody = new btRigidBody(groundRigidBodyCI);
    m_dynamicsWorld->addRigidBody(groundRigidBody);

    // btCollisionShape* fallShape = new btSphereShape(0.05);
    // m_collisionShapes.push_back(fallShape);
    // btDefaultMotionState* fallMotionState =
    //         new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 15, 0)));
    // btScalar mass = 1;
    // btVector3 fallInertia(0, 0, 0);
    // fallShape->calculateLocalInertia(mass, fallInertia);
    // btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
    // fallRigidBody = new btRigidBody(fallRigidBodyCI);
    // m_dynamicsWorld->addRigidBody(fallRigidBody);

    // btCollisionShape* node_uniform_shape = load_bcs("./node_uniform.bcs", false);
    // if (node_uniform_shape != NULL){
    //     m_collisionShapes.push_back(node_uniform_shape);
    //     btDefaultMotionState* node_uniform_state =
    //             new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 10, 0)));
    //     btScalar mass_node = 0.3;
    //     btVector3 node_uniform_inertia(0, 0, 0);
    //     node_uniform_shape->calculateLocalInertia(mass_node, node_uniform_inertia);
    //     btRigidBody::btRigidBodyConstructionInfo node_RigidBodyCI(mass_node, node_uniform_state, node_uniform_shape, node_uniform_inertia);
    //     node_RigidBody = new btRigidBody(node_RigidBodyCI);
    //     m_dynamicsWorld->addRigidBody(node_RigidBody);

    //     btTransform trans_node;
    //     node_RigidBody->getMotionState()->getWorldTransform(trans_node);
    //     std::cerr << "node Initial height: " << trans_node.getOrigin().getY() << std::endl;
    
    // }

}


void    helloBulletDemo::clientResetScene()
{
    exitPhysics();
    initPhysics();
}

void helloBulletDemo::exitPhysics()
{

    //remove the rigidbodies from the dynamics world and delete them
    for (int i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    //delete collision shapes
    for (int j=0;j<m_collisionShapes.size();j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }
    m_collisionShapes.clear();

    delete m_dynamicsWorld;
    
    delete m_solver;
    
    delete m_broadphase;
    
    delete m_dispatcher;

    delete m_collisionConfiguration;
}