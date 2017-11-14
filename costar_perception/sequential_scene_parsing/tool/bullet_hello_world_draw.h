#ifndef BASIC_DEMO_H
#define BASIC_DEMO_H

#ifdef _WINDOWS
#include "debugdrawer/Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "debugdrawer/GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif


#include "debugdrawer/GlutStuff.h"
#include "debugdrawer/GLDebugDrawer.h"

#include <LinearMath/btAlignedObjectArray.h>
#include <iostream>
#include <btBulletDynamicsCommon.h>
// #include <Bullet3Common/b3FileUtils.h>

#include "bcs_loader.h"

class helloBulletDemo : public PlatformDemoApplication
{
public:
    helloBulletDemo(){};
    virtual ~helloBulletDemo()
    {
        exitPhysics();
    }

    GLDebugDrawer gDebugDraw;

    void initPhysics();

    void exitPhysics();

    virtual void clientMoveAndDisplay();

    virtual void displayCallback();
    virtual void clientResetScene();

    virtual void setCameraClippingPlaneNearFar(btScalar near, btScalar far = 10000.f);
    virtual void setCameraPositionAndTarget(btVector3 cam_position, btVector3 cam_target);

    static DemoApplication* Create()
    {
        helloBulletDemo* demo = new helloBulletDemo;
        demo->myinit();
        demo->initPhysics();
        return demo;
    }


private:
    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

    btBroadphaseInterface*  m_broadphase;

    btCollisionDispatcher*  m_dispatcher;

    btConstraintSolver* m_solver;

    btDefaultCollisionConfiguration* m_collisionConfiguration;
    btRigidBody* fallRigidBody;
    btRigidBody* node_RigidBody;
};



#endif 