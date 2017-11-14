#include "bullet_hello_world_draw.h"

int main (int argc, char* argv[])
{

    helloBulletDemo ccdDemo;
    ccdDemo.initPhysics();


#ifdef CHECK_MEMORY_LEAKS
    ccdDemo.exitPhysics();
#else
    return glutmain(argc, argv,1024,600,"Bullet Physics Demo. http://bulletphysics.org",&ccdDemo);
#endif
    
    //default glut doesn't return from mainloop
    return 0;
}