#ifndef BCS_LOADER_H
#define BCS_LOADER_H

#include <BulletWorldImporter/btBulletWorldImporter.h>

inline
btCollisionShape* load_bcs(const char* filename,bool verbose)   {
	btBulletWorldImporter loader(0);//don't store info into the world
	loader.setVerboseMode(verbose);
	if (!loader.loadFile(filename))
	{
		std::cerr << "Open file failed.\n";
		return NULL;	
	} 
	btCollisionShape* shape = NULL;
	if (loader.getNumCollisionShapes()>0) {
		std::cerr << "Number of shapes in the file: " << loader.getNumCollisionShapes();
		shape = loader.getCollisionShapeByIndex(0);
	}
	// Hope there are no leaks.
	return shape;
}

#endif