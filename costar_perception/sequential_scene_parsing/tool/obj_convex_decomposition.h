#ifndef OBJ_CONVEX_DECOMPOSITION_H
#define OBJ_CONVEX_DECOMPOSITION_H
// This code is based on Convex Decomposition demo and btHACDCompoundShape.h

// Bullet stuffs
#include <HACD/hacdHACD.h>

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btStridingMeshInterface.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h>
#include <BulletCollision/Gimpact/btGImpactShape.h>
#include <LinearMath/btGeometryUtil.h>
#include <LinearMath/btSerializer.h>
#include <ConvexDecomposition/ConvexDecomposition.h>
#include <ConvexDecomposition/cd_wavefront.h>

// Standard library stuff
#include <vector>
#include <algorithm>
#include <stdio.h> //printf debugging
#include <iostream>
#include <sstream>

class ObjConvexDecomposition
{
public:
	// constructor
	ObjConvexDecomposition();
	ObjConvexDecomposition(size_t n_clusters, btScalar concavity, bool invert, bool add_extra_distance_points, 
		bool add_neighbours_distance_points, bool add_faces_points, size_t max_hull_vertices = 100);
	void setOutputFilename(std::string filename);
	void setGenerateAdditionalVRMLfile(bool vrml_flag);
	void setInputFile(std::string input_file_location);

	// gives better convex hull quality, but slightly slower
	void setPolyhedralContactClipping(bool poly_flag);

	// this will compute HACD decomposition and saved the output file. If HACD failed, return false
	bool computeDecomposition();
	bool saveCompoundShape();
	btCompoundShape* getResult();

private:
	// .obj file location
	std::string input_file_location_;

	// .bcs filename output
	std::string filename_;
	bool vrml_flag_;

	// HACD parameters
	// Recommended parameters: 2 100 0 0 0 0 100
	size_t n_clusters_;
	btScalar concavity_;
	bool invert_;
	bool add_extra_distance_points_;
	bool add_neighbours_distance_points_;
	bool add_faces_points_;
	size_t max_hull_vertices_;

	// output parameter
	bool convex_hull_enable_polyhedral_contact_clipping_;
	bool reduce_hull_vertices_using_BtShapeHull_;
	btScalar convex_hull_collision_margin_;
	// result
	btCompoundShape result;
};

#endif