// This code is based on Convex Decomposition demo and btHACDCompoundShape.h

// Bullet stuffs
#include "obj_convex_decomposition.h"

ObjConvexDecomposition::ObjConvexDecomposition(size_t n_clusters, btScalar concavity, bool invert, bool add_extra_distance_points, 
		bool add_neighbours_distance_points, bool add_faces_points, size_t max_hull_vertices):
	n_clusters_(n_clusters),
	concavity_(concavity),
	invert_(invert),
	add_extra_distance_points_(add_extra_distance_points),
	add_neighbours_distance_points_(add_neighbours_distance_points),
	add_faces_points_(add_faces_points),
	max_hull_vertices_(max_hull_vertices)
{
	this->filename_ = "output";
	this->vrml_flag_ = false;
	this->convex_hull_enable_polyhedral_contact_clipping_ = false;
	this->reduce_hull_vertices_using_BtShapeHull_ = false;
	this->convex_hull_collision_margin_ = 0.0001f; // 0.1 mm
}

void ObjConvexDecomposition::setOutputFilename(std::string filename)
{
	this->filename_ = filename;
}

void ObjConvexDecomposition::setGenerateAdditionalVRMLfile(bool vrml_flag)
{
	this->vrml_flag_ = vrml_flag;
}


void ObjConvexDecomposition::setInputFile(std::string input_file_location)
{
	this->input_file_location_ = input_file_location;
}

void ObjConvexDecomposition::setPolyhedralContactClipping(bool poly_flag)
{
	this->convex_hull_enable_polyhedral_contact_clipping_ = poly_flag;
}

bool ObjConvexDecomposition::computeDecomposition()
{
	ConvexDecomposition::WavefrontObj wo;
	unsigned int number_of_triangles = wo.loadObj(this->input_file_location_.c_str());

	if (number_of_triangles == 0){
		std::cout << "Something wrong with the obj file. Number of triangles = " << number_of_triangles << std::endl;
		return false;
	}

	//-----------------------------------------------
	// HACD
	//-----------------------------------------------

	std::vector< HACD::Vec3<HACD::Real> > points;
	std::vector< HACD::Vec3<long> > triangles;

	for(int i=0; i<wo.mVertexCount; i++ ) 
	{
		int index = i*3;
		HACD::Vec3<HACD::Real> vertex(wo.mVertices[index], wo.mVertices[index+1],wo.mVertices[index+2]);
		points.push_back(vertex);
	}

	for(int i=0;i<wo.mTriCount;i++)
	{
		int index = i*3;
		HACD::Vec3<long> triangle(wo.mIndices[index], wo.mIndices[index+1], wo.mIndices[index+2]);
		triangles.push_back(triangle);
	}


	HACD::HACD myHACD;
	myHACD.SetPoints(&points[0]);
	myHACD.SetNPoints(points.size());
	myHACD.SetTriangles(&triangles[0]);
	myHACD.SetNTriangles(triangles.size());
	myHACD.SetCompacityWeight(0.1);
	myHACD.SetVolumeWeight(0.0);

	myHACD.SetNClusters(this->n_clusters_);                    // minimum number of clusters
	myHACD.SetNVerticesPerCH(this->max_hull_vertices_);        // max of 100 vertices per convex-hull
	myHACD.SetConcavity(this->concavity_);                     // maximum concavity
	myHACD.SetAddExtraDistPoints(this->add_extra_distance_points_);   
	myHACD.SetAddNeighboursDistPoints(this->add_neighbours_distance_points_);   
	myHACD.SetAddFacesPoints(this->add_faces_points_); 

	myHACD.Compute();

	// save vrml file if the vrml flag is set to be true
	if (this->vrml_flag_){
		std::stringstream vrml_filename;
		vrml_filename << this->filename_ << ".wrl";
		myHACD.Save(vrml_filename.str().c_str(), false);	
	}

	// generate the btCompoundShape
	for (size_t CH = 0, nClusters = myHACD.GetNClusters(); CH < nClusters; ++CH)
	{	
		const size_t nPoints =  myHACD.GetNPointsCH(CH);
		const size_t nTriangles =  myHACD.GetNTrianglesCH(CH);
		
		std::vector < HACD::Vec3<HACD::Real> > points(nPoints);
		std::vector < HACD::Vec3<long> > triangles(nTriangles);	// unused, but they could be used to display the convex hull...
		if (myHACD.GetCH(CH, &points[0], &triangles[0]))	{
				btConvexHullShape* convexHullShape = NULL;

				#pragma region Calculate centroid
				// We use the simple aabb center, not the center of the homogeneous mass in the volume.
				// This makes sense, since HACD decomposed shapes are not suitable for being released to 'destroy' an object.
				// That's because they can overlap in various ways (that's not always bad for collision detection, because they
				// can avoid small objects to get stuck between clusters better).	
				btVector3 centroid(0,0,0);
			btVector3 minValue,maxValue,tempVert;
			if (nPoints>0)	{
				minValue=maxValue=btVector3(points[0].X(),points[0].Y(),points[0].Z());
			}
			else minValue=maxValue=btVector3(0,0,0);
				for (size_t i=1; i<nPoints; i++)	{
					tempVert = btVector3(points[i].X(),points[i].Y(),points[i].Z());
				minValue.setMin(tempVert);
				maxValue.setMax(tempVert);
			}
			centroid = (minValue+maxValue)*btScalar(0.5);			
			#pragma endregion
		
			#pragma region Calculate convexHullShape
			btAlignedObjectArray < btVector3 > verts;
			verts.resize(nPoints);
			for (size_t i=0; i<nPoints; i++)	{
				verts[i]=btVector3(points[i].X()-centroid.x(),points[i].Y()-centroid.y(),points[i].Z()-centroid.z());
			}

			convexHullShape = new btConvexHullShape(&verts[0].x(),verts.size());
			convexHullShape->setMargin(this->convex_hull_collision_margin_);
			if (reduce_hull_vertices_using_BtShapeHull_)	{
				//create a hull approximation
					btShapeHull* hull = new btShapeHull(convexHullShape);
				if (hull)	{
					hull->buildHull(this->convex_hull_collision_margin_);
					if (hull->numVertices() < verts.size())	{
						delete convexHullShape;convexHullShape = NULL;
						convexHullShape = new btConvexHullShape((btScalar*)hull->getVertexPointer(),hull->numVertices());
						convexHullShape->setMargin(this->convex_hull_collision_margin_);
					}
						delete hull;hull = NULL;
					}	
			}
			if (this->convex_hull_enable_polyhedral_contact_clipping_) convexHullShape->initializePolyhedralFeatures();	
			#pragma endregion
		
				this->result.addChildShape(btTransform(btQuaternion::getIdentity(),centroid),convexHullShape);	
				// if (params.keepSubmeshesSeparated) m_submeshIndexOfChildShapes.push_back(subPart);
		}
	}
	if (this->result.getNumChildShapes() == 0)
	{
		std::cout << "Convex decomposition failed.\n Number of btCompoundShape child is zero.\n";
		return false;
	}
	else
	{
	 	btDefaultSerializer serializer;
		serializer.startSerialization();
		this->result.serializeSingleShape(&serializer);
		serializer.finishSerialization();

		// add extension
		this->filename_ = this->filename_ + ".bcs";

		FILE* file = fopen(this->filename_.c_str(),"wb");
		if (file)
		{
			fwrite(serializer.getBufferPointer(),serializer.getCurrentBufferSize(),1, file);
			fclose(file);
			printf("btCompoundShape saved to: \"%s\"\n",this->filename_.c_str());
			return true;
		}
		else {	
			 printf("ERROR: I can't save the btCompoundShape to: \"%s\"\n",this->filename_.c_str());
			 return false;
		}
	}
}