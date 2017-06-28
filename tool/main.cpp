#include "obj_convex_decomposition.h"

#include <iostream>
#include <sstream>
#include <vector>

#include <btBulletDynamicsCommon.h>

#include "bcs_loader.h"

bool saveAsBulletRigidBody(std::string filename, btCollisionShape* input, btScalar mass=1.0, btScalar friction = 0.3)
{
	btDefaultMotionState* object_motion_state = 
		new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));
	btVector3 inertia(0, 0, 0);
    input->calculateLocalInertia(mass, inertia);

	btRigidBody::btRigidBodyConstructionInfo object_RigidBodyCI(mass, 
		object_motion_state, input);

	btRigidBody* object_rigidbody = new btRigidBody(object_RigidBodyCI);
	// object_rigidbody->setFriction(friction);
	// object_rigidbody->setRollingFriction(friction);
	// object_rigidbody->setDamping(0.5,0.5);
	btDefaultSerializer serializer;
	serializer.startSerialization();
	object_rigidbody->serializeSingleObject(&serializer);
	serializer.finishSerialization();

	// add extension
	filename = filename + ".bullet";

	FILE* file = fopen(filename.c_str(),"wb");
	if (file)
	{
		fwrite(serializer.getBufferPointer(),serializer.getCurrentBufferSize(),1, file);
		fclose(file);
		printf("bullet file saved to: \"%s\"\n",filename.c_str());
		return true;
	}
	else {	
		 printf("ERROR: Can't save the bullet file to: \"%s\"\n",filename.c_str());
		 return false;
	}
}

int main(int argc, char *argv[])
{
	std::string file_io[2];
	int HACD_parameter[7];

	// CONVERTING BCS FILE TO BULLET FILE
	if (argc == 3)
	{
		for (int i = 1; i < argc; i++)
		{
			std::stringstream ss_filename(argv[i]);
			ss_filename >> file_io[i - 1];
		}
		btCollisionShape* simplified_mesh = load_bcs(file_io[0].c_str(), false);
		if (simplified_mesh)
		{
			btCompoundShape* compound_shape = (btCompoundShape*)simplified_mesh;
			if (saveAsBulletRigidBody(file_io[1], compound_shape)) return 0;
		}
		return 1;
	}
	// USAGE INFORMATION
	else if (argc < 9)
	{
		std::cerr << "Convert existing .bcs file to .bullet file\n";
		std::cerr << "Usage[1]: " << argv[0] << " input_file\n";
		std::cerr << "For example" << argv[0] << "input.bcs output\n";

		std::cerr << "Generate new .bcs file and .bullet file\n";
		std::cerr << "Usage: " << argv[0]
			<< " input_file output_file n_clusters concavity invert add_extra_distance_points"
			<< " add_neighbours_distance_points add_faces_points (optional)max_hull_vertices\n";
		std::cerr << "For example: " << argv[0] << " input.obj output 2 100 0 0 0 0 100\n";
		return 1;
	}
	// CONVEX DECOMPOSITION
	else {
		for (int i = 1; i < argc; i++)
		{
			
			if (i < 3)
			{
				std::stringstream ss_filename(argv[i]);
				ss_filename >> file_io[i - 1];
			}
			else
			{
				std::istringstream ss_integer_input(argv[i]);
				if (!(ss_integer_input >> HACD_parameter[i - 3]))
				{
					std::cerr << "Invalid number. Clustering parameter should be an integer.\n";
					return 1;
				}
			}
		}
	}

	size_t n_clusters = HACD_parameter[0];
	double concavity = HACD_parameter[1];
	bool invert = HACD_parameter[2];
	bool add_extra_distance_points = HACD_parameter[3];
	bool add_neighbours_distance_points = HACD_parameter[4];
	bool add_faces_points = HACD_parameter[5];
	if (argc == 9) {
		ObjConvexDecomposition obj_decomp(n_clusters, concavity, invert,
			 add_extra_distance_points, add_neighbours_distance_points, add_faces_points);
		obj_decomp.setInputFile(file_io[0]);
		obj_decomp.setOutputFilename(file_io[1]);
		obj_decomp.setGenerateAdditionalVRMLfile(true);
		if (obj_decomp.computeDecomposition())
		{
			obj_decomp.saveCompoundShape();
			saveAsBulletRigidBody(file_io[1], obj_decomp.getResult());
		}
	}
	else {
		size_t max_hull_vertices = HACD_parameter[6];
		ObjConvexDecomposition obj_decomp(n_clusters, concavity, invert,
			 add_extra_distance_points, add_neighbours_distance_points, add_faces_points, max_hull_vertices);
		obj_decomp.setInputFile(file_io[0]);
		obj_decomp.setOutputFilename(file_io[1]);
		obj_decomp.setGenerateAdditionalVRMLfile(true);
		{
			obj_decomp.saveCompoundShape();
			saveAsBulletRigidBody(file_io[1], obj_decomp.getResult());
		}
	}
	return 0;
}