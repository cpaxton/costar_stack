#include "obj_convex_decomposition.h"

#include <iostream>
#include <sstream>
#include <vector>

int main(int argc, char *argv[])
{
	std::string file_io[2];
	int HACD_parameter[7];
	if (argc < 9)
	{
		std::cerr << "Usage: " << argv[0] << " input_file output_file n_clusters concavity invert add_extra_distance_points add_neighbours_distance_points add_faces_points (optional)max_hull_vertices\n";
		std::cerr << "For example: " << argv[0] << " input.obj output 2 100 0 0 0 0 100\n";
		return 1;
	}
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
		obj_decomp.computeDecomposition();
	}
	else {
		size_t max_hull_vertices = HACD_parameter[6];
		ObjConvexDecomposition obj_decomp(n_clusters, concavity, invert,
			 add_extra_distance_points, add_neighbours_distance_points, add_faces_points, max_hull_vertices);
		obj_decomp.setInputFile(file_io[0]);
		obj_decomp.setOutputFilename(file_io[1]);
		obj_decomp.setGenerateAdditionalVRMLfile(true);
		obj_decomp.computeDecomposition();
	}
	return 0;
}