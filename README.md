# Sequential Scene Parsing

This rosnode will improve object detection pose estimate and help solving occluded object detection based on sequential scene parsing derived from paper:
	1. Hager, G.D. and Wegbreit, B., 2011. Scene parsing using a prior world model. The International Journal of Robotics Research, p.0278364911399340.
	2. Brucker, M., Léonard, S., Bodenmüller, T. and Hager, G.D., 2012, May. Sequential scene parsing using range and intensity information. In Robotics and Automation (ICRA), 2012 IEEE International Conference on (pp. 5417-5424). IEEE.

# Requirements
This package requires these dependencies:
	- bullet 2.83+ with EXTRA packages installed.
		- In OSX installation, do brew install bullet --with-extra
		- In linux installation, configure bullet with BUILD_SHARED_LIBS = ON, BUILD_EXTRAS=ON, INSTALL_EXTRA_LIBS=ON then build and it
	- pcl 1.7.2
	- boost 1.59.x

Additionally, this package has a ros compatible bundling. To install this with ros capability:

# Usage
	1. Use obj_convex_decomposition on your input obj mesh file to generate a collision mesh (.bcs). Put the generated file into the mesh directory
	2. Use pcl_mesh_sampling to generate surface sampled point cloud of your input obj mesh file, and put it into the mesh directory.
