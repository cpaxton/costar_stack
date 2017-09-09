#include <iostream>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>

#include "scene_data_forces.h"

std::string printbtVector3(const btVector3 &vec)
{
	std::stringstream ss;
	ss << vec.x() << ", " << vec.y() << ", " << vec.z();
	return ss.str();
}

int main()
{
	std::string package_path = ros::package::getPath("sequential_scene_parsing");

	PointCloudXYZPtr block_only_scene(new PointCloudXYZ());
	pcl::PCDReader reader;
	if (reader.read(package_path+"/mesh/wood_cube.pcd",*block_only_scene) == 0)
	{
		std::cerr << "Block point cloud is loaded successfully\n";
	}
	else
	{
		std::cerr << "Fail to load scene point cloud located at: " << package_path+"/mesh/wood_cube.pcd" << ".\n";
		return 0;
	}

	FeedbackDataForcesGenerator data_forces_generator;
	data_forces_generator.setFeedbackForceMode(1);
	data_forces_generator.setModelDirectory(package_path+"/mesh");
	data_forces_generator.setModelCloud("wood_cube");
	data_forces_generator.setDebugMode(true);

	// 0.1 gravity, 5cm
	data_forces_generator.setForcesParameter(1.,0.25);
	data_forces_generator.setSceneData(block_only_scene);

	btTransform test_pose;
	test_pose.setIdentity();

	// the block detected block is translated 1 cm in -x direction. The data forces should give +x directional forces
	test_pose.setOrigin(btVector3(-0.5,0.,0.));
	std::pair<btVector3, btVector3> feedback = data_forces_generator.applyFeedbackForcesDebug(test_pose,"wood_cube");
	std::cerr << "The block detected block is translated 1 cm in -x direction. The data forces should give +x directional forces with 0 torque\n";
	std::cerr << "Force: " << printbtVector3(feedback.first) << "; Torque: " << printbtVector3(feedback.second) << std::endl;

	test_pose.setOrigin(btVector3(0.,0.5,0.));
	feedback = data_forces_generator.applyFeedbackForcesDebug(test_pose,"wood_cube");
	std::cerr << "The block detected block is translated 1 cm in +y direction. The data forces should give -y directional forces with 0 torque\n";
	std::cerr << "Force: " << printbtVector3(feedback.first) << "; Torque: " << printbtVector3(feedback.second) << std::endl;

	test_pose.setOrigin(btVector3(0.5, 0. ,0.5));
	std::cerr << "The block detected block is translated 1 cm in +x and +z direction. The data forces should give -x and -z directional forces with 0 torque\n";
	feedback = data_forces_generator.applyFeedbackForcesDebug(test_pose,"wood_cube");
	std::cerr << "Force: " << printbtVector3(feedback.first) << "; Torque: " << printbtVector3(feedback.second) << std::endl;

	return 0;
}