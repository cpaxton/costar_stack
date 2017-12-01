#include <iostream>
#include <stdlib.h> 
#include <ctime>
#include <sstream>
#include "sp_segmenter/symmetricOrientationRealignment.h"

double generateRandomOrientation(const int &symmetricProperties)
{
    const double degToRad = boost::math::constants::pi<double>() / 180;
	int random = rand()%(int(360/symmetricProperties)) * symmetricProperties;
	std::cout << random << ", ";
	return (random) * degToRad;
}

/// tests if the symmetry between two quaternions matches what is expected
/// @param 
/// return true on success false on failure
bool test_symmetry(Eigen::Quaternion<float> right_side_up_q,
	               Eigen::Quaternion<float> up_side_down_q,
	               ObjectSymmetry sym,
	               bool is_equivalent = true)
{

    // TODO(ahundt) are the wxyz data entries the same between rviz and eigen?
    // see "orientation flipping bug poses.txt" for raw data + screeenshot of what this means
    Eigen::Quaternion<float> identity = Eigen::Quaternion<float>::Identity();
	Eigen::Quaternion<float> normalized_right_side_up_q = normalizeModelOrientation(right_side_up_q,sym);
	Eigen::Quaternion<float> normalized_up_side_down_q = normalizeModelOrientation(up_side_down_q,sym);
	// normalizeModelOrientation(rand_link_q,link);
	bool isTestSuccessful = (normalized_right_side_up_q.angularDistance(normalized_up_side_down_q) < 0.01) && is_equivalent;

	if(!isTestSuccessful)
	{
		std::cerr << "Angular distance: " << normalized_right_side_up_q.angularDistance(normalized_up_side_down_q) << "\n";

		if(is_equivalent)  std::cerr << "ERROR! quaternions *should* match but don't! \n";
		if(!is_equivalent) std::cerr << "ERROR! quaternions should *not* match but do! \n";

		std::cerr << "right_side_up_q "; printQuaternion(right_side_up_q); std::cerr << "up_side_down_q "; printQuaternion(up_side_down_q); std::cerr << "\n";
		std::cerr << "normalized_right_side_up_q "; printQuaternion(normalized_right_side_up_q); std::cerr << "normalized_up_side_down_q "; printQuaternion(normalized_up_side_down_q); std::cerr << "\n\n";
	}

	return isTestSuccessful;
}


int main(int argc, char **argv)
{
    const double degToRad = boost::math::constants::pi<double>() / 180;
	srand(0);//time(NULL));
	std::string mode;
	// for (int i = 0; i < argc; i++)
	// 	std::cout << i << " arg: " << argv[i] << std::endl;
	if (argc-1 == 1) mode = argv[1];
	else 
	{
		std::cout << "Number of args: " << argc - 1 << std::endl;
		std::cout << "This program need to have one argument. Please enter node or link\n";
		return -1;
	}

	bool check_link=false, check_node=false;
	check_link = (mode == std::string("link"));
	check_node = (mode == std::string("node"));
	if (!check_link && !check_node)
	{
		std::cout << "Please enter args as either link or node\n";
		return -1;
	}
	std::stringstream ss;
	ObjectSymmetry link(180.,180.,90.);
	ObjectSymmetry node(90.,90.,90.);


	Eigen::Quaternion<float> right_side_up_q(0.55944, 0.57531, -0.40931, -0.43418);
	Eigen::Quaternion<float> up_side_down_q(0.69686, -0.11566, 0.69885, 0.11234);
	if (test_symmetry(right_side_up_q, up_side_down_q, link)) {
		std::cout << "Test successful!\n";
	} else {
		std::cout << "Test failed!\n";
	}

	int rnd_node_x, rnd_node_y, rnd_node_z;
	int rnd_link_x, rnd_link_y, rnd_link_z;
	rnd_node_x = rand() % 90 - 45;
	rnd_node_y = rand() % 90 - 45;
	rnd_node_z = rand() % 90 - 45;
	rnd_link_x = rand() % 180 - 90;
	rnd_link_y = rand() % 180 - 90;
	rnd_link_z = rand() % 90 - 45;

	bool remove45Degrees;
	bool haveSecondResult = false;

	Eigen::Matrix3f node_result, link_result;
	Eigen::Quaternion<float> initial_node, initial_link, firstResult, secondResult;
	if (check_node){
		ss << "Target RPY node: " << rnd_node_x << " "
			<< rnd_node_y << " "
			<< rnd_node_z << std::endl;
		std::cout << ss.str();
		node_result = Eigen::Matrix3f::Identity() * Eigen::AngleAxisf(rnd_node_x * degToRad, Eigen::Vector3f::UnitZ())
				* Eigen::AngleAxisf(rnd_node_y * degToRad, Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(rnd_node_z * degToRad, Eigen::Vector3f::UnitX());
		std::cout << "Node Rotation Matrix: \n" << node_result << std::endl;
		Eigen::Vector3f ea = extractRPYfromRotMatrix(node_result);
		initial_node = Eigen::Quaternion<float>(node_result);
		initial_node = normalizeModelOrientation(initial_node,node);
		node_result = initial_node.matrix();
		ss << "Original matrix:\n" << node_result << std::endl;
	}
	if (check_link){
		ss << "Target RPY link: " << rnd_link_x << " "
			<< rnd_link_y << " "
			<< rnd_link_z << std::endl;
		std::cout << ss.str();
		link_result = Eigen::Matrix3f::Identity() * Eigen::AngleAxisf(rnd_link_x * degToRad, Eigen::Vector3f::UnitZ())
				* Eigen::AngleAxisf(rnd_link_y * degToRad, Eigen::Vector3f::UnitY())
				* Eigen::AngleAxisf(rnd_link_z * degToRad, Eigen::Vector3f::UnitX());
		std::cout << "Link Rotation Matrix: \n" << link_result << std::endl;
		Eigen::Vector3f ea = extractRPYfromRotMatrix(link_result);
		initial_link = Eigen::Quaternion<float>(link_result);
		initial_link = normalizeModelOrientation(initial_link,link);
		link_result = initial_link.matrix();
		ss << "Original matrix:\n" << link_result << std::endl;
	}
	unsigned int numberOfSuccess = 0, numberOfIteration = 100, consistencyToFirstResult = 0;
	// std::cout << "Target RPY link: " << rnd_link_x << " "
	// 	<< rnd_link_y << " "
	// 	<< rnd_link_z << std::endl;

	for (unsigned int i = 0; i < numberOfIteration; i++)
	{
		if (check_node){
			double node_roll =  generateRandomOrientation(90);
			double node_pitch = generateRandomOrientation(90);
			double node_yaw =   generateRandomOrientation(90);
			std::cout<<std::endl;
			Eigen::Matrix3f rand_node_rotmatrix;
			rand_node_rotmatrix = node_result * Eigen::AngleAxisf(node_yaw, Eigen::Vector3f::UnitZ())
	    		* Eigen::AngleAxisf(node_pitch, Eigen::Vector3f::UnitY())
	    		* Eigen::AngleAxisf(node_roll, Eigen::Vector3f::UnitX());
    		// std::cerr  << "True initial Rot Matrix: \n" << rand_node_rotmatrix << std::endl;
			Eigen::Quaternion<float> rand_node_q(rand_node_rotmatrix);
			Eigen::Quaternion<float> normalizedQ = normalizeModelOrientation(rand_node_q,node);
			//normalizeModelOrientation(rand_node_q,node);
			if (i == 0)firstResult = normalizedQ;
			else if (firstResult.angularDistance(normalizedQ) < 0.01) consistencyToFirstResult++;
			else if (!haveSecondResult) {
				std::cout << "^Different from first result??\n";
				secondResult = normalizedQ; haveSecondResult = true;
			}

			if (initial_node.angularDistance(normalizedQ) < 0.01) numberOfSuccess++;
			else std::cout << "^Different from Ori??\n";
		}
        if (check_link){
			double link_roll =  generateRandomOrientation(180);
			double link_pitch = generateRandomOrientation(180);
			double link_yaw =   generateRandomOrientation(90);
			std::cout<<std::endl;
			Eigen::Matrix3f rand_link_rotmatrix;
			rand_link_rotmatrix = link_result * Eigen::AngleAxisf(link_yaw, Eigen::Vector3f::UnitZ())
			* Eigen::AngleAxisf(link_pitch, Eigen::Vector3f::UnitY())
			* Eigen::AngleAxisf(link_roll, Eigen::Vector3f::UnitX());
				std::cerr  << "Initial Rot Matrix: \n" << rand_link_rotmatrix << std::endl;
			Eigen::Quaternion<float> rand_link_q(rand_link_rotmatrix);
			Eigen::Quaternion<float> normalizedQ = normalizeModelOrientation(rand_link_q,link);
			// normalizeModelOrientation(rand_link_q,link);
			if (i == 0)firstResult = normalizedQ;
			else if (firstResult.angularDistance(normalizedQ) < 0.01) consistencyToFirstResult++;
			else if (!haveSecondResult) {
			       std::cout << "^Different from first result??\n";
			       secondResult = normalizedQ; haveSecondResult = true;
			}
			if (initial_link.angularDistance(normalizedQ) < 0.01)  numberOfSuccess++;
			else std::cout << "^Different from Ori??\n";
       }

	}
	std::cout << ss.str();
	// std::cout << "Ori Matrix: \n" << node_result << std::endl;
	std::cout << "Number of success = " << numberOfSuccess << " out of " << numberOfIteration << std::endl;
	std::cout << "Consistency to first result = " << consistencyToFirstResult << " out of " << numberOfIteration - 1 << std::endl;
	if (haveSecondResult){
		std::cout << "First set result: \n" << firstResult.angularDistance(Eigen::Quaternion<float>::Identity ()) << std::endl;	
		std::cout << "Second set result: \n" << secondResult.angularDistance(Eigen::Quaternion<float>::Identity ()) << std::endl;
	}
	

	return 0;
}