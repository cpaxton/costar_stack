#include "color_nn_segmenter.h"

PointCloudXYZ::Ptr ColorNnSegmenter::stripCloudCoordinate(const PointCloudXYZRGB &input_cloud)
{
	// removes the spatial coordinate from point cloud and returns the XYZ point cloud with color coordinate.
	PointCloudXYZ::Ptr result(new PointCloudXYZ());
	result->width = input_cloud.width;
	result->height = input_cloud.height;
	result->resize(result->height*result->width);

	std::size_t i = 0;
	for(PointCloudXYZRGB::const_iterator it = input_cloud.begin(); it != input_cloud.end(); ++it, ++i)
	{
		result->points[i] = pcl::PointXYZ(it->r,it->g,it->b);
    }
    return result;
}

void ColorNnSegmenter::rgbToLabCloud(PointCloudXYZ::Ptr input_cloud)
{
	for(PointCloudXYZ::iterator it = input_cloud->begin(); it != input_cloud->end(); ++it)
	{
		convertRgbToLab(it->x,it->y,it->z);
	}
}

PointCloudXYZ::Ptr ColorNnSegmenter::openCvKMeans(const PointCloudXYZ &input_cloud, const int &cluster_size)
{
	// convert input cloud to opencv Mat
	cv::Mat points(input_cloud.size(),1, CV_32FC3);
	std::size_t idx = 0;

	std::cout << "Input model points:\n";
	for(PointCloudXYZ::const_iterator it = input_cloud.begin(); it != input_cloud.end(); ++it, ++idx)
	{
		points.at<cv::Vec3f>(idx,0) = cv::Vec3f(it->x,it->y,it->z);
		// std::cerr << points.at<cv::Vec3f>(0,idx) << std::endl;
	}

	// reshape to 

	cv::Mat labels;
	int attempts = 10;
	cv::Mat centers;
	// bool success = false;

	int max_cluster_size = input_cloud.size() > cluster_size ? cluster_size : input_cloud.size(); 
	
	// use opencv kmeans to extract the cluster center, since pcl 1.7.2 does not have kmeans
	cv::kmeans(points, max_cluster_size, labels, 
		cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10e4, 1e-4),
		attempts, cv::KMEANS_RANDOM_CENTERS, centers);

	// std::cerr << "Kmeans size: " << centers.size() << std::endl;
	// success = centers.rows > 0;

	PointCloudXYZ::Ptr kmeans_result(new PointCloudXYZ());
	std::cout << "Kmeans of the model:\n";
	for (unsigned int i = 0; i < centers.rows; ++i )
	{
		// for (unsigned int j = 0; j < centers.cols; ++j)
		{
			cv::Vec3f cv_center = centers.at<cv::Vec3f>(i);
			std::cout << i << ": " << cv_center << std::endl;
			pcl::PointXYZ center_point(cv_center[0],cv_center[1],cv_center[2]);
			kmeans_result->push_back(center_point);
		}
	}
	return kmeans_result;
}

bool ColorNnSegmenter::checkDirectoryValid(const boost::filesystem::path dir_path)
{
	namespace fs = boost::filesystem;
	if ( !fs::exists(dir_path))
	{
		std::cerr << "Error: '" << dir_path << "' does not exist.\n";
		return false;
	}
	else if (!fs::is_directory(dir_path))
	{
		std::cerr << "Error: '" << dir_path << "' is not a directory.\n";
		return false;
	}
	else
	{
		return true;
	}
}

bool ColorNnSegmenter::trainModel(const std::string &training_data_directory, const int kmeans_point_per_model)
{
	namespace fs = boost::filesystem;
	fs::path p(training_data_directory);
	fs::directory_iterator end_iter;

	if (!checkDirectoryValid(p))
	{
		this->ready = false;
		return false;
	}

	// Find all training files
	std::map<std::string,std::vector<std::string> > filepath_in_model; 
	for (fs::directory_iterator model_iter(p); model_iter != end_iter; ++model_iter)
	{
		if (fs::is_directory(model_iter->status()))
		{
			std::string model_name = model_iter->path().filename().string();
			std::cout << "Found model directory: '" << model_name << std::endl;
			filepath_in_model[model_name] = std::vector<std::string>();
			for (fs::directory_iterator file_iter(model_iter->path()); file_iter != end_iter; ++file_iter)
			{
				std::string extension = boost::filesystem::extension(file_iter->path());
				if (extension == ".pcd")
				{
					std::cout << "Found training data: " << file_iter->path().filename() << std::endl;
					filepath_in_model[model_name].push_back(file_iter->path().string());
				}
				else
				{
					std::cout << "Ignored file: " << file_iter->path().filename() << std::endl;
				}
			}
		}
	}

	PointCloudXYZL::Ptr result_model_cloud(new PointCloudXYZL());

	// label 0 is reserved for background objects
	unsigned int label_counter = 1;
	// process the files
	for (std::map<std::string,std::vector<std::string> >::const_iterator it = filepath_in_model.begin(); it != filepath_in_model.end(); ++it, ++label_counter)
	{
		PointCloudXYZ combined_lab_color_cloud;
		for (std::vector<std::string>::const_iterator file_it = it->second.begin(); file_it != it->second.end(); ++file_it )
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (*file_it, *cloud) == -1) 
			{
				std::cerr << "Fail to load file " << *file_it << std::endl;
				continue;
			}
			else if (cloud->size() == 0)
			{
				std::cerr << "Cloud in file " << *file_it << " is empty.\n";
				continue;
			}
			
			// remove NaN points from all saved training clouds
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
			if (pcl::io::savePCDFile(*file_it,*cloud,true) == -1)
			{
				std::cerr << "Fail to update saved cloud after removing NaN points.\n";
			}
			
			PointCloudXYZ::Ptr color_only_cloud = this->stripCloudCoordinate(*cloud);
			this->rgbToLabCloud(color_only_cloud);
			if (file_it == it->second.begin())
			{
				combined_lab_color_cloud = *color_only_cloud;
			}
			else
			{
				combined_lab_color_cloud += *color_only_cloud;
			}
		}

		// pcl::Kmeans kmeans(combined_lab_color_cloud.size(), 3);
		// kmeans.setClusterSize(kmeans_point_per_model);
		// kmeans.addDataPoint (combined_lab_color_cloud.points); 
		// kmeans.kMeans();

		// pcl::Kmeans::Centroids centroids = kmeans.get_centroids();
		if (combined_lab_color_cloud.size() == 0)
		{
			std::cerr << "Warning, color model type: " << it->first << " is skipped because it has no data points.\n";
			continue;
		}

		PointCloudXYZ::Ptr centroids = this->openCvKMeans(combined_lab_color_cloud, kmeans_point_per_model);

		for(PointCloudXYZ::const_iterator pt_it = centroids->begin(); pt_it != centroids->end(); ++pt_it) 
		{
			pcl::PointXYZL cluster_centroid;
			cluster_centroid.x = pt_it->x;
			cluster_centroid.y = pt_it->y;
			cluster_centroid.z = pt_it->z;
			cluster_centroid.label = label_counter;
			result_model_cloud->points.push_back(cluster_centroid); 
		}
		
		label_index_map[label_counter] = it->first;
	}

	if (result_model_cloud->size() > 0)
	{
		model_cloud = result_model_cloud;

		pcl::copyPointCloud(*model_cloud, *unlabelled_model_cloud);
		kdtree.setInputCloud(unlabelled_model_cloud);
		this->ready = true;
		return true;
	}
	else
	{
		std::cerr << "Training failed, no model cloud is generated.\n";
		this->ready = false;
		return false;
	}
}

bool ColorNnSegmenter::saveModel(const std::string &target_directory, const std::string &model_name)
{
	namespace fs = boost::filesystem;
	fs::path p(target_directory);
	if (!checkDirectoryValid(p))
	{
		if (fs::create_directories(p))
		{
			std::cout << "Created model directory" << p.string().c_str() << std::endl;
		}
		else
		{
			std::cerr << "Fail to create model directory" << p.string().c_str() << std::endl;
			return false;
		}
	}

	if (model_cloud->size() > 0)
	{
		fs::path cloud_path = p;
		cloud_path /= model_name + ".pcd";
		if (pcl::io::savePCDFile(cloud_path.string().c_str(),*model_cloud,true) == -1)
		{
			std::cerr << "Error, fail to save the model cloud data in: " << cloud_path.string();
			return false;
		}

		std::map<std::string, std::string> data_map;
		for (std::map<unsigned, std::string>::const_iterator it = label_index_map.begin(); it != label_index_map.end(); ++it)
		{
			std::string label_as_string = boost::lexical_cast<std::string>(it->first);
			data_map[label_as_string] = it->second;
		}
		// assume that the cloud is always saved in the same directory as the dat file
		data_map["cloud_filename"] = cloud_path.filename().string();
		
		fs::path map_path = p;
		map_path /= model_name + ".dat";
		std::ofstream out_map_file;
		out_map_file.open(map_path.string().c_str());
		boost::archive::text_oarchive oarch(out_map_file);
		oarch << data_map;

		return true;
	}
	else
	{
		std::cerr << "Error, the model cloud is empty.\n";
		return false;
	}
}

bool ColorNnSegmenter::loadModel(const std::string &model_dat_file_path)
{
	namespace fs = boost::filesystem;
	fs::path p(model_dat_file_path);
	if (!fs::exists(p))
	{
		std::cerr << "Model file: " << p.string() << " does not exist.\n";
		return false;
	}
	fs::path cloud_path = p.parent_path();

	std::ifstream in_map_file(model_dat_file_path.c_str());
	boost::archive::text_iarchive iarch(in_map_file);
	std::map<std::string, std::string> data_map;

	iarch >> data_map;
	std::string cloud_filename = data_map["cloud_filename"];
	cloud_path /= cloud_filename;
	
	std::string str_cloud_path = cloud_path.string();
	PointCloudXYZL::Ptr cloud_data(new PointCloudXYZL());

	if (pcl::io::loadPCDFile(str_cloud_path.c_str(), *cloud_data) == -1) 
	{
		std::cerr << "Fail to load file " << str_cloud_path << std::endl;
		this->ready = false;
		return false;
	}
	else if (cloud_data->size() == 0)
	{
		std::cerr << "Error: Cloud data in " << str_cloud_path << " is empty.\n";
		this->ready = false;
		return false;
	}

	data_map.erase("cloud_filename");
	for (std::map<std::string, std::string>::const_iterator it = data_map.begin(); it != data_map.end(); ++it)
	{
		unsigned int label_index = boost::lexical_cast<unsigned int>(it->first);
		label_index_map[label_index] = it->second;
	}

	model_cloud = cloud_data;
	pcl::copyPointCloud(*model_cloud, *unlabelled_model_cloud);
	kdtree.setInputCloud(unlabelled_model_cloud);
	this->ready = true;

	return true;
}


void ColorNnSegmenter::setBackgroundColorLabel(const std::string &ignored_labels)
{
	background_label_index_map.clear();
	std::vector<std::string> ignored_label_list;
	boost::split(ignored_label_list,ignored_labels,boost::is_any_of(","));

	std::map<std::string,unsigned int> inverted_label_map;
	for (std::map<unsigned int, std::string>::const_iterator it = label_index_map.begin(); it != label_index_map.end(); ++it)
	{
		inverted_label_map[it->second] = it->first;
	}

	for (std::vector<std::string>::const_iterator it = ignored_label_list.begin(); it != ignored_label_list.end(); ++it)
	{
		if (inverted_label_map.find(*it)!=inverted_label_map.end())
		{
			std::cerr << "Ignored color label: " << *it <<  " with index: " << inverted_label_map[*it] << std::endl;
			background_label_index_map.insert(inverted_label_map[*it]);	
		}
	}
}

PointCloudXYZL::Ptr ColorNnSegmenter::segment(const PointCloudXYZRGB &input_cloud)
{
	if (!this->ready)
	{
		std::cerr << "Error: Color segmenter is not ready yet.\n";
		return PointCloudXYZL::Ptr();
	}

	PointCloudXYZ::Ptr color_only_cloud = this->stripCloudCoordinate(input_cloud);
	this->rgbToLabCloud(color_only_cloud);

	int k = 1;
	std::vector<int> point_idx_knn(k);
	std::vector<float> point_knn_distance(k);
	
	PointCloudXYZL::Ptr segmentation_result(new PointCloudXYZL());
	pcl::copyPointCloud(input_cloud,*segmentation_result);

	std::size_t idx = 0;

	const float bad_distance = std::numeric_limits<float>::quiet_NaN();
	pcl::PointXYZL ignored_point;
	ignored_point.x = bad_distance; ignored_point.y = bad_distance; ignored_point.z = bad_distance;

	for(PointCloudXYZ::const_iterator pt_it = color_only_cloud->begin(); pt_it != color_only_cloud->end(); ++pt_it, ++idx) 
	{
		pcl::PointXYZL &target_point = segmentation_result->points[idx];
		if (!pcl_isfinite(target_point.x) || !pcl_isfinite(target_point.y) || !pcl_isfinite(target_point.z))
		{
			target_point.label = 0;
			continue;
		}

		if ( kdtree.nearestKSearch (*pt_it, k, point_idx_knn, point_knn_distance) > 0 )
		{
			// use the nearest neighboor to find the color label
			const int &nearest_neighboor_pt_idx = point_idx_knn[0];
			const unsigned int &label = model_cloud->points[nearest_neighboor_pt_idx].label;
			if (background_label_index_map.find(label) == background_label_index_map.end())
			{
				target_point.label = label;
			}
			else
			{
				target_point.label = 0;
				// segmentation_result->points[idx] = ignored_point;
			}
		}
	}
	return segmentation_result;
}
