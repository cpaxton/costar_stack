

#include "sequential_scene_parsing.h"


SceneGraph::SceneGraph(ImagePtr input, ImagePtr background_image)
{
	this->addBackground(background_image);
	// this->processImage(input);
	this->physics_engine_ready_ = false;
}

// void SceneGraph::setPhysicsEngine(PhysicsEngine* physics_engine)
void SceneGraph::setPhysicsEngine(PhysicsEngine* physics_engine)
{
	if (this->debug_messages_) std::cerr <<"Setting physics engine into the scene graph.\n";
	this->physics_engine_ = physics_engine;

	// Check if physics engine exist
	this->physics_engine_ready_ = (physics_engine_ != NULL);
	if (this->physics_engine_ready_)
	{
		this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME + RUN_UNTIL_HAVE_SUPPORT_GRAPH);
	}
}

void SceneGraph::addBackground(ImagePtr background_image, int mode)
{
	if (this->debug_messages_) std::cerr <<"Adding background into the scene graph.\n";
	
	// add background
	// this->object_label_.push_back("g");
	this->background_label_ = "g";
	this->object_point_cloud_["g"] = background_image;
	btTransform identity; identity.setIdentity();
	this->object_instance_parameter_["g"] = identity;

	// search for plane of support
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<ImagePoint> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (background_image);
	seg.segment (*inliers, *coefficients);
	Eigen::Vector3f normal(coefficients->values[0],coefficients->values[1], coefficients->values[2]);
	btVector3 normal_bt = convertEigenToBulletVector(normal);
	float coeff = coefficients->values[3];

	Eigen::Vector4f cloud_centroid;
	pcl::compute3DCentroid(*background_image,cloud_centroid);
	btVector3 bt_cloud_centroid(cloud_centroid[0],cloud_centroid[1],cloud_centroid[2]);

	switch (mode)
	{
		case BACKGROUND_PLANE:
		{
			if (inliers->indices.size () == 0)
			{
				std::cerr <<"Could not estimate a planar model for the given dataset.";
				return;
			}

			if (this->debug_messages_) std::cerr << "Background(plane) normal: "<< normal.transpose() <<", coeff: " << coeff << std::endl;
			this->physics_engine_->addBackgroundPlane(normal_bt * SCALING, -btScalar(coeff * SCALING),bt_cloud_centroid * SCALING);
			break;
		}
		case BACKGROUND_HULL:
		{
			std::vector<btVector3> convex_plane_points;
			convex_plane_points.reserve(background_image->size());
			for (std::size_t i = 0; i < background_image->size(); i++)
			{
				btVector3 convex_hull_point(background_image->points[i].x,background_image->points[i].y,background_image->points[i].z);
				convex_plane_points.push_back(convex_hull_point * SCALING);
			}
			this->physics_engine_->addBackgroundConvexHull(convex_plane_points, normal_bt * SCALING);
			break;
		}
		case BACKGROUND_MESH:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr background_no_color (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::copyPointCloud(*background_image,*background_no_color);

			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (background_no_color);
			n.setInputCloud(background_no_color);
			n.setSearchMethod (tree);
			n.setKSearch (20);
			n.compute (*normals);

			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields (*background_no_color, *normals, *cloud_with_normals);
			//* cloud_with_normals = cloud + normals

			// Create search tree*
			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud (cloud_with_normals);

			pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
			pcl::PolygonMesh triangles;

			// Set the maximum distance between connected points (maximum edge length)
			gp3.setSearchRadius (0.025);

			// Set typical values for the parameters
			gp3.setMu (2.5);
			gp3.setMaximumNearestNeighbors (100);
			gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			gp3.setMinimumAngle(M_PI/18); // 10 degrees
			gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			gp3.setNormalConsistency(false);

			// Get result
			gp3.setInputCloud (cloud_with_normals);
			gp3.setSearchMethod (tree2);
			gp3.reconstruct (triangles);

			// Additional vertex information
			std::vector<int> parts = gp3.getPartIDs();
			std::vector<int> states = gp3.getPointStates();

			pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::fromPCLPointCloud2 (triangles.cloud,*mesh_cloud);
			btTriangleMesh* trimesh = new btTriangleMesh();
			
			for (std::vector<pcl::Vertices>::const_iterator it = triangles.polygons.begin(); it != triangles.polygons.end(); ++it )
			{
				trimesh->addTriangle( 
						pclPointToBulletVector(mesh_cloud->points[ it->vertices[0] ]) * SCALING,
						pclPointToBulletVector(mesh_cloud->points[ it->vertices[1] ]) * SCALING,
						pclPointToBulletVector(mesh_cloud->points[ it->vertices[2] ]) * SCALING 
					);
			}
			this->physics_engine_->addBackgroundMesh(trimesh,normal_bt * SCALING, bt_cloud_centroid * SCALING);

			break;
		}
		default:
			break;
	}
}

void SceneGraph::addScenePointCloud(ImagePtr scene_image)
{
	if (scene_image->empty())
	{
		std::cerr << "Error, input scene cloud is empty.\n";
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_coordinates_only(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*scene_image, *point_coordinates_only);
	data_probability_check_.setPointCloudData(point_coordinates_only);
}

void SceneGraph::addNewObjectTransforms(const std::vector<ObjectWithID> &objects)
{
	this->physics_engine_->resetObjects();
	// object_label_.reserve(objects.size());
	object_label_class_map.clear();
	for (std::vector<ObjectWithID>::const_iterator it = objects.begin(); it != objects.end(); ++it)
	{
		object_label_class_map[it->getID()] = it->getObjectClass();
		// object_label_.push_back();
	}

	if (this->debug_messages_) std::cerr <<"Adding new objects into the scene graph.\n";
	this->physics_engine_->addObjects(objects);
}

std::map<std::string, ObjectParameter> SceneGraph::getCorrectedObjectTransform()
{
	if (this->debug_messages_) std::cerr <<"Getting corrected object transform from the scene graph.\n";
	seq_mtx_.lock();
	this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME,1./200,30);
	std::map<std::string, ObjectParameter> result = this->physics_engine_->getUpdatedObjectPose();
	this->getCurrentSceneSupportGraph();
	write_graphviz(std::cout, this->scene_support_graph_, label_writer(this->scene_support_graph_));

	seq_mtx_.unlock();
	return result;
}

bool SceneGraph::loadObjectModels(const std::string &input_model_directory_path, 
	const std::vector<std::string> &object_names)
{
	std::cerr << "Adding objrecransac model.\n";
	bool result = boost::filesystem::is_directory(input_model_directory_path);
	if (result)
	{
	    std::string model_path = input_model_directory_path;
	    if (*model_path.rbegin() != '/')
	        model_path += "/";

	    for (std::vector<std::string>::const_iterator it = object_names.begin(); it != object_names.end(); ++it)
		    data_probability_check_.addModelFromPath(model_path + *it, *it);
		std::cerr << "Done.\n";
		return true;
	}
	else
	{
		std::cerr << "Error, object model directory: " << input_model_directory_path << " does not exist.\n";
		return false;
	}
}

void SceneGraph::setObjectSymmetryMap(const std::map<std::string, ObjectSymmetry> &object_symmetry_map)
{
	this->object_symmetry_map_ = object_symmetry_map;
}

void SceneGraph::setDebugMode(bool debug)
{
	this->debug_messages_ = debug;
}

void SceneGraph::getCurrentSceneSupportGraph()
{
	this->scene_support_graph_ = this->physics_engine_->getCurrentSceneGraph(this->vertex_map_);
}

void SceneGraph::getUpdatedSceneSupportGraph()
{
	this->scene_support_graph_ = this->physics_engine_->getUpdatedSceneGraph(this->vertex_map_);
}

void SceneGraph::setObjectHypothesesMap(std::map<std::string, ObjectHypothesesData > &object_hypotheses_map)
{
	this->object_hypotheses_map_ = object_hypotheses_map;
}

double SceneGraph::evaluateObjectProbability(const std::string &object_label, const std::string &object_model_name, const bool &verbose)
{
	// std::cerr << "Accessing support graph data.\n";
	vertex_t object_in_graph = this->vertex_map_[object_label];
	scene_support_vertex_properties &object_physics_status = this->scene_support_graph_[object_in_graph];
	btTransform &object_pose = object_physics_status.object_pose_;
	if (!object_physics_status.ground_supported_)
	{
		if (verbose)
		{
			std::cerr << object_label << " is skipped because it is not supported by the ground.\n";
			return 0;
		}
	}
	// std::cerr << "Calculating physical probability criterion.\n";

	const double &stability_probability = object_physics_status.stability_penalty_;
	double support_probability = getObjectSupportContribution(object_physics_status);
	double collision_probability = getObjectCollisionPenalty(object_physics_status);
	
	// std::cerr << "Calculating data match probability criterion.\n";
	double object_data_compliance = this->data_probability_check_.getConfidence(object_model_name, object_pose);
	double object_total_probability = object_data_compliance * support_probability * stability_probability * collision_probability;

	if (verbose)
	{
		std::cerr << "Probability =  " <<  object_total_probability << "; "
			<< "data = " << object_data_compliance << ", "
			<< "stability = " << stability_probability << ", "
			<< "support = " << support_probability << ", "
			<< "collision = " << collision_probability
			// << ", " << "penetration_depth = " << object_physics_status.penetration_distance_ 
			<< std::endl;	
	}
	return object_total_probability;
}

double SceneGraph::evaluateSceneOnObjectHypothesis(std::map<std::string, btTransform> &object_pose_from_graph, 
	const std::string &object_label, const std::string &object_model_name, bool &background_support_status,
	const btTransform &object_pose_hypothesis, const bool &reset_position)
{
	this->physics_engine_->prepareSimulationForOneTestHypothesis(object_label, object_pose_hypothesis, reset_position);
	this->getUpdatedSceneSupportGraph();

	double scene_hypothesis = 1;
	for (std::map<std::string, vertex_t>::iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		bool verbose = false;
		if (it->first == object_label) verbose = true;

		// only check probability for object that are exist in the dictionary
		if (object_label_class_map.find(it->first) == object_label_class_map.end()) continue;

		double obj_probability = this->evaluateObjectProbability(it->first, object_label_class_map[it->first], verbose);

		vertex_t &object_in_graph = this->vertex_map_[object_label];
		object_pose_from_graph[object_label] = this->scene_support_graph_[object_in_graph].object_pose_;

		if (obj_probability == 0)
		{
			if (background_support_status)
			{
				scene_hypothesis *= 0.00000001;
			}
			else
			{
				// skips the object if it has 0 probability (invalid object state) and the best data
				// probability says that the object is not background supported
				continue;
			}
		}
		else
		{
			// update the background support status if there exist a hypothesis where the object is not floating
			if (!background_support_status) background_support_status = true;
			scene_hypothesis *= obj_probability;
		}
	}
	// double hypothesis_probability = this->evaluateObjectProbability(object_label, object_model_name);
	// std::cerr << "Getting object probability.\n";
	return scene_hypothesis;
}

void SceneGraph::evaluateAllObjectHypothesisProbability()
{
	// Set the best test pose map based on the best data
	std::map<vertex_t, bool> vertex_background_support_status;

	seq_mtx_.lock();
	this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME + RUN_UNTIL_HAVE_SUPPORT_GRAPH, 1./500);
	vertex_t &ground_vertex = this->vertex_map_["background"];
	OrderedVertexVisitor vis  = getOrderedVertexList(this->scene_support_graph_, ground_vertex);

	std::map<vertex_t,std::size_t> ordered_vertex_map =  vis.getVertexVisitMap();
	std::vector<vertex_t> v_ordered = vis.getVertexVisitList();
	std::vector<vertex_t> disconnected_root_vertex;

	// get disconnected components from the ground and add it to the ordered vertex map
	// TODO: Actually finds all roots of disconnected components, then find and add the ordered vertex visitor
	// to the list of vertex
	std::size_t counter = ordered_vertex_map.size();
	for (std::map<std::string, vertex_t>::iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		if (ordered_vertex_map.find(it->second) == ordered_vertex_map.end())
		{
			ordered_vertex_map[it->second] = ++counter;
		}
	}

	std::map<std::size_t, vertex_t> value_ordered_vertex_map;
	for (std::map<vertex_t, std::size_t>::iterator it = ordered_vertex_map.begin(); 
		it != ordered_vertex_map.end(); ++it)
	{
		value_ordered_vertex_map[it->second] = it->first;
	}

	std::cerr << "Vertex fix order: \n";
	for (std::map<vertex_t, std::size_t>::iterator it = value_ordered_vertex_map.begin(); 
		it != value_ordered_vertex_map.end(); ++it)
	{
		std::cerr << this->scene_support_graph_[it->second].object_id_ << " " << it->first << std::endl;
		vertex_background_support_status[it->first] = this->scene_support_graph_[it->second].ground_supported_;
	}

	seq_mtx_.unlock();

	double best_object_probability_effect = 0;
	for (std::map<std::size_t, vertex_t>::iterator it = value_ordered_vertex_map.begin();
		it != value_ordered_vertex_map.end(); ++it)
	// for (std::map<std::string, ObjectHypothesesData >::const_iterator it = this->object_hypotheses_map_.begin();
	// 	it != this->object_hypotheses_map_.end(); ++it)
	{
		const std::string &object_pose_label = this->scene_support_graph_[it->second].object_id_;
		const ObjectHypothesesData &obj_hypotheses = this->object_hypotheses_map_[object_pose_label];
		const std::string &object_model_name = obj_hypotheses.first;
		// const std::string &object_pose_label = it->first;
		// const std::string &object_model_name = it->second.first;
		const std::vector<ObjectParameter> &object_pose_hypotheses = obj_hypotheses.second;
		bool &current_background_support_status = vertex_background_support_status[it->second];
		bool updated_background_support_status = current_background_support_status;
		
		bool update_from_this_object = false;
		ObjectParameter best_object_pose;
		std::map<std::string, ObjectParameter> best_object_pose_from_graph;

		std::size_t counter = 0;
		for (std::vector<ObjectParameter>::const_iterator it2 = object_pose_hypotheses.begin();
			it2 != object_pose_hypotheses.end(); ++it2)
		{
			ObjectParameter object_pose = *it2;
			// std::cerr << "-------------------------------------------------------------\n";
			std::cerr << "Evaluating object: " << object_pose_label << " hypothesis #" << ++counter << std::endl;
			int i = 0;
			std::map<std::string, ObjectParameter> tmp_object_pose_config;
			// for (int i = 0; i < 5; i++)
			// {
				seq_mtx_.lock();
				double scene_hypothesis_probability = this->evaluateSceneOnObjectHypothesis(tmp_object_pose_config,
					object_pose_label, object_model_name, updated_background_support_status,
					object_pose, true);
				// get updated object pose from the scene simulation
				// object_pose = this->scene_support_graph_[it->second].object_pose_;

				seq_mtx_.unlock();

				// update the best scene if the current scene probability is better or there is
				// a change from not background supported to background supported on this object hypothesis
				if ((best_object_probability_effect < scene_hypothesis_probability) ||
					(!current_background_support_status && updated_background_support_status)
					)
				{
					if (!current_background_support_status && updated_background_support_status)
					{
						current_background_support_status = updated_background_support_status;
					}
					best_object_pose = object_pose;
					best_object_probability_effect = scene_hypothesis_probability;
					best_object_pose_from_graph = tmp_object_pose_config;
					update_from_this_object = true;
				}

				std::cerr << "Scene probability = " << scene_hypothesis_probability
					<< " current best: " << best_object_probability_effect << std::endl;

				// this->physics_engine_->stepSimulationWithoutEvaluation(0.2,1/100.);
			// }
			// std::cerr << "-------------------------------------------------------------\n\n";
		}
		
		// only update if this object provides valid best object pose
		if (update_from_this_object)
		{
			seq_mtx_.lock();
			this->physics_engine_->changeBestTestPoseMap(object_pose_label, best_object_pose);
			seq_mtx_.unlock();
		}
		// this->physics_engine_->changeBestTestPoseMap(best_object_pose_from_graph);
	}

	this->getUpdatedSceneSupportGraph();
	double scene_hypothesis = 1;
	for (std::map<std::string, vertex_t>::iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		bool current_background_support_status = this->scene_support_graph_[it->second].ground_supported_;
		bool best_background_support_status = vertex_background_support_status[it->second];
		// only check probability for object that are exist in the dictionary
		if (object_label_class_map.find(it->first) == object_label_class_map.end()) continue;
		std::cerr << it->first << " ";
		double obj_probability = this->evaluateObjectProbability(it->first, object_label_class_map[it->first], true);

		// skips the object if it has no probability
		if (obj_probability == 0) 
		{
			if (!current_background_support_status && best_background_support_status)
			{
				scene_hypothesis *= 0.00000001;
			}
			else
			{
				continue;
			}
		}
		else scene_hypothesis *= obj_probability;
	}
	std::cerr << "Final Scene probability = " << scene_hypothesis << std::endl;
	// return scene_hypothesis;
}

std::map<std::string, ObjectParameter> SceneGraph::getCorrectedObjectTransformFromSceneGraph()
{
	std::map<std::string, ObjectParameter> result;
	this->seq_mtx_.lock();
	this->getCurrentSceneSupportGraph();
	for (std::map<std::string, vertex_t>::iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		const std::string &object_id = this->scene_support_graph_[it->second].object_id_; 
		const btTransform &pose = this->scene_support_graph_[it->second].object_pose_;
		std::string &model_name = object_label_class_map[object_id];
		
		if (keyExistInConstantMap(model_name, object_symmetry_map_))
		{
			const btTransform &original_pose = this->physics_engine_->getTransformOfBestData(object_id);
			Eigen::Quaternion<float> original_q = convertBulletToEigenQuaternion<float>(original_pose.getRotation());
			Eigen::Quaternion<float> corrected_q = convertBulletToEigenQuaternion<float>(pose.getRotation());
			corrected_q = normalizeModelOrientation(corrected_q, original_q, object_symmetry_map_[model_name]);
			btTransform symmetric_corrected_pose(convertEigenToBulletQuaternion<float>(corrected_q), pose.getOrigin());
			result[object_id] = symmetric_corrected_pose;
		}
		else
		{
			result[object_id] = pose;
		}
	}
	this->seq_mtx_.unlock();
	return result;
}

