#include "sequential_scene_parsing.h"


SceneHypothesisAssessor::SceneHypothesisAssessor(ImagePtr input, ImagePtr background_image)
{
	this->addBackground(background_image);
	// this->processImage(input);
	this->physics_engine_ready_ = false;
}

// void SceneHypothesisAssessor::setPhysicsEngine(PhysicsEngine* physics_engine)
void SceneHypothesisAssessor::setPhysicsEngine(PhysicsEngine* physics_engine)
{
	if (this->debug_messages_) std::cerr <<"Setting physics engine into the scene graph.\n";
	this->physics_engine_ = physics_engine;
	this->physics_engine_->setFeedbackDataForcesGenerator(&data_forces_generator_);
	// Check if physics engine exist
	this->physics_engine_ready_ = (physics_engine_ != NULL);
	if (this->physics_engine_ready_)
	{
		this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME + RUN_UNTIL_HAVE_SUPPORT_GRAPH);
	}
}

void SceneHypothesisAssessor::addBackground(ImagePtr background_image, int mode)
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

void SceneHypothesisAssessor::addScenePointCloud(ImagePtr scene_image)
{
	if (scene_image->empty())
	{
		std::cerr << "Error, input scene cloud is empty.\n";
	}
	seq_mtx_.lock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr point_coordinates_only(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*scene_image, *point_coordinates_only);
	// data_probability_check_.setPointCloudData(point_coordinates_only);
	data_forces_generator_.setSceneData(point_coordinates_only);
	sequential_scene_hypothesis_.setConfidenceCheckTool(data_forces_generator_);
	sequential_scene_hypothesis_.setSceneData(point_coordinates_only);
	std::cerr << "Scene point cloud has been updated.\n";
	seq_mtx_.unlock();
}

void SceneHypothesisAssessor::addNewObjectTransforms(const std::vector<ObjectWithID> &objects)
{
	// TODO:: Only permanently remove rigid bodies that are removed by sequential_scene_hypothesis
	this->physics_engine_->resetObjects(false);
	// object_label_.reserve(objects.size());
	
	// do not delete the label class map
	// object_label_class_map.clear();
	for (std::vector<ObjectWithID>::const_iterator it = objects.begin(); it != objects.end(); ++it)
	{
		object_label_class_map[it->getID()] = it->getObjectClass();
		// object_label_.push_back();
	}

	if (this->debug_messages_) std::cerr <<"Adding new objects into the scene graph.\n";
	this->physics_engine_->addObjects(objects);
}

std::map<std::string, ObjectParameter> SceneHypothesisAssessor::getCorrectedObjectTransform()
{
	if (this->debug_messages_) std::cerr <<"Getting corrected object transform from the scene graph.\n";
	seq_mtx_.lock();
	this->data_forces_generator_.resetCachedIcpResult();

	FeedbackDataForcesGenerator tmp_data_forces_generator = this->data_forces_generator_;
	// Setup the feedback data forces to be very high when running thru the best data
	// this->data_forces_generator_.setForcesParameter(10,0.05);

	this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME,1./120,30);
	// this->physics_engine_->setSimulationMode(BULLET_DEFAULT,1./200,30);

	std::map<std::string, ObjectParameter> result = this->physics_engine_->getUpdatedObjectPoses();
	this->getCurrentSceneSupportGraph();

	write_graphviz(std::cout, this->scene_support_graph_, label_writer(this->scene_support_graph_));
	std::map<std::string, int> object_action_map;
	for (std::map<std::string, ObjectParameter>::iterator it = result.begin(); it != result.end(); ++it)
	{
		object_action_map[it->first] = ADD_OBJECT;
	}
	double scene_hypothesis = this->evaluateSceneProbabilityFromGraph(object_action_map);
	std::cerr << "Final Scene probability = " << scene_hypothesis << std::endl;

	// set test pose to the converged best data pose
	// this->physics_engine_->changeBestTestPoseMap(result);

	// this->data_forces_generator_ = tmp_data_forces_generator;
	seq_mtx_.unlock();
	return result;
}

bool SceneHypothesisAssessor::loadObjectModels(const std::string &input_model_directory_path, 
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
		{
			// data_probability_check_.addModelFromPath(model_path + *it, *it);
			
			std::stringstream ss;
			ss << model_path << *it << ".pcd";
			pcl::PCDReader reader;
			pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud(new pcl::PointCloud<pcl::PointXYZ>());
			if (reader.read(ss.str(),*surface_cloud) == 0){
				data_forces_generator_.setModelCloud(surface_cloud, *it);
				std::cerr << "Model point cloud loaded successfully\n";
			}
			else
				std::cerr << "Failed to load " << *it << " model point cloud\n"; 
		}

		std::cerr << "Done.\n";
		return true;
	}
	else
	{
		std::cerr << "Error, object model directory: " << input_model_directory_path << " does not exist.\n";
		return false;
	}
}

void SceneHypothesisAssessor::setObjectSymmetryMap(const std::map<std::string, ObjectSymmetry> &object_symmetry_map)
{
	this->object_symmetry_map_ = object_symmetry_map;
}

void SceneHypothesisAssessor::setDebugMode(bool debug)
{
	this->debug_messages_ = debug;
}

void SceneHypothesisAssessor::getCurrentSceneSupportGraph()
{
	this->scene_support_graph_ = this->physics_engine_->getCurrentSceneGraph(this->vertex_map_);
}

void SceneHypothesisAssessor::getUpdatedSceneSupportGraph()
{
	this->scene_support_graph_ = this->physics_engine_->getUpdatedSceneGraph(this->vertex_map_);
}

void SceneHypothesisAssessor::setObjectHypothesesMap(std::map<std::string, ObjectHypothesesData > &object_hypotheses_map)
{
	this->object_hypotheses_map_ = object_hypotheses_map;
}

double SceneHypothesisAssessor::evaluateObjectProbability(const std::string &object_label, 
	const std::string &object_model_name, const int &object_action, const bool &verbose)
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
	// double ransac_confidence = this->data_probability_check_.getConfidence(object_model_name, object_pose);
	double ransac_confidence = this->data_forces_generator_.getIcpConfidenceResult(object_model_name, object_pose);

	// ignore data compliance if it is support retained object
	double object_data_compliance = object_action != SUPPORT_RETAINED_OBJECT ? 
		dataProbabilityScale(ransac_confidence) : 1.0;
	
	double object_total_probability = object_data_compliance * support_probability * stability_probability * collision_probability;

	if (verbose)
	{
		std::cerr << "Probability =  " <<  object_total_probability << "; "
			<< "object_data_compliance = " << object_data_compliance<< ", "
			<< "data = " << ransac_confidence << ", "
			<< "stability = " << stability_probability << ", "
			<< "support = " << support_probability << ", "
			<< "collision = " << collision_probability
			// << ", " << "penetration_depth = " << object_physics_status.penetration_distance_ 
			<< std::endl;
	}
	return object_total_probability;
}

double SceneHypothesisAssessor::evaluateSceneOnObjectHypothesis(std::map<std::string, btTransform> &object_pose_from_graph, 
	const std::string &object_label, const std::string &object_model_name, bool &background_support_status,
	const btTransform &object_pose_hypothesis, const int &object_action, const bool &reset_position)
{
	this->physics_engine_->prepareSimulationForOneTestHypothesis(object_label, object_pose_hypothesis, reset_position);
	this->getUpdatedSceneSupportGraph();

	double scene_hypothesis = 1;
	for (std::map<std::string, vertex_t>::iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		if (it->first == "background") continue;

		bool verbose = false;
		if (it->first == object_label) verbose = true;

		// only check probability for object that are exist in the dictionary
		if (object_label_class_map.find(it->first) == object_label_class_map.end()) continue;

		double obj_probability = this->evaluateObjectProbability(it->first, object_label_class_map[it->first], object_action, verbose);

		vertex_t &object_in_graph = this->vertex_map_[it->first];
		object_pose_from_graph[it->first] = this->scene_support_graph_[object_in_graph].object_pose_;

		if (obj_probability == 0)
		{
			if (background_support_status)
			{
				scene_hypothesis *= 1e-20;
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
	return scene_hypothesis;
}

void SceneHypothesisAssessor::evaluateAllObjectHypothesisProbability()
{
	// Set the best test pose map based on the best data
	this->data_forces_generator_.resetCachedIcpResult();
	std::map<std::string, bool> object_background_support_status;

	// optimize the scene probability by the distance to the ground vertex
	std::vector<map_string_transform> object_test_pose_map_by_dist;
	std::map<std::string, map_string_transform> object_childs_map;

	this->getSceneSupportGraphFromCurrentObjects(object_background_support_status,object_test_pose_map_by_dist,
		object_childs_map);

	std::cerr << "Object list best data: ";
	for (std::size_t dist_idx = 0; dist_idx < object_test_pose_map_by_dist.size(); ++dist_idx)
	{
		for (map_string_transform::iterator it = object_test_pose_map_by_dist[dist_idx].begin(); 
		it != object_test_pose_map_by_dist[dist_idx].end(); ++it)
		{
			std::cerr << "[" << dist_idx << "]" << it->first <<", ";
		}
	}
	std::cerr << std::endl;

	SceneHypothesis current_best_scene(vertex_map_,scene_support_graph_);
	this->sequential_scene_hypothesis_.setCurrentDataSceneStructure(current_best_scene);
	std::map<std::string, AdditionalHypotheses> hypotheses_to_test = 
		sequential_scene_hypothesis_.generateObjectHypothesesWithPreviousKnowledge(object_test_pose_map_by_dist,
			object_childs_map,
			this->object_hypotheses_map_);

	this->physics_engine_->removeAllRigidBodyFromWorld();
	
	// resimulate objects with retained object
	for (std::size_t dist_idx = 0; dist_idx < object_test_pose_map_by_dist.size(); ++dist_idx)
	{
		this->physics_engine_->addExistingRigidBodyBackFromMap(object_test_pose_map_by_dist[dist_idx]);
	}

	this->physics_engine_->stepSimulationWithoutEvaluation(1.5 * GRAVITY_SCALE_COMPENSATION, 
		GRAVITY_SCALE_COMPENSATION/120.);
	this->getUpdatedSceneSupportGraph();
	this->physics_engine_->changeBestTestPoseMap(this->physics_engine_->getCurrentObjectPoses());
	this->getSceneSupportGraphFromCurrentObjects(object_background_support_status,object_test_pose_map_by_dist,
		object_childs_map);

	std::cerr << "Object list + additional hypothesis after simulation:\n";
	for (std::size_t dist_idx = 0; dist_idx < object_test_pose_map_by_dist.size(); ++dist_idx)
	{
		for (map_string_transform::iterator it = object_test_pose_map_by_dist[dist_idx].begin(); 
		it != object_test_pose_map_by_dist[dist_idx].end(); ++it)
		{
			std::cerr << "[" << dist_idx << "]" << it->first << std::endl 
				<< printTransform(it->second);

			const AdditionalHypotheses &obj_hypotheses = hypotheses_to_test[it->first];
			bool ignore_data_forces = !(obj_hypotheses.object_action_ == ADD_OBJECT || obj_hypotheses.object_action_ == PERTURB_OBJECT);
			this->physics_engine_->setIgnoreDataForces(it->first,ignore_data_forces);
		}
	}
	std::cerr << std::endl;
	std::cerr << "Scene hypotheses structure:\n";
	write_graphviz(std::cerr, this->scene_support_graph_, label_writer(this->scene_support_graph_));
	this->physics_engine_->removeAllRigidBodyFromWorld();


	std::map<std::string, map_string_transform> child_of_vertices;
	
	OneFrameSceneHypotheses scene_hypotheses_list;
	std::map<std::string, int> scene_object_hypothesis_id;
	std::map<std::string, int> object_action_map;

	for (std::size_t dist_idx = 0; dist_idx < object_test_pose_map_by_dist.size(); ++dist_idx)
	{
		this->physics_engine_->addExistingRigidBodyBackFromMap(object_test_pose_map_by_dist[dist_idx]);
		for (std::map<std::string, btTransform>::const_iterator it = object_test_pose_map_by_dist[dist_idx].begin(); 
			it != object_test_pose_map_by_dist[dist_idx].end(); ++it)
		{
			scene_object_hypothesis_id[it->first] = 0;
		}
		// Force best scene to update when the distance increased.
		// bool force_update_by_increased_distance = true;

		// each object must contribute to one best hypotheses

		for (std::map<std::string, btTransform>::iterator it = object_test_pose_map_by_dist[dist_idx].begin();
			it != object_test_pose_map_by_dist[dist_idx].end(); ++it)
		{
			double best_object_probability_effect = 0;

			const std::string &object_pose_label = it->first;
			if (!keyExistInConstantMap(object_pose_label, hypotheses_to_test))
			{
				std::cerr << "Skipped " << object_pose_label << " because its hypothesis does not exist\n";
				continue;
			}
			
			const AdditionalHypotheses &obj_hypotheses = hypotheses_to_test[object_pose_label];
			const std::string &object_model_name = obj_hypotheses.model_name_;
			const std::vector<ObjectParameter> &object_pose_hypotheses = obj_hypotheses.poses_;
			object_action_map[object_pose_label] = obj_hypotheses.object_action_;

			bool &current_background_support_status = object_background_support_status[it->first];
			bool updated_background_support_status = current_background_support_status;
			
			// bool update_from_this_object = false;
			ObjectParameter best_object_pose = object_pose_hypotheses[0];
			std::map<std::string, ObjectParameter> best_object_pose_from_graph;

			std::size_t counter = 0;
			std::size_t number_of_object_hypotheses = object_pose_hypotheses.size();
			double best_ransac_confidence;
			int best_hypothesis_id = 0;

			for (std::vector<ObjectParameter>::const_iterator it2 = object_pose_hypotheses.begin();
				it2 != object_pose_hypotheses.end(); ++it2)
			{
				std::cerr << "Evaluating object: " << object_pose_label << " hypothesis #" 
					<< ++counter << "/" << number_of_object_hypotheses << std::endl;
				// std::cerr << "Transform: " << printTransform(*it2);
				scene_object_hypothesis_id[object_pose_label] = counter - 1;
				ObjectParameter object_pose = *it2;
				double ransac_confidence = this->data_forces_generator_.getIcpConfidenceResult(object_model_name, object_pose);
				if (it2 == object_pose_hypotheses.begin())
				{
					best_ransac_confidence = ransac_confidence;
					std::cerr << "Best hypothesis confidence: " << best_ransac_confidence << std::endl;
				}

				if (ransac_confidence < 0.5 * best_ransac_confidence)
				{
					std::cerr << "Skipped hypothesis with confidence: " << ransac_confidence << std::endl;
					continue;
				}

				// limit observation to 5 best previous hypothesis for static object
				if (obj_hypotheses.object_action_ == STATIC_OBJECT && counter > 5) break;
				else if (counter > 15) break;

				// std::cerr << "-------------------------------------------------------------\n";
				double scene_hypothesis_probability;
				std::map<std::string, ObjectParameter> tmp_object_pose_config;
				for (int i = 0; i < 2; i++)
				{
					this->physics_engine_->stepSimulationWithoutEvaluation(.75 * GRAVITY_SCALE_COMPENSATION, 
						GRAVITY_SCALE_COMPENSATION/120.);

					seq_mtx_.lock();
					scene_hypothesis_probability = this->evaluateSceneOnObjectHypothesis(tmp_object_pose_config,
						object_pose_label, object_model_name, updated_background_support_status,
						object_pose, obj_hypotheses.object_action_, i == 0);

					// get updated object pose from the scene simulation
					const vertex_t updated_vertex = this->vertex_map_[it->first];
					object_pose = this->scene_support_graph_[updated_vertex].object_pose_;

					seq_mtx_.unlock();

					std::cerr << "Scene probability = " << scene_hypothesis_probability << std::endl;
				}

				if (object_childs_map[object_pose_label].size() > 0)
				{
					// check object probability after the object stable and the childs get added back together
					this->physics_engine_->addExistingRigidBodyBackFromMap(object_childs_map[object_pose_label]);
					this->physics_engine_->stepSimulationWithoutEvaluation(.75/GRAVITY_SCALE_COMPENSATION, 
						GRAVITY_SCALE_COMPENSATION/120.);

					seq_mtx_.lock();
					scene_hypothesis_probability = this->evaluateSceneOnObjectHypothesis(tmp_object_pose_config,
						object_pose_label, object_model_name, updated_background_support_status,
						object_pose, obj_hypotheses.object_action_, false);

					// get updated object pose from the scene simulation
					const vertex_t updated_vertex = this->vertex_map_[it->first];
					object_pose = this->scene_support_graph_[updated_vertex].object_pose_;
					seq_mtx_.unlock();
					this->physics_engine_->removeExistingRigidBodyWithMap(object_childs_map[object_pose_label]);

					std::cerr << "Scene probability = " << scene_hypothesis_probability << std::endl;	
				}

				// update the best scene if the current scene probability is better or there is
				// a change from not background supported to background supported on this object hypothesis
				if ((best_object_probability_effect < scene_hypothesis_probability) ||
					(!current_background_support_status && updated_background_support_status)
					// || force_update_by_increased_distance
					)
				{
					if (!current_background_support_status && updated_background_support_status)
					{
						current_background_support_status = updated_background_support_status;
					}
					best_object_pose = object_pose;
					best_object_probability_effect = scene_hypothesis_probability;
					best_object_pose_from_graph = tmp_object_pose_config;
					// update_from_this_object = true;
					best_hypothesis_id = counter - 1;
					// force_update_by_increased_distance = false;
				}
				std::cerr << "Best scene probability: " << best_object_probability_effect << std::endl;

				SceneHypothesis observed_scene(vertex_map_,scene_support_graph_,
					scene_hypothesis_probability, scene_object_hypothesis_id);
				scene_hypotheses_list.push_back(observed_scene);
				// std::cerr << "-------------------------------------------------------------\n\n";
			}
			
			// only update if this object provides valid best object pose
			// if (update_from_this_object)
			{
				seq_mtx_.lock();
				std::cerr << "========================= \n";
				std::cerr << "Update the best map from object: " << object_pose_label 
					<< " hypothesis #" << best_hypothesis_id + 1 << std::endl;
				std::cerr << "========================= \n";
				this->physics_engine_->changeBestTestPoseMap(object_pose_label, best_object_pose);
				scene_object_hypothesis_id[object_pose_label] = best_hypothesis_id;
				this->data_forces_generator_.removeCachedIcpResult(object_pose_label);
				// this->physics_engine_->changeBestTestPoseMap(best_object_pose_from_graph);
				seq_mtx_.unlock();
			}
		}
		
	}
	// set the position to the best result for all objects
	this->physics_engine_->prepareSimulationForWithBestTestPose();
	this->physics_engine_->setSimulationMode(RESET_VELOCITY_ON_EACH_FRAME,GRAVITY_SCALE_COMPENSATION/120.,
		GRAVITY_SCALE_COMPENSATION*30);
	// this->physics_engine_->stepSimulationWithoutEvaluation(.75, 1/120.);
	this->getUpdatedSceneSupportGraph();
	double scene_hypothesis = this->evaluateSceneProbabilityFromGraph(object_action_map);
	std::cerr << "Final Scene probability = " << scene_hypothesis << std::endl;
	SceneHypothesis final_scene(vertex_map_,scene_support_graph_,
		scene_hypothesis, scene_object_hypothesis_id);
	std::cerr << "Final scene structure:\n";
	write_graphviz(std::cerr, this->scene_support_graph_, label_writer(this->scene_support_graph_));

	this->sequential_scene_hypothesis_.setPreviousSceneObservation(SceneObservation(
		final_scene, scene_hypotheses_list, this->object_label_class_map));
	// return scene_hypothesis;
}

double SceneHypothesisAssessor::evaluateSceneProbabilityFromGraph(const std::map<std::string, int> &object_action_map)
{
	double scene_hypothesis = 1;
	for (std::map<std::string, vertex_t>::iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		bool current_background_support_status = this->scene_support_graph_[it->second].ground_supported_;
		bool best_background_support_status = this->scene_support_graph_[it->second].ground_supported_;
		// only check probability for object that are exist in the dictionary
		if (object_label_class_map.find(it->first) == object_label_class_map.end()) continue;
		std::cerr << it->first << " ";
		double obj_probability = this->evaluateObjectProbability(it->first, 
			object_label_class_map[it->first], getContentOfConstantMap(it->first,object_action_map), true);
		// skips the object if it has no probability
		if (obj_probability == 0) 
		{
			if (!current_background_support_status && best_background_support_status)
			{
				scene_hypothesis *= 1e-20;
			}
			else
			{
				continue;
			}
		}
		else scene_hypothesis *= obj_probability;
	}
	return scene_hypothesis;
}

std::map<std::string, ObjectParameter> SceneHypothesisAssessor::getCorrectedObjectTransformFromSceneGraph()
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


void SceneHypothesisAssessor::getSceneSupportGraphFromCurrentObjects(
	std::map<std::string, bool> &object_background_support_status,
	std::vector< std::map<std::string, btTransform> > &object_test_pose_map_by_dist,
	std::map<std::string, map_string_transform> &object_childs_map)
{
	seq_mtx_.lock();
	this->getCurrentSceneSupportGraph();

	vertex_t &ground_vertex = this->vertex_map_["background"];
	OrderedVertexVisitor vis = getOrderedVertexList(this->scene_support_graph_, ground_vertex, false);
	object_childs_map = getAllChildTransformsOfVertices(vis.getVertexDistanceMap());
	std::map<std::size_t, std::vector<vertex_t> > vertex_visit_by_dist = vis.getVertexVisitOrderByDistances();

	// DO NOT USE THE VERTEX IDX DIRECTLY, SINCE IT MAY CHANGE WHEN SCENE GRAPH IS UPDATED

	std::map<std::string, btTransform> disconnected_vertices_poses;
	object_test_pose_map_by_dist.clear();
	object_test_pose_map_by_dist.reserve(vertex_visit_by_dist.size());

	for (std::map<std::size_t, std::vector<vertex_t> >::iterator it = vertex_visit_by_dist.begin();
		it != vertex_visit_by_dist.end(); ++it)
	{
		std::map<std::string, btTransform> pose_of_vertices = this->physics_engine_->getAssociatedBestPoseDataFromStringVector(
			getAssociatedIdFromVertexVector(scene_support_graph_, it->second), true);

		for (std::vector<vertex_t>::iterator it_2 = it->second.begin(); it_2 != it->second.end(); ++it_2)
		{
			const std::string &object_pose_label = this->scene_support_graph_[*it_2].object_id_;	
			object_background_support_status[object_pose_label] = this->scene_support_graph_[*it_2].ground_supported_;
		}

		if (it->first != 0)
		{
			object_test_pose_map_by_dist.push_back( pose_of_vertices );
		}
		else
		{
			disconnected_vertices_poses = pose_of_vertices;
		}
	}
	if (disconnected_vertices_poses.size() > 0) 
	{
		object_test_pose_map_by_dist.push_back(disconnected_vertices_poses);
	}
	seq_mtx_.unlock();
}

std::map<std::string, map_string_transform> SceneHypothesisAssessor::getAllChildTransformsOfVertices(
	const std::map<vertex_t, std::size_t> &vertex_distance_map)
{
	std::map<std::string, map_string_transform> result;
	for (std::map<std::string, vertex_t>::const_iterator it = this->vertex_map_.begin();
		it != this->vertex_map_.end(); ++it)
	{
		map_string_transform object_child_transforms;
		std::size_t parent_vertex_distance = getContentOfConstantMap(it->second,vertex_distance_map);
		std::vector<vertex_t> childs = getAllChildVertices(this->scene_support_graph_,it->second);
		for (std::vector<vertex_t>::iterator c_it = childs.begin(); c_it != childs.end(); ++c_it)
		{
			std::size_t child_vertex_distance = getContentOfConstantMap(*c_it,vertex_distance_map);
			// only use the direct child
			if (child_vertex_distance == parent_vertex_distance + 1)
			{
				const std::string &child_name = this->scene_support_graph_[*c_it].object_id_;
				// use best test data in order to reflect the latest change of the scene graph
				object_child_transforms[child_name] = this->physics_engine_->getTransformOfBestData(child_name, true);
			}
		}
		result[it->first] = object_child_transforms;
	}
	return result;
}


