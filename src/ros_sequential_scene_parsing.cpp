#include "ros_sequential_scene_parsing.h"
#include "utility_ros.h"

RosSceneHypothesisAssessor::RosSceneHypothesisAssessor()
{
	this->setPhysicsEngine(&this->physics_engine_);
	this->class_ready_ = false;
	this->physics_gravity_direction_set_ = false;
	this->has_tf_ = false;
	this->scene_cloud_updated_ = false;
	this->object_list_updated_ = false;
}

RosSceneHypothesisAssessor::RosSceneHypothesisAssessor(const ros::NodeHandle &nh)
{
	this->setPhysicsEngine(&this->physics_engine_);
	this->physics_gravity_direction_set_ = false;
	this->setNodeHandle(nh);
}

void RosSceneHypothesisAssessor::callGlutMain(int argc, char* argv[])
{
	this->physics_engine_.renderingLaunched();
	glutmain(argc, argv,1024,600,"Scene Parsing Demo",&this->physics_engine_);
}

void RosSceneHypothesisAssessor::exitGlutMain()
{
	this->physics_engine_.renderingLaunched(false);
#ifdef BT_USE_FREEGLUT
	//return from glutMainLoop(), detect memory leaks etc.
	glutLeaveMainLoop();
#else
	exit(0);
#endif
}

void RosSceneHypothesisAssessor::setNodeHandle(const ros::NodeHandle &nh)
{
	this->nh_ = nh;
	this->has_background_ = false;
	this->has_scene_cloud_ = false;
	std::string detected_object_topic;
	std::string background_pcl2_topic;
	std::string scene_pcl2_topic;
	std::string object_folder_location;
	std::string background_location;

	std::string object_hypotheses_topic;
	std::string objransac_model_location, objransac_model_list;
	std::vector<std::string> object_names;

	double data_forces_magnitude_per_point;
	double data_forces_max_distance;

	bool debug_mode, load_table;

	nh.param("detected_object_topic", detected_object_topic,std::string("/detected_object"));
	nh.param("background_pcl2_topic", background_pcl2_topic,std::string("/background_points"));
	nh.param("scene_pcl2_topic", scene_pcl2_topic,std::string("/scene_points"));

	nh.param("object_folder_location",object_folder_location,std::string(""));
	nh.param("TF_z_inv_gravity_dir",this->tf_z_is_inverse_gravity_direction_,std::string(""));
	nh.param("bg_normal_as_gravity",background_normal_as_gravity_,false);

	nh.param("tf_publisher_initial",this->tf_publisher_initial,std::string(""));
	nh.param("debug_mode",debug_mode,false);
	nh.param("load_table",load_table,false);

	nh.param("object_hypotheses_topic",object_hypotheses_topic,std::string("/object_hypothesis"));
	nh.param("objransac_model_directory",objransac_model_location,object_folder_location);
	nh.param("objransac_model_names",objransac_model_list,std::string(""));

	nh.param("data_forces_magnitude",data_forces_magnitude_per_point,0.5);
	nh.param("data_forces_max_distance",data_forces_max_distance,0.01);

	nh.param("best_hypothesis_only",best_hypothesis_only_,false);
	nh.param("small_obj_g_comp",GRAVITY_SCALE_COMPENSATION,3);
	SCALED_GRAVITY_MAGNITUDE = SCALING * GRAVITY_MAGNITUDE / GRAVITY_SCALE_COMPENSATION;

	if (load_table){
		nh.param("table_location",background_location,std::string(""));
		pcl::PCDReader reader;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr background_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
		if (reader.read(background_location,*background_cloud) == 0){
			this->nh_.param("background_mode",background_mode_,0);
			this->addBackground(background_cloud, background_mode_);
			std::cerr << "Background point loaded successfully\n";
		}
		else
			std::cerr << "Failed to load the background points\n"; 
	}

	std::cerr << "Debug mode: " << debug_mode << std::endl;
	this->setDebugMode(debug_mode);

	this->obj_database_.setObjectFolderLocation(object_folder_location);
	if (this->fillObjectPropertyDatabase()) this->obj_database_.loadDatabase(this->physical_properties_database_);
	this->physics_engine_.setObjectPenaltyDatabase(this->obj_database_.getObjectPenaltyDatabase());
	
	this->detected_object_sub = this->nh_.subscribe(detected_object_topic,1,
		&RosSceneHypothesisAssessor::updateSceneFromDetectedObjectMsgs,this);
	this->background_pcl_sub = this->nh_.subscribe(background_pcl2_topic,1,&RosSceneHypothesisAssessor::addBackgroundCallback,this);
	this->scene_pcl_sub = this->nh_.subscribe(scene_pcl2_topic,1,&RosSceneHypothesisAssessor::addSceneCloud,this);

	this->done_message_pub = this->nh_.advertise<std_msgs::Empty>("done_hypothesis_msg",1);

	// setup objrecransac tool
	boost::split(object_names,objransac_model_list,boost::is_any_of(","));
	this->loadObjectModels(objransac_model_location, object_names);
	this->object_hypotheses_sub = this->nh_.subscribe(object_hypotheses_topic,1,
		&RosSceneHypothesisAssessor::fillObjectHypotheses,this);
	
	// setup feedback force parameters
	this->setDataFeedbackForcesParameters(data_forces_magnitude_per_point, data_forces_max_distance);

	// sleep for caching the initial TF frames.
	sleep(1.0);
	
	this->class_ready_ = true;
}

void RosSceneHypothesisAssessor::addBackgroundCallback(const sensor_msgs::PointCloud2 &pc)
{
	if (!this->has_background_)
	{
		this->nh_.param("background_mode",background_mode_,0);
		std::cerr << "Background points added to the scene.\n";
		// convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA::Ptr
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
		pcl::fromROSMsg(pc, *cloud);

		this->addBackground(cloud,background_mode_);
		this->has_background_ = true;
	}
}

void RosSceneHypothesisAssessor::addSceneCloud(const sensor_msgs::PointCloud2 &pc)
{
	std::cerr << "Adding scene points.\n";
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::fromROSMsg(pc, *cloud);
	if (!cloud->empty())
	{
		this->has_scene_cloud_ = true;
		this->mtx_.lock();
		this->addScenePointCloud(cloud);
		this->mtx_.unlock();
		this->scene_cloud_updated_ = true;
	}
	else
	{
		std::cerr << "Point Cloud input is empty.\n";
	}
	std::cerr << "Done\n";
}

void RosSceneHypothesisAssessor::updateSceneFromDetectedObjectMsgs(const costar_objrec_msgs::DetectedObjectList &detected_objects)
{
	while (!this->scene_cloud_updated_)
	{
		// wait for the scene cloud to be updated
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		std::cerr << "Waiting for scene cloud update.\n";
		return;
	}
	this->scene_cloud_updated_ = false;

	std::cerr << "Updating scene based on detected object message.\n";
	std::vector<ObjectWithID> objects;
	// allocate memory for all objects
	objects.reserve(detected_objects.objects.size());
	std::map<std::string, ObjectSymmetry> object_symmetry_map;

	this->parent_frame_ = detected_objects.header.frame_id;
	ros::Time now = ros::Time::now();

	// get and save the TF name and pose in the class variable
	for (unsigned int i = 0; i < detected_objects.objects.size(); i++) {
		std::string object_tf_frame = detected_objects.objects.at(i).id;
		if (this->listener_.waitForTransform(object_tf_frame,this->parent_frame_,now,ros::Duration(1.0)))
		{
			ObjectWithID obj_tmp;
			std::string object_class = detected_objects.objects.at(i).object_class;

			tf::StampedTransform transform;
			this->listener_.lookupTransform(this->parent_frame_,object_tf_frame,ros::Time(0),transform);

			// True if object_class exist in the database
			// Or the object that do not exist in the database successfully added
			if (this->obj_database_.objectExistInDatabase(object_class) ||
				this->obj_database_.addObjectToDatabase(object_class))
			{
				obj_tmp.assignPhysicalPropertyFromObject(this->obj_database_.getObjectProperty(object_class));
				btTransform bt = convertRosTFToBulletTF(transform);
				obj_tmp.assignData(object_tf_frame, bt, object_class);
				objects.push_back(obj_tmp);

				if (!keyExistInConstantMap(object_class, object_symmetry_map))
				{
					const costar_objrec_msgs::ObjectSymmetry &obj_sym_msg = detected_objects.objects.at(i).symmetry;
					object_symmetry_map[object_class] = ObjectSymmetry(obj_sym_msg.x_rotation, 
					obj_sym_msg.y_rotation, obj_sym_msg.z_rotation);
				}
			}
			else
			{
				std::cerr << "Fail. Object with class: " << object_class << " do not exists.\n";
			}
		}
		else std::cerr << "Fail to get: " << object_tf_frame << " transform to " << this->parent_frame_ << std::endl;
	}
	if (! this->physics_gravity_direction_set_)
	{
		std::cerr << "Gravity direction of the physics_engine is not set yet.\n";
		if (this->background_normal_as_gravity_)
		{
			std::cerr << "Setting gravity direction of the physics_engine.\n";
			this->physics_engine_.setGravityFromBackgroundNormal(true);
			this->physics_gravity_direction_set_ = true;
		}
		else if (this->listener_.waitForTransform(this->tf_z_is_inverse_gravity_direction_,
			this->parent_frame_,now,ros::Duration(1.0)))
		{
			std::cerr << "Setting gravity direction of the physics_engine.\n";
			tf::StampedTransform transform;
			this->listener_.lookupTransform(this->parent_frame_,
				this->tf_z_is_inverse_gravity_direction_,ros::Time(0),transform);
			btTransform bt = convertRosTFToBulletTF(transform);
			this->physics_engine_.setGravityVectorDirectionFromTfYUp(bt);
			this->physics_gravity_direction_set_ = true;
		}
		else
		{
			std::cerr << "Gravity direction is not set yet. No update will be done to the object TF\n";
			return;
		}
		// Do nothing if gravity has not been set and the direction cannot be found
	}

	this->mtx_.lock();
	this->addNewObjectTransforms(objects);
	this->setObjectSymmetryMap(object_symmetry_map);
	std::cerr << "Getting corrected object transform...\n";
	std::map<std::string, ObjectParameter> object_transforms = this->getCorrectedObjectTransform(best_hypothesis_only_);
	this->updateTfFromObjTransformMap(object_transforms);
	this->mtx_.unlock();
	
	this->has_tf_ = true;
	this->publishTf();

	this->object_list_updated_ = true;

	std::cerr << "Published tf with parent frame: "<< this->parent_frame_ << "\n";
	std::cerr << "Done. Waiting for new detected object message...\n";

}

void RosSceneHypothesisAssessor::updateTfFromObjTransformMap(const std::map<std::string, ObjectParameter> &input_tf_map)
{
	this->object_transforms_tf_.clear();
	for (std::map<std::string, ObjectParameter>::const_iterator it = input_tf_map.begin(); 
		it != input_tf_map.end(); ++it)
	{
		std::stringstream ss;
		tf::Transform tf_transform;
		tf_transform = convertBulletTFToRosTF(it->second);
		ss << this->tf_publisher_initial << "/" << it -> first;
		object_transforms_tf_[ss.str()] = tf_transform;
	}
}

void RosSceneHypothesisAssessor::publishTf()
{
	if (!this->has_tf_) return;

	for (std::map<std::string, tf::Transform>::const_iterator it = this->object_transforms_tf_.begin(); 
		it != this->object_transforms_tf_.end(); ++it)
	{
		this->tf_broadcaster_.sendTransform(tf::StampedTransform(it->second,ros::Time::now(),
			this->parent_frame_,it->first) );
	}
}

void RosSceneHypothesisAssessor::setDebugMode(bool debug)
{
	this->setDebug(debug);
	this->physics_engine_.setDebugMode(debug);
	this->obj_database_.setDebugMode(debug);
}

bool RosSceneHypothesisAssessor::fillObjectPropertyDatabase()
{
	if (! this->nh_.hasParam("object_property")) return false;
	XmlRpc::XmlRpcValue object_params;
	// std::map< std::string, std::string > object_params;
	this->nh_.getParam("object_property", object_params);
	for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = object_params.begin(); it != object_params.end(); ++it)
	{
		std::string object_name = it->first;
		std::cerr << "Found object property: " << object_name << std::endl;
		double mass, friction, rolling_friction;
		mass = object_params[it->first]["mass"];
		friction = object_params[it->first]["mass"];
		rolling_friction = object_params[it->first]["mass"];
		this->physical_properties_database_[object_name] = PhysicalProperties(mass,friction,rolling_friction);
	}
	return true;
}

void RosSceneHypothesisAssessor::fillObjectHypotheses(const objrec_hypothesis_msgs::AllModelHypothesis &detected_object_hypotheses)
{
	while (!this->object_list_updated_)
	{
		// wait for the scene cloud to be updated
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		std::cerr << "Waiting for object list update.\n";
		return;
	}
	this->object_list_updated_ = false;
	std::cerr << "Received input hypotheses list.\n";
	if (best_hypothesis_only_)
	{
		std::cerr << "Hypotheses is not processed since ros param best_hypothesis_only has been set to True.\n";
		return;
	}

	std::map<std::string, ObjectHypothesesData > object_hypotheses_map;
	for (unsigned int i = 0; i < detected_object_hypotheses.all_hypothesis.size(); i++)
	{
		const objrec_hypothesis_msgs::ModelHypothesis &model_hypo = detected_object_hypotheses.all_hypothesis[i];
		const std::string &object_tf_name = model_hypo.tf_name;
		const std::string &object_model_name = model_hypo.model_name;
		std::cerr << "Object '" << object_model_name << "' with id: " << object_tf_name << " hypotheses size:"
			<< model_hypo.model_hypothesis.size() << ".\n";
		std::vector<btTransform> object_pose_hypotheses;
		object_pose_hypotheses.reserve(model_hypo.model_hypothesis.size());
		for (unsigned int i = 0; i < model_hypo.model_hypothesis.size(); i++)
		{
			tf::Transform transform;
			tf::transformMsgToTF(model_hypo.model_hypothesis[i].transform, transform);
			double gl_matrix[16];
			transform.getOpenGLMatrix(gl_matrix);
			btTransform bt = convertRosTFToBulletTF(transform);
            object_pose_hypotheses.push_back(bt);
		}
		object_hypotheses_map[object_tf_name] = std::make_pair(object_model_name,object_pose_hypotheses);
	}
	this->mtx_.lock();
	this->setObjectHypothesesMap(object_hypotheses_map);

	while (!this->has_scene_cloud_)
	{
		std::cerr << "Waiting for input scene point cloud.\n";
		ros::Duration(0.5).sleep();
		return;
	}
	
	this->evaluateAllObjectHypothesisProbability();
	this->updateTfFromObjTransformMap(this->getCorrectedObjectTransformFromSceneGraph());

	this->publishTf();
	
	std_msgs::Empty done_msg;
	this->done_message_pub.publish(done_msg);

	this->mtx_.unlock();
}


