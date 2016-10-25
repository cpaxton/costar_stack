#include <pcl/io/pcd_io.h>
#include "ros_sequential_scene_parsing.h"

RosSceneGraph::RosSceneGraph()
{
	this->ros_scene_.setPhysicsEngine(&this->physics_engine_);
	this->class_ready_ = false;
	this->physics_gravity_direction_set_ = false;
	this->has_tf_ = false;
	this->exit_ = false;
}

RosSceneGraph::RosSceneGraph(const ros::NodeHandle &nh)
{
	this->ros_scene_.setPhysicsEngine(&this->physics_engine_);
	this->physics_gravity_direction_set_ = false;
	this->setNodeHandle(nh);
	this->exit_ = false;
}

void RosSceneGraph::callGlutMain(int argc, char* argv[])
{
	this->physics_engine_.renderingLaunched();
	glutmain(argc, argv,1024,600,"Scene Parsing Demo",&this->physics_engine_);
}

void RosSceneGraph::setNodeHandle(const ros::NodeHandle &nh)
{
	this->nh_ = nh;
	this->has_background_ = false;
	std::string detected_object_topic;
	std::string background_pcl2_topic;
	std::string object_folder_location;
	std::string background_location;

	bool debug_mode, load_table;

	nh.param("detected_object_topic", detected_object_topic,std::string("/detected_object"));
	nh.param("background_pcl2_topic", background_pcl2_topic,std::string("/background_points"));
	nh.param("object_folder_location",object_folder_location,std::string(""));
	nh.param("TF_z_inv_gravity_dir",this->tf_z_is_inverse_gravity_direction_,std::string(""));

	nh.param("tf_publisher_initial",this->tf_publisher_initial,std::string(""));
	nh.param("debug_mode",debug_mode,false);
	nh.param("load_table",load_table,false);
	if (load_table){
		nh.param("table_location",background_location,std::string(""));
		pcl::PCDReader reader;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr background_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
		if (reader.read(background_location,*background_cloud) == 0){
			this->ros_scene_.addBackground(background_cloud);
			std::cerr << "Background point loaded successfully\n";
		}
		else
			std::cerr << "Failed to load the background points\n"; 
	}
	


	std::cerr << "Debug mode: " << debug_mode << std::endl;
	this->setDebugMode(debug_mode);

	this->obj_database_.setObjectFolderLocation(object_folder_location);

	// sleep for caching the initial TF frames.
	sleep(1.0);
	this->detected_object_sub = this->nh_.subscribe(detected_object_topic,1,&RosSceneGraph::updateSceneFromDetectedObjectMsgs,this);
	this->background_pcl_sub = this->nh_.subscribe(background_pcl2_topic,1,&RosSceneGraph::addBackground,this);
	this->class_ready_ = true;
}

void RosSceneGraph::addBackground(const sensor_msgs::PointCloud2 &pc)
{
	if (!this->has_background_)
	{
		std::cerr << "Background points added to the scene.\n";
		// convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA::Ptr
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
		pcl::fromROSMsg(pc, *cloud);

		this->ros_scene_.addBackground(cloud);
		this->has_background_ = true;
	}
	this->exit_ = true;
}
	

void RosSceneGraph::updateSceneFromDetectedObjectMsgs(const costar_objrec_msgs::DetectedObjectList &detected_objects)
{
	std::cerr << "Updating scene based on detected object message.\n";
	std::vector<ObjectWithID> objects;
	// allocate memory for all objects
	objects.reserve(detected_objects.objects.size());

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
				double gl_matrix[15];
				float gl_matrix_f[15];
				transform.getOpenGLMatrix(gl_matrix);

				for (int i = 0; i < 15; i++) gl_matrix_f[i] = float(gl_matrix[i]);
				btTransform bt; bt.setFromOpenGLMatrix(gl_matrix_f);
				obj_tmp.assignData(object_tf_frame, bt);
				objects.push_back(obj_tmp);
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
		if (this->listener_.waitForTransform(this->tf_z_is_inverse_gravity_direction_,this->parent_frame_,now,ros::Duration(1.0)))
		{
			std::cerr << "Setting gravity direction of the physics_engine.\n";
			tf::StampedTransform transform;
			this->listener_.lookupTransform(this->parent_frame_,this->tf_z_is_inverse_gravity_direction_,ros::Time(0),transform);
			double gl_matrix[15];
			float gl_matrix_f[15];
			transform.getOpenGLMatrix(gl_matrix);

			for (int i = 0; i < 15; i++) gl_matrix_f[i] = float(gl_matrix[i]);
			btTransform bt; bt.setFromOpenGLMatrix(gl_matrix_f);
			this->physics_engine_.setGravityVectorDirectionFromTfYUp(bt);
			this->physics_gravity_direction_set_ = true;
		}
		// Do nothing if gravity has not been set and the direction cannot be found
		else return;
	}
	this->ros_scene_.addNewObjectTransforms(objects);
	std::cerr << "Getting corrected object transform...\n";
	this->object_transforms_.clear();
	this->object_transforms_ = this->ros_scene_.getCorrectedObjectTransform();

	this->object_transforms_tf_.clear();
	for (std::map<std::string, ObjectParameter>::const_iterator it = this->object_transforms_.begin(); 
		it != this->object_transforms_.end(); ++it)
	{
		std::stringstream ss;
		tf::Transform tf_transform;
		double gl_matrix[15];
		float gl_matrix_f[15];
		it->second.getOpenGLMatrix(gl_matrix_f);
		for (int i = 0; i < 15; i++) gl_matrix[i] = double(gl_matrix_f[i]);
		tf_transform.setFromOpenGLMatrix(gl_matrix);
		ss << this->tf_publisher_initial << "/" << it -> first;
		object_transforms_tf_[ss.str()] = tf_transform;
	}

	std::cerr << "Published tf with parent frame: "<< this->parent_frame_ << "\n";
	this->has_tf_ = true;
	this->publishTf();
	std::cerr << "Done. Waiting for new detected object message...\n";
	this->exit_ = true;
}

void RosSceneGraph::publishTf()
{
	if (!this->has_tf_) return;

	for (std::map<std::string, tf::Transform>::const_iterator it = this->object_transforms_tf_.begin(); 
		it != this->object_transforms_tf_.end(); ++it)
	{
		this->tf_broadcaster_.sendTransform(tf::StampedTransform(it->second,ros::Time::now(),this->parent_frame_,it->first) );
	}
}

void RosSceneGraph::setDebugMode(bool debug)
{
	this->debug_messages_ = debug;
	this->ros_scene_.setDebugMode(debug);
	this->physics_engine_.setDebugMode(debug);
	this->obj_database_.setDebugMode(debug);
}