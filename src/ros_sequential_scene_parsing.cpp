#include "ros_sequential_scene_parsing.h"

RosSceneGraph::RosSceneGraph()
{
	this->ros_scene_.setPhysicsEngine(&this->physics_engine_);
	this->class_ready_ = false;
	this->physics_gravity_direction_set_ = false;
}

RosSceneGraph::RosSceneGraph(const ros::NodeHandle &nh)
{
	this->ros_scene_.setPhysicsEngine(&this->physics_engine_);
	this->physics_gravity_direction_set_ = false;
	this->setNodeHandle(nh);
}

void RosSceneGraph::setNodeHandle(const ros::NodeHandle &nh)
{
	this->nh_ = nh;

	std::string detected_object_topic;
	std::string background_pcl2_topic;
	std::string object_folder_location;

	nh.param("detected_object_topic", detected_object_topic,std::string("/detected_object"));
	nh.param("background_pcl2_topic", background_pcl2_topic,std::string("/background_points"));
	nh.param("object_folder_location",object_folder_location,std::string(""));
	nh.param("TF_y_inv_gravity_dir",this->tf_y_is_inverse_gravity_direction_,std::string(""));

	nh.param("tf_publisher_initial",this->tf_publisher_initial,std::string(""));

	this->obj_database_.setObjectDatabaseLocation(object_folder_location);

	// sleep for caching the initial TF frames.
	sleep(1.0);
	this->detected_object_sub = this->nh_.subscribe(detected_object_topic,1,&RosSceneGraph::updateSceneFromDetectedObjectMsgs,this);
	this->background_pcl_sub = this->nh_.subscribe(background_pcl2_topic,1,&RosSceneGraph::addBackground,this);
	this->class_ready_ = true;
}

void RosSceneGraph::addBackground(const sensor_msgs::PointCloud2 &pc)
{
	// convert sensor_msgs::PointCloud2 to pcl::PointXYZRGBA::Ptr
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::fromROSMsg(pc, *cloud);

	this->ros_scene_.addBackground(cloud);
}

void RosSceneGraph::updateSceneFromDetectedObjectMsgs(const costar_objrec_msgs::DetectedObjectList &detected_objects)
{
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
				obj_tmp.assignData(object_tf_frame,bt );
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
		if (this->listener_.waitForTransform(this->tf_y_is_inverse_gravity_direction_,this->parent_frame_,now,ros::Duration(1.0)))
		{
			tf::StampedTransform transform;
			this->listener_.lookupTransform(this->parent_frame_,this->tf_y_is_inverse_gravity_direction_,ros::Time(0),transform);
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
	this->object_transforms_ = this->ros_scene_.getCorrectedObjectTransform();
}

void RosSceneGraph::publishTf() const
{
	for (std::map<std::string, ObjectParameter>::const_iterator it = this->object_transforms_.begin(); 
		it != this->object_transforms_.end(); ++it)
	{
		tf::Transform tf_transform;
		double gl_matrix[15];
		float gl_matrix_f[15];
		it->second.getOpenGLMatrix(gl_matrix_f);
		for (int i = 0; i < 15; i++) gl_matrix[i] = double(gl_matrix_f[i]);
		tf_transform.setFromOpenGLMatrix(gl_matrix);
		
		tf::TransformBroadcaster br;
		std::stringstream ss;
		// the name of tf published by this node is: initial/original_object_tf_name
		ss << this->tf_publisher_initial << "/" << it->first;
		br.sendTransform(tf::StampedTransform(tf_transform,ros::Time::now(),this->parent_frame_,ss.str()) );
	}
}

