#include "ros_sequential_scene_parsing.h"

RosSceneGraph::RosSceneGraph()
{
	this->ros_scene_.setPhysicsEngine(&this->physics_engine_);
	this->class_ready_ = false;
}

RosSceneGraph::RosSceneGraph(const ros::NodeHandle &nh)
{
	this->ros_scene_.setPhysicsEngine(&this->physics_engine_);
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

	std::string parent_frame = detected_objects.header.frame_id;
	ros::Time now = ros::Time::now();

	// get and save the TF name and pose in the class variable
	for (unsigned int i = 0; i < detected_objects.objects.size(); i++) {
		std::string object_tf_frame = detected_objects.objects.at(i).id;
		if (this->listener_.waitForTransform(object_tf_frame,parent_frame,now,ros::Duration(1.0)))
		{
			ObjectWithID obj_tmp;
			std::string object_class = detected_objects.objects.at(i).object_class;

			tf::StampedTransform transform;
			this->listener_.lookupTransform(parent_frame,object_tf_frame,ros::Time(0),transform);

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
        else std::cerr << "Fail to get: " << object_tf_frame << " transform to " << parent_frame << std::endl;
    }

	this->ros_scene_.addNewObjectTransforms(objects);
	this->object_transforms_ = this->ros_scene_.getCorrectedObjectTransform();
}