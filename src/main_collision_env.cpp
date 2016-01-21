#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <shape_msgs/Mesh.h>
#include <moveit_msgs/CollisionObject.h>

class collision_environment
{
	protected:
		ros::NodeHandle nh;
		ros::Publisher add_collision_object_publisher;
		tf::TransformListener listener;
	;

	private:
		std::string listOfTF;
		std::vector<std::string> detectedObjectsTF;
		std::string meshDir;
	;

	public:
		collision_environment()
		{
			add_collision_object_publisher = nh.advertise<moveit_msgs::CollisionObject>("collision_objects",1000);
		};

		void getAllObjectTF()
		{
			// get the TF list
			listOfTF = listener.allFramesAsString();
			

			// filter the TF list to grab the object list
		};

		void addAllCollisionObject()
		{
			// update list of object
			getAllObjectTF();
			std::cerr << "List of all frames:\n" << listOfTF << std::endl;


			// load the mesh of all detected object

			// add the collision object based on TF position and mesh
			
		};


};

int main(int argc, char** argv)
{
	// initialize ros node
	ros::init(argc,argv, "collision_environment");
	std::cerr << "Started \n";

	collision_environment environment;

	environment.addAllCollisionObject();
	ros::spin();
	
	// load all mesh
	return 1;
}
