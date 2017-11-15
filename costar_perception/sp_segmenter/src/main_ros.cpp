#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sp_segmenter/features.h"
#include "sp_segmenter/JHUDataParser.h"
#include "sp_segmenter/greedyObjRansac.h"
#include "sp_segmenter/plane.h"
#include "sp_segmenter/refinePoses.h"
#include "sp_segmenter/common.h"
#include "sp_segmenter/stringVectorArgsReader.h"
#include "sp_segmenter/tracker.h"

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// chi objrec ransac utils
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

// contains function for spatial data structure that also normalize the orientation of pose
#include "sp_segmenter/spatial_pose.h"

#define OBJECT_MAX 100


bool hasTF;
SpatialPose sp_segmenter_poses;

// for orientation normalization
std::map<std::string, ObjectSymmetry> objectDict;
std::map<std::string, unsigned int> objectTFIndex; // keep information about TF index

bool compute_pose = true;
bool view_flag = false;
pcl::visualization::PCLVisualizer::Ptr viewer;
Hier_Pooler hie_producer;
std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
std::vector<model*> binary_models(3);
float zmax = 1.2;
float radius = 0.02;
float down_ss = 0.003;
double aboveTable;
double pairWidth = 0.05;
double voxelSize = 0.003; 
bool bestPoseOnly;
double minConfidence;
bool enableTracking;

boost::shared_ptr<Tracker> tracker;
    
boost::shared_ptr<greedyObjRansac> objrec;
std::vector<std::string> model_name(OBJECT_MAX, "");
std::vector<ModelT> mesh_set;

std::string POINTS_IN, POINTS_OUT, POSES_OUT;

ros::Publisher pc_pub;
ros::Publisher pose_pub;
ros::Subscriber pc_sub;


sensor_msgs::CameraInfo cam_info;

std::map<std::string, int> model_name_map;
uchar color_label[11][3] = 
{ {0, 0, 0}, 
  {255, 0, 0},
  {0, 255, 0},
  {0, 0, 255},
  {255, 255, 0},
  {255, 0, 255},
  {0, 255, 255},
  {255, 128, 0},
  {255, 0, 128},
  {0, 128, 255},
  {128, 0, 255},
};   

void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3]);
pcl::PointCloud<PointLT>::Ptr densifyLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, const pcl::PointCloud<PointT>::Ptr ori_cloud);


/*********************************************************************************/



void callback(const sensor_msgs::PointCloud2 &pc) {

	pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());

	//fromROSMsg(pc,*full_cloud); // convert to PCL format
	pcl::PCLPointCloud2 pcl_pc;
	pcl_conversions::toPCL(pc, pcl_pc);

	pcl::fromPCLPointCloud2(pcl_pc, *full_cloud);

	// remove NaNs
	std::vector<int> idx_ff;

	pcl::removeNaNFromPointCloud(*full_cloud, *cloud, idx_ff);

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.1, zmax);
	//pass.setFilterLimitsNegative (true);
	pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>());
	pass.filter (*scene);

	pcl::PointCloud<PointT>::Ptr scene_f = removePlane(scene, aboveTable);

	//computeNormals(cloud, cloud_normal, radius);
	//pcl::PointCloud<PointT>::Ptr label_cloud = recog.recognize(cloud, cloud_normal);

	spPooler triple_pooler;
	triple_pooler.lightInit(scene_f, hie_producer, radius, down_ss);
	std::cerr << "LAB Pooling!" << std::endl;
	triple_pooler.build_SP_LAB(lab_pooler_set, false);

  if(viewer)
  {
	  viewer->removeAllPointClouds();
  }
	pcl::PointCloud<PointLT>::Ptr foreground_cloud(new pcl::PointCloud<PointLT>());
	for( int ll = 0 ; ll <= 2 ; ll++ )
	{
		std::cerr << "L" << ll <<" Inference!" << std::endl;
		bool reset_flag = ll == 0 ? true : false;
		if( ll >= 1 ) {
			triple_pooler.extractForeground(false);
		}
		triple_pooler.InputSemantics(binary_models[ll], ll, reset_flag, false);

	}

	foreground_cloud = triple_pooler.getSemanticLabels();
	if( viewer )
	{
		//viewer->addPointCloud(final_cloud, "labels");
		visualizeLabels(foreground_cloud, viewer, color_label);
		viewer->spin();
	}

	pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
	for(  size_t l = 0 ; l < foreground_cloud->size() ;l++ )
	{
		if( foreground_cloud->at(l).label > 0 )
			final_cloud->push_back(foreground_cloud->at(l));
	}

	sensor_msgs::PointCloud2 output_msg;
	toROSMsg(*final_cloud,output_msg);
	output_msg.header.frame_id = pc.header.frame_id;
	pc_pub.publish(output_msg);
  

	/* POSE */
	if (compute_pose) {

    ROS_INFO("Computing poses");
		geometry_msgs::PoseArray msg;
		msg.header.frame_id = pc.header.frame_id;
		for (int i = 0; i < 1; ++i) { // was 99
			std::vector<poseT> all_poses1;

			pcl::PointCloud<myPointXYZ>::Ptr foreground(new pcl::PointCloud<myPointXYZ>());
			pcl::copyPointCloud(*foreground_cloud, *foreground);

            if (bestPoseOnly)
                objrec->StandardBest(foreground, all_poses1);
            else
			    objrec->StandardRecognize(foreground, all_poses1,minConfidence);

			pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
			pcl::copyPointCloud(*scene_f, *scene_xyz);

			std::vector<poseT> all_poses =  RefinePoses(scene_xyz, mesh_set, all_poses1);
			std::cout << "# Poses found: " << all_poses.size() << std::endl;
            
            // normalize symmetric object Orientation
            if (!hasTF) sp_segmenter_poses.createTree(objectDict, all_poses, ros::Time::now().toSec(), objectTFIndex);
            else sp_segmenter_poses.updateTree(all_poses, ros::Time::now().toSec(), objectTFIndex);
            hasTF = true;
            
            all_poses.clear();
            all_poses = sp_segmenter_poses.getAllPoses();

			for (poseT &p: all_poses) {
				geometry_msgs::Pose pmsg;
				std::cout <<"pose = ("<<p.shift.x()<<","<<p.shift.y()<<","<<p.shift.z()<<")"<<std::endl;
				pmsg.position.x = p.shift.x();
				pmsg.position.y = p.shift.y();
				pmsg.position.z = p.shift.z();
				pmsg.orientation.x = p.rotation.x();
				pmsg.orientation.y = p.rotation.y();
				pmsg.orientation.z = p.rotation.z();
				pmsg.orientation.w = p.rotation.w();

				msg.poses.push_back(pmsg);
			}
			std::cout<<"done objrec ransac"<<std::endl;

      if(enableTracking)
      {
        tracker->generateTrackingPoints(pc.header.stamp, all_poses);
      }

			if( viewer )
			{
				std::cout<<"VISUALIZING"<<std::endl;
				viewer->removeAllPointClouds();
				viewer->addPointCloud(scene_f, "whole_scene");
				objrec->visualize_m(viewer, all_poses, model_name_map, color_label);
				viewer->spin();
				objrec->clearMesh(viewer, all_poses);
				viewer->removeAllPointClouds();
			}


		}

		pose_pub.publish(msg);

	}

}

int main(int argc, char** argv)
{
    //std::string in_path("/home/chi/Downloads/");
    //std::string scene_name("UR5_1");

    ros::init(argc,argv,"sp_segmenter_node");    
    ros::NodeHandle nh("~");
    //getting subscriber/publisher parameters
    nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
    nh.param("POINTS_OUT", POINTS_OUT,std::string("points_out"));
    nh.param("POSES_OUT", POSES_OUT,std::string("poses_out"));
    //get only best poses (1 pose output) or multiple poses
    nh.param("bestPoseOnly", bestPoseOnly, true);
    nh.param("minConfidence", minConfidence, 0.0);
    nh.param("aboveTable", aboveTable, 0.01);
    hasTF = false;
    nh.param("enableTracking", enableTracking, false);

    
    bool use_cuda;
    nh.param("use_cuda", use_cuda,true);

    if (bestPoseOnly)
        std::cerr << "Node will only output the best detected poses \n";
    else
        std::cerr << "Node will output all detected poses \n";

    pc_sub = nh.subscribe(POINTS_IN,1,callback);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);
    nh.param("pairWidth", pairWidth, 0.05);
    pose_pub = nh.advertise<geometry_msgs::PoseArray>(POSES_OUT,1000);
 

    objrec = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(pairWidth, voxelSize));
    objrec->setUseCUDA(use_cuda);
    //pcl::console::parse_argument(argc, argv, "--p", in_path);
    //pcl::console::parse_argument(argc, argv, "--i", scene_name);
    
    //in_path = in_path + scene_name + "/";
    
    //std::string out_cloud_path("../../data_pool/IROS_demo/UR5_1/");
    //pcl::console::parse_argument(argc, argv, "--o", out_cloud_path);
    //boost::filesystem::create_directories(out_cloud_path);
    
    std::string svm_path,shot_path;
    //get path parameter for svm and shot
    nh.param("svm_path", svm_path,std::string("data/UR5_drill_svm/"));
    nh.param("shot_path", shot_path,std::string("data/UW_shot_dict/"));
//    std::string sift_path("UW_new_sift_dict/");
//    std::string fpfh_path("UW_fpfh_dict/");
/***************************************************************************************************************/
    
    float ratio = 0.1;
    
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
    pcl::console::parse_argument(argc, argv, "--zmax", zmax);
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::string mesh_path;
    //get parameter for mesh path and curname
    nh.param("mesh_path", mesh_path,std::string("data/mesh/"));
    std::vector<std::string> cur_name = stringVectorArgsReader(nh, "cur_name", std::string("drill"));
    
    //get symmetry parameter of the objects
    objectDict = fillDictionary(nh, cur_name);

    for (int model_id = 0; model_id < cur_name.size(); model_id++)
    {
    	// add all models. model_id starts in model_name start from 1.
    	std::string temp_cur = cur_name.at(model_id);
    	objrec->AddModel(mesh_path + temp_cur, temp_cur);
        model_name[model_id+1] = temp_cur;
        model_name_map[temp_cur] = model_id+1;
        ModelT mesh_buf = LoadMesh(mesh_path + temp_cur + ".obj", temp_cur);
        objectTFIndex[temp_cur] = 0;
        mesh_set.push_back(mesh_buf);
    }
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    if( pcl::console::find_switch(argc, argv, "-p") == true )
        compute_pose = true;

/***************************************************************************************************************/
    hie_producer = Hier_Pooler(radius);
    hie_producer.LoadDict_L0(shot_path, "200", "200");
    hie_producer.setRatio(ratio);
/***************************************************************************************************************/    
//    std::vector<cv::SiftFeatureDetector*> sift_det_vec;
//    for( float sigma = 0.7 ; sigma <= 1.61 ; sigma += 0.1 )
//    {	
//        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
//        	0, // nFeatures
//        	4, // nOctaveLayers
//        	-10000, // contrastThreshold 
//        	100000, //edgeThreshold
//        	sigma//sigma
//        	);
//        sift_det_vec.push_back(sift_det);	
//    }
/***************************************************************************************************************/
//    std::vector< boost::shared_ptr<Pooler_L0> > sift_pooler_set(1+1);
//    for( size_t i = 1 ; i < sift_pooler_set.size() ; i++ )
//    {
//        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
//        sift_pooler_set[i] = cur_pooler;
//    }
//    sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_400.cvmat"); 
/***************************************************************************************************************/
//    std::vector< boost::shared_ptr<Pooler_L0> > fpfh_pooler_set(1+1);
//    for( size_t i = 1 ; i < fpfh_pooler_set.size() ; i++ )
//    {
//        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0(-1));
//        fpfh_pooler_set[i] = cur_pooler;
//    }
//    fpfh_pooler_set[1]->LoadSeedsPool(fpfh_path+"dict_fpfh_L0_400.cvmat");
/***************************************************************************************************************/
    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        lab_pooler_set[i] = cur_pooler;
    }
/***************************************************************************************************************/
    for( int ll = 0 ; ll <= 2 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;
        
        binary_models[ll] = load_model((svm_path+"binary_L"+ss.str()+"_f.model").c_str());
    }
    
/***************************************************************************************************************/
    
    if( view_flag )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
    }

    if(enableTracking)
    {
      tracker = boost::shared_ptr<Tracker>(new Tracker());
      for(ModelT& model : mesh_set)
      {
         
        if(!tracker->addTracker(model))
        {
          ROS_ERROR("Tried to add duplicate model name to tracker");
          return -1;
        }
      }
    }

    ros::spin();
    
    for( int ll = 0 ; ll <= 2 ; ll++ )
        free_and_destroy_model(&binary_models[ll]);

//    for( int ll = 1 ; ll <= 4 ; ll++ )
//        free_and_destroy_model(&multi_models[ll]);
    
    return 1;
} 
