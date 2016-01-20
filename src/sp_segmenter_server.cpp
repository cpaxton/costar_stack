#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/features.h"
#include "../include/JHUDataParser.h"
#include "greedyObjRansac.h"

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// for TF services
#include <std_srvs/Empty.h>
// #include <tf/tf_conversions.h>
#include <tf/transform_broadcaster.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// chi objrec ransac utils
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

#define OBJECT_MAX 100

//for TF
bool hasTF; 
geometry_msgs::PoseArray sp_segmenter_poses;
sensor_msgs::PointCloud2 inputCloud;

bool compute_pose = false;
bool view_flag = false;
pcl::visualization::PCLVisualizer::Ptr viewer;
Hier_Pooler hie_producer;
std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
std::vector<model*> binary_models(3);
float zmax = 1.2;
float radius = 0.02;
float down_ss = 0.003;
double pairWidth = 0.05;
double voxelSize = 0.003; 
bool bestPoseOnly;
    
boost::shared_ptr<greedyObjRansac> objrec;
std::vector<std::string> model_name(OBJECT_MAX, "");
std::vector<ModelT> mesh_set;

std::string POINTS_IN, POINTS_OUT, POSES_OUT;

ros::Publisher pc_pub;
// ros::Publisher pose_pub;
ros::Subscriber pc_sub;

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

pcl::PointCloud<PointT>::Ptr removePlane(const pcl::PointCloud<PointT>::Ptr scene, float T = 0.02)
{
    pcl::PointCloud<PointT>::Ptr downsampledPointCloud (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (scene);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*downsampledPointCloud); 

    pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(T);

    seg.setInputCloud(downsampledPointCloud);
    seg.segment(*inliers, *plane_coef);
    
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (scene);
    proj.setModelCoefficients (plane_coef);

    pcl::PointCloud<PointT>::Ptr scene_projected(new pcl::PointCloud<PointT>());
    proj.filter (*scene_projected);

    pcl::PointCloud<PointT>::iterator it_ori = scene->begin();
    pcl::PointCloud<PointT>::iterator it_proj = scene_projected->begin();
    
    pcl::PointCloud<PointT>::Ptr scene_f(new pcl::PointCloud<PointT>());
    for( int base = 0 ; it_ori < scene->end(), it_proj < scene_projected->end() ; it_ori++, it_proj++, base++ )
    {
        float diffx = it_ori->x-it_proj->x;
        float diffy = it_ori->y-it_proj->y;
        float diffz = it_ori->z-it_proj->z;

        if( diffx * it_ori->x + diffy * it_ori->y + diffz * it_ori->z >= 0 )
            continue;
        //distance from the point to the plane
        float dist = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
        
        if ( dist >= T+0.005 )//fabs((*it_ori).x) <= 0.1 && fabs((*it_ori).y) <= 0.1 )
            scene_f->push_back(*it_ori);
    }
    
    return scene_f;
}

/*********************************************************************************/
std::vector<poseT> RefinePoses(const pcl::PointCloud<myPointXYZ>::Ptr scene, const std::vector<ModelT> &mesh_set, const std::vector<poseT> &all_poses)
{
    int pose_num = all_poses.size();
    std::vector<ModelT> est_models(pose_num);
    pcl::PointCloud<myPointXYZ>::Ptr down_scene(new pcl::PointCloud<myPointXYZ>());
    pcl::VoxelGrid<myPointXYZ> sor;
    sor.setInputCloud(scene);
    sor.setLeafSize(0.005, 0.005, 0.005);
    sor.filter(*down_scene);
    
    #pragma omp parallel for schedule(dynamic, 1)
    for(int i = 0 ; i < pose_num ; i++ ){
        for( int j = 0 ; j < mesh_set.size() ; j++ ){
            if( mesh_set[j].model_label == all_poses[i].model_name )
            {
                est_models[i].model_label = all_poses[i].model_name;
                est_models[i].model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
                pcl::transformPointCloud(*mesh_set[j].model_cloud, *est_models[i].model_cloud, all_poses[i].shift, all_poses[i].rotation);
                break;
            }
        } 
    }
    
    std::vector< pcl::search::KdTree<myPointXYZ>::Ptr > tree_set(est_models.size());
    #pragma omp parallel for schedule(dynamic, 1)
    for( int i = 0 ; i < pose_num ; i++ )
    {
        tree_set[i] = pcl::search::KdTree<myPointXYZ>::Ptr (new pcl::search::KdTree<myPointXYZ>());
        tree_set[i]->setInputCloud(est_models[i].model_cloud);
    }   
    
    std::vector<int> votes(pose_num, 0);
    std::vector< std::vector<int> > adj_graph(pose_num);
    for( int i = 0 ; i < pose_num ; i++ )
        adj_graph[i].resize(pose_num, 0);
    float sqrT = 0.01*0.01;
    int down_num = down_scene->size();
    
    std::vector< std::vector<int> > bin_vec(down_num);
    #pragma omp parallel for
    for(int i = 0 ; i < pose_num ; i++ )
    {
        int count = 0;
        for( pcl::PointCloud<myPointXYZ>::const_iterator it = down_scene->begin() ; it < down_scene->end() ; it++, count++ )
        {
            std::vector<int> idx (1);
            std::vector<float> sqrDist (1);
            int nres = tree_set[i]->nearestKSearch(*it, 1, idx, sqrDist);
            if ( nres >= 1 && sqrDist[0] <= sqrT )
            {
                #pragma omp critical
                {   
                    bin_vec[count].push_back(i);
                }
                votes[i]++;
            }
        }
    }
    
    for( int it = 0 ; it < down_num ; it++ )
        for( std::vector<int>::iterator ii = bin_vec[it].begin() ; ii < bin_vec[it].end() ; ii++ )
            for( std::vector<int>::iterator jj = ii+1 ; jj < bin_vec[it].end() ; jj++ )
            {
                adj_graph[*ii][*jj]++;
                adj_graph[*jj][*ii]++;
            }
    std::vector<bool> dead_flag(pose_num, 0);
    for( int i = 0 ; i < pose_num ; i++ ){
        if( dead_flag[i] == true )
            continue;
        for( int j = i+1 ; j < pose_num ; j++ )
        {
            if( dead_flag[j] == true )
                continue;
            int min_tmp = std::min(votes[i], votes[j]);
            if( (adj_graph[i][j]+0.0) / min_tmp >= 0.3 )
            {
                std::cerr << votes[i] << " " << i << std::endl;
                std::cerr << votes[j] << " " << j << std::endl;
                if( votes[i] > votes[j] )
                    dead_flag[j] = true;
                else
                {
                    dead_flag[i] = true;
                    break;
                }
            }
        }
    }
    std::vector<poseT> refined_poses;
    for( int i = 0 ; i < pose_num ; i++ )   
        if( dead_flag[i] == false )
            refined_poses.push_back(all_poses[i]);
    
    return refined_poses;
}
/*********************************************************************************/

std::vector<poseT> spSegmenterCallback(
	const pcl::PointCloud<PointT>::Ptr full_cloud, pcl::PointCloud<PointLT> final_cloud)
{
	// By default pcl::PassThrough will remove NaNs point unless setKeepOrganized is true
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (full_cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.1, zmax);
	//pass.setFilterLimitsNegative (true);
	pcl::PointCloud<PointT>::Ptr scene(new pcl::PointCloud<PointT>());
	pass.filter (*scene);

	pcl::PointCloud<PointT>::Ptr scene_f = removePlane(scene);

	// pcl::PointCloud<NormalT>::Ptr cloud_normal(new pcl::PointCloud<NormalT>());
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

	
	for(  size_t l = 0 ; l < foreground_cloud->size() ;l++ )
	{
		if( foreground_cloud->at(l).label > 0 )
			final_cloud.push_back(foreground_cloud->at(l));
	}

	std::vector<poseT> all_poses;
	/* POSE */
	if (compute_pose) {
		for (int i = 0; i < 1; ++i) { // was 99
			std::vector<poseT> all_poses1;

			pcl::PointCloud<myPointXYZ>::Ptr foreground(new pcl::PointCloud<myPointXYZ>());
			pcl::copyPointCloud(*foreground_cloud, *foreground);

            if (bestPoseOnly)
                objrec->StandardBest(foreground, all_poses1);
            else
			    objrec->StandardRecognize(foreground, all_poses1);

			pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
			pcl::copyPointCloud(*scene_f, *scene_xyz);

			all_poses =  RefinePoses(scene_xyz, mesh_set, all_poses1);
			std::cout << "# Poses found: " << all_poses.size() << std::endl;

			std::cout<<"done objrec ransac"<<std::endl;
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

	}
	return all_poses;

}

std::vector<std::string> stringVectorArgsReader (ros::NodeHandle nh, 
    const std::string param_name, const std::string default_value)
{
// The function will read string arguments passed from nodehandle, 
// separate it for every ',' character, and return it to vector<string> variable
	std::string tmp;
	std::vector<std::string> result;
	nh.param(param_name,tmp,default_value);

	char splitOperator(',');
	std::size_t found = tmp.find(splitOperator);
	std::size_t pos = 0;

	while (found!=std::string::npos)
	{
		std::string buffer;
		buffer.assign(tmp, pos, found - pos);
		result.push_back(buffer);
		pos = found + 1;
		found = tmp.find(splitOperator,pos);
	}

	std::string buffer;
	buffer.assign(tmp, pos, tmp.length() - pos);
	result.push_back(buffer);
	return result;
}

void updateCloudData (const sensor_msgs::PointCloud2 &pc)
{
    // The callback from main only update the cloud data
    inputCloud = pc;
}

bool serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    // Service call will run SPSegmenter
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());

	fromROSMsg(inputCloud,*full_cloud); // convert to PCL format

	// get all poses from spSegmenterCallback
    std::vector<poseT> all_poses = spSegmenterCallback(full_cloud,*final_cloud);

    //publishing the segmented point cloud
	sensor_msgs::PointCloud2 output_msg;
	toROSMsg(*final_cloud,output_msg);
	output_msg.header.frame_id = inputCloud.header.frame_id;
	pc_pub.publish(output_msg);

	//converting all_poses to geometry_msgs::Pose and update the sp_segmenter_poses

	geometry_msgs::PoseArray msg;
	msg.header.frame_id = inputCloud.header.frame_id;
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
    sp_segmenter_poses = msg;

    hasTF = true;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"sp_segmenter_server");    
    ros::NodeHandle nh("~");
    ros::Rate r(10); //10Hz
    ros::ServiceServer spSegmenter = nh.advertiseService("SPSegmenter",serviceCallback);
    hasTF = false;
    static tf::TransformBroadcaster br;


// From SPSegmenter main Starts

    //std::string in_path("/home/chi/Downloads/");
    //std::string scene_name("UR5_1");

    //getting subscriber/publisher parameters
    nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
    nh.param("POINTS_OUT", POINTS_OUT,std::string("points_out"));
    nh.param("POSES_OUT", POSES_OUT,std::string("poses_out"));
    //get only best poses (1 pose output) or multiple poses
    nh.param("bestPoseOnly", bestPoseOnly, true);

    if (bestPoseOnly)
        std::cerr << "Node will only output the best detected poses \n";
    else
        std::cerr << "Node will output all detected poses \n";

    pc_sub = nh.subscribe(POINTS_IN,1,updateCloudData);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);
    nh.param("pairWidth", pairWidth, 0.05d);
    // pose_pub = nh.advertise<geometry_msgs::PoseArray>(POSES_OUT,1000);

    objrec = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(pairWidth, voxelSize));
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
    
    for (int model_id = 0; model_id < cur_name.size(); model_id++)
    {
        // add all models. model_id starts in model_name start from 1.
        std::string temp_cur = cur_name.at(model_id);
        objrec->AddModel(mesh_path + temp_cur, temp_cur);
        model_name[model_id+1] = temp_cur;
        model_name_map[temp_cur] = model_id+1;
        ModelT mesh_buf = LoadMesh(mesh_path + temp_cur + ".obj", temp_cur);
        
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
//          0, // nFeatures
//          4, // nOctaveLayers
//          -10000, // contrastThreshold 
//          100000, //edgeThreshold
//          sigma//sigma
//          );
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
// From SPSegmenter ENDs

    while (ros::ok())
    {
        if (hasTF)
        {
            // broadcast all transform
            std::string parent = sp_segmenter_poses.header.frame_id;
            for (size_t i = 0; i < sp_segmenter_poses.poses.size(); i++)
            {
                geometry_msgs::Pose pose_i = sp_segmenter_poses.poses[i];
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(pose_i.position.x,pose_i.position.y,pose_i.position.z));
                transform.setRotation(tf::Quaternion(
                    pose_i.orientation.x,pose_i.orientation.y,pose_i.orientation.z,pose_i.orientation.w));
                std::stringstream child;
                // Does not have tracking yet, can not keep the label on object.
                child << "Object no " << i;

                br.sendTransform(
                    tf::StampedTransform(
                        transform,ros::Time::now(),sp_segmenter_poses.header.frame_id, child.str())
                    );
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    for( int ll = 0 ; ll <= 2 ; ll++ )
        free_and_destroy_model(&binary_models[ll]);

    return 1;
}

void visualizeLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, pcl::visualization::PCLVisualizer::Ptr viewer, uchar colors[][3])
{
    pcl::PointCloud<PointT>::Ptr view_cloud(new pcl::PointCloud<PointT>());
    for( pcl::PointCloud<PointLT>::const_iterator it = label_cloud->begin() ; it < label_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        
        PointT pt;
        pt.x = it->x;
        pt.y = it->y;
        pt.z = it->z;
        pt.rgba = colors[it->label][0] << 16 | colors[it->label][1] << 8 | colors[it->label][2];
        view_cloud->push_back(pt);
    }
    
    if(viewer)
    {
      viewer->removePointCloud("label_cloud");
      viewer->addPointCloud(view_cloud, "label_cloud");
      //viewer->spinOnce(1);
      viewer->spin();
    }
}

pcl::PointCloud<PointLT>::Ptr densifyLabels(const pcl::PointCloud<PointLT>::Ptr label_cloud, const pcl::PointCloud<PointT>::Ptr ori_cloud)
{
    pcl::search::KdTree<PointLT> tree;
    tree.setInputCloud(label_cloud);
    
    pcl::PointCloud<PointLT>::Ptr dense_cloud(new pcl::PointCloud<PointLT>());
    
    float T = 0.01;
    float sqrT = T*T;
    for( pcl::PointCloud<PointT>::const_iterator it = ori_cloud->begin() ; it < ori_cloud->end() ; it++ )
    {
        if( pcl_isfinite(it->z) == false )
            continue;
        PointLT tmp;
        tmp.x = it->x;
        tmp.y = it->y;
        tmp.z = it->z;
       
        std::vector<int> ind (1);
	std::vector<float> sqr_dist (1);
        int nres = tree.nearestKSearch(tmp, 1, ind, sqr_dist);
        if ( nres >= 1 && sqr_dist[0] <= sqrT )
        {
            tmp.label = label_cloud->at(ind[0]).label;
            dense_cloud->push_back(tmp);
        }   
    }
    return dense_cloud;
}
