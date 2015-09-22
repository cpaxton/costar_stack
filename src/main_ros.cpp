#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../include/features.h"
#include "../include/JHUDataParser.h"

// ros stuff
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// include to convert from messages to pointclouds and vice versa
#include <pcl_conversions/pcl_conversions.h>

// chi objrec ransac utils
#include <greedy/utility.h>
#include <eigen3/Eigen/src/Geometry/Quaternion.h>

bool view_flag = false;
pcl::visualization::PCLVisualizer::Ptr viewer;
Hier_Pooler hie_producer;
std::vector< boost::shared_ptr<Pooler_L0> > lab_pooler_set(5+1);
std::vector<model*> binary_models(3);
float zmax = 1.2;
float radius = 0.02;
float down_ss = 0.003;

std::string POINTS_IN("/camera_2/depth_registered/points");
std::string POINTS_OUT("points_out");

ros::Publisher pc_pub;
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
    pcl::ModelCoefficients::Ptr plane_coef(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(T);

    seg.setInputCloud(scene);
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
/*********************************************************************************/
void showPoses(const std::vector<ModelT> &model_set, const std::vector<poseT> &poses, pcl::visualization::PCLVisualizer::Ptr viewer, bool adjust = true)
{
  std::vector<pcl::PointCloud<myPointXYZ>::Ptr> rec;
  for( std::vector<ModelT>::const_iterator it = model_set.begin() ; it < model_set.end() ; it++ )
  {
    pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
    pcl::fromPCLPointCloud2(it->model_mesh->cloud, *cur_cloud);
    rec.push_back(cur_cloud);
  }

  int count = 0;
  Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f (1, 0, 0)));
  for(std::vector<poseT>::const_iterator it = poses.begin() ; it < poses.end() ; it++, count++ )
  {
    for( int i = 0 ; i < model_set.size() ; i++ )
    {
      if(model_set[i].model_label == it->model_name )
      {
        pcl::PointCloud<myPointXYZ>::Ptr cur_cloud(new pcl::PointCloud<myPointXYZ>()); 
        //pcl::transformPointCloud(*rec[i], *cur_cloud, Eigen::Vector3f (0, 0, 0), calibrate_rot);
        if( adjust )
          pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation*calibrate_rot);
        else
          pcl::transformPointCloud(*rec[i], *cur_cloud, it->shift, it->rotation);

        std::stringstream ss;
        ss << count;

        viewer->addPolygonMesh<myPointXYZ>(cur_cloud, model_set[i].model_mesh->polygons, it->model_name+"_"+ss.str());
        if( it->model_name == "link" )
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.55, 0.05, it->model_name+"_"+ss.str());
        else if( it->model_name == "node" )
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.05, 0.55, 1.0, it->model_name+"_"+ss.str());
        break;
      }
    }

  }
}

bool matchCloud(const ModelT &model, const ModelT &cloud, float ratio, float T)
{
  if( model.model_label != cloud.model_label || sqrDistPt(model.model_center->at(0), cloud.model_center->at(0)) >= 0.1*0.1 )
    return false;

  pcl::search::KdTree<myPointXYZ> tree;
  tree.setInputCloud(model.model_cloud);

  float sqrT = T*T;
  int count = 0;  
  for( pcl::PointCloud<myPointXYZ>::const_iterator it = cloud.model_cloud->begin() ; it < cloud.model_cloud->end() ; it++ )
  {               
    std::vector<int> indices (1);
    std::vector<float> sqr_distances (1);
    int nres = tree.nearestKSearch(*it, 1, indices, sqr_distances);
    if ( nres >= 1 && sqr_distances[0] <= sqrT )
      count++;
  }
  //std::cerr << "COUNT: " << (count+0.0) / cloud.model_cloud->size() << std::endl;
  if( (count+0.0) / cloud.model_cloud->size() > ratio )
    return true;
  else
    return false;
}

int overlapPose(const std::vector<ModelT> &model_set, const std::vector<poseT> &est_poses, const std::vector<poseT> &gt_poses)
{
  std::vector<ModelT> est_insts, gt_insts;
  for( std::vector<poseT>::const_iterator it = est_poses.begin() ; it < est_poses.end() ; it++ ){
    for( int i = 0 ; i < model_set.size() ; i++ ){
      if( model_set[i].model_label == it->model_name )
      {
        ModelT new_data;
        new_data.model_label = it->model_name;
        new_data.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
        new_data.model_center = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*model_set[i].model_cloud, *new_data.model_cloud, it->shift, it->rotation);
        pcl::transformPointCloud(*model_set[i].model_center, *new_data.model_center, it->shift, it->rotation);
        est_insts.push_back(new_data);
        break;
      }
    }
  }
  Eigen::Quaternionf calibrate_rot(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f (1, 0, 0)));
  for( std::vector<poseT>::const_iterator it = gt_poses.begin() ; it < gt_poses.end() ; it++ ){
    for( int i = 0 ; i < model_set.size() ; i++ ){
      if( model_set[i].model_label == it->model_name )
      {
        ModelT new_data;
        new_data.model_label = it->model_name;
        new_data.model_cloud = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); 
        new_data.model_center = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>());
        pcl::transformPointCloud(*model_set[i].model_cloud, *new_data.model_cloud, it->shift, it->rotation*calibrate_rot);
        pcl::transformPointCloud(*model_set[i].model_center, *new_data.model_center, it->shift, it->rotation*calibrate_rot);
        gt_insts.push_back(new_data);
        break;
      }
    }
  }
  //std::cerr << est_insts.size() << " " << gt_insts.size() << std::endl;
  int true_count = 0;
  for( std::vector<ModelT>::iterator est_it = est_insts.begin(); est_it < est_insts.end() ; est_it++ ){
    for( std::vector<ModelT>::iterator gt_it = gt_insts.begin(); gt_it < gt_insts.end() ; gt_it++ ){
      if( matchCloud(*gt_it, *est_it, 0.7, 0.01) == true )
      {
        true_count++;
        break;
      }
    }
  }

  return true_count;
}
/*********************************************************************************/
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

	pcl::PointCloud<PointT>::Ptr scene_f = removePlane(scene);

	//computeNormals(cloud, cloud_normal, radius);
	//pcl::PointCloud<PointT>::Ptr label_cloud = recog.recognize(cloud, cloud_normal);

	spPooler triple_pooler;
	triple_pooler.lightInit(scene, hie_producer, radius, down_ss);
	std::cerr << "LAB Pooling!" << std::endl;
	triple_pooler.build_SP_LAB(lab_pooler_set, false);

			viewer->removeAllPointClouds();
	pcl::PointCloud<PointLT>::Ptr foreground_cloud(new pcl::PointCloud<PointLT>());
	for( int ll = 0 ; ll <= 2 ; ll++ )
	{
		std::cerr << "L" << ll <<" Inference!" << std::endl;
		bool reset_flag = ll == 0 ? true : false;
		if( ll >= 1 ) {
			triple_pooler.extractForeground(false);
		}
		triple_pooler.InputSemantics(binary_models[ll], ll, reset_flag, false);


		foreground_cloud = triple_pooler.getSemanticLabels();
		if( viewer )
		{
			//viewer->addPointCloud(final_cloud, "labels");
			visualizeLabels(foreground_cloud, viewer, color_label);
			viewer->spin();
		}

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
  }



int main(int argc, char** argv)
{
    //std::string in_path("/home/chi/Downloads/");
    //std::string scene_name("UR5_1");

    ros::init(argc,argv,"sp_segmenter_node");    
    ros::NodeHandle nh;
    pc_sub = nh.subscribe(POINTS_IN,1,callback);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);

    //pcl::console::parse_argument(argc, argv, "--p", in_path);
    //pcl::console::parse_argument(argc, argv, "--i", scene_name);
    
    //in_path = in_path + scene_name + "/";
    
    //std::string out_cloud_path("../../data_pool/IROS_demo/UR5_1/");
    //pcl::console::parse_argument(argc, argv, "--o", out_cloud_path);
    //boost::filesystem::create_directories(out_cloud_path);
    
    std::string svm_path("data/UR5_svm/");
    std::string shot_path("data/UW_shot_dict/");
//    std::string sift_path("UW_new_sift_dict/");
//    std::string fpfh_path("UW_fpfh_dict/");
/***************************************************************************************************************/
    
    float ratio = 0.1;
    
    pcl::console::parse_argument(argc, argv, "--rt", ratio);
    pcl::console::parse_argument(argc, argv, "--ss", down_ss);
     pcl::console::parse_argument(argc, argv, "--zmax", zmax);
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        view_flag = true;
    
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

    ros::spin();
    
    for( int ll = 0 ; ll <= 2 ; ll++ )
        free_and_destroy_model(&binary_models[ll]);
//    for( int ll = 1 ; ll <= 4 ; ll++ )
//        free_and_destroy_model(&multi_models[ll]);
    
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
    
    viewer->removePointCloud("label_cloud");
    viewer->addPointCloud(view_cloud, "label_cloud");
//viewer->spinOnce(1);
    viewer->spin();
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
