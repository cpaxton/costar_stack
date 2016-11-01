#include "sp_segmenter/semanticSegmentation.h"
#include <pcl/filters/crop_box.h>
#include <tf_conversions/tf_eigen.h>

semanticSegmentation::semanticSegmentation(int argc, char** argv) : useTableSegmentation(true)
{
    this->classReady = false;
    
    this->down_ss = 0.003;
    this->ratio = 0.1;
    this->compute_pose = false;
    this->view_flag = false;
    pcl::console::parse_argument(argc, argv, "--rt", this->ratio);
    pcl::console::parse_argument(argc, argv, "--ss", this->down_ss);
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        this->view_flag = true;
    if( pcl::console::find_switch(argc, argv, "-p") == true )
        this->compute_pose = true;
}

void semanticSegmentation::setNodeHandle(const ros::NodeHandle &nh)
{
    this->nh = nh;
    initializeSemanticSegmentation(); // initialize all variables before doing semantic segmentation
    this->classReady = true;
    this->doingGripperSegmentation = false;
    this->number_of_segmentation_done = 0;
}

semanticSegmentation::semanticSegmentation(int argc, char** argv, const ros::NodeHandle &nh)
{
    this->classReady = false;
    
    this->down_ss = 0.003;
    this->ratio = 0.1;
    this->compute_pose = false;
    this->view_flag = false;
    pcl::console::parse_argument(argc, argv, "--rt", this->ratio);
    pcl::console::parse_argument(argc, argv, "--ss", this->down_ss);
    if( pcl::console::find_switch(argc, argv, "-v") == true )
        this->view_flag = true;
    if( pcl::console::find_switch(argc, argv, "-p") == true )
        this->compute_pose = true;
    
    this->setNodeHandle(nh);
}

void semanticSegmentation::initializeSemanticSegmentation()
{
    // maxframes = 15;
    this->nh.param("maxFrames",maxframes,15);
    cur_frame_idx = 0;
    cloud_ready = false;
    cloud_vec.clear();
    cloud_vec.resize(maxframes);
    cur_frame_idx = 0;
    cloud_ready = false;

    this->hasTF = false;
    this->radius = 0.02;
    this->pairWidth = 0.1;
    this->voxelSize = 0.003;
    uchar color_label_tmp[11][3] =
    { 
        {255, 255, 255},
        {255, 0, 0},
        {0, 255, 0},
        {0, 0, 255},
        {255, 255, 0},
        {255, 0, 255},
        {0, 255, 255},
        {255, 128, 0},
        {255, 0, 128},
        {0, 128, 255},
        {128, 0, 255}
    };

    table_corner_published = 0;

    std::copy(&color_label_tmp[0][0], &color_label_tmp[0][0]+11*3,&color_label[0][0]);
    double cropBoxX, cropBoxY, cropBoxZ;
    
    this->nh.param("useTF",useTFinsteadOfPoses,true);
    this->nh.param("GripperTF",gripperTF,std::string("endpoint_marker"));
    
    //getting subscriber/publisher parameterscloud_ready = false;
    this->nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
    this->nh.param("POINTS_OUT", POINTS_OUT,std::string("points_out"));
    //get only best poses (1 pose output) or multiple poses
    this->nh.param("objRecRANSACdetector", objRecRANSACdetector, std::string("StandardRecognize"));

    this->nh.param("minConfidence", minConfidence, 0.0);
    this->nh.param("aboveTableMin", aboveTableMin, 0.01);
    this->nh.param("aboveTableMax", aboveTableMax, 0.25);
    this->haveTable = false;
    this->nh.param("loadTable",loadTable, true);
    this->nh.param("useTableSegmentation",useTableSegmentation,true);
    this->nh.param("tableDistanceThreshold",tableDistanceThreshold,0.02);
    this->nh.param("tableAngularThreshold",tableAngularThreshold,2.0);
    this->nh.param("tableMinimalInliers",tableMinimalInliers,5000);
    
    this->nh.param("useCropBox",useCropBox,true);
    this->nh.param("cropBoxX",cropBoxX,1.0);
    this->nh.param("cropBoxY",cropBoxY,1.0);
    this->nh.param("cropBoxZ",cropBoxZ,1.0);
    this->nh.param("setObjectOrientation",setObjectOrientationTarget,false);
    this->nh.param("preferredOrientation",targetNormalObjectTF,std::string("/world"));
    this->nh.param("useBinarySVM",useBinarySVM,false);
    this->nh.param("useMultiClassSVM",useMultiClassSVM,true);
    this->nh.param("useMedianFilter",use_median_filter,true);
    this->nh.param("enableTracking",enableTracking,false);
    
    this->nh.param("useObjectPersistence",useObjectPersistence,false);

    crop_box_size = Eigen::Vector3f(cropBoxX, cropBoxY, cropBoxZ);
    

    this->nh.param("cropBoxGripperX",cropBoxX,1.0);
    this->nh.param("cropBoxGripperY",cropBoxY,1.0);
    this->nh.param("cropBoxGripperZ",cropBoxZ,1.0);
    crop_box_gripper_size = Eigen::Vector3f(cropBoxX, cropBoxY, cropBoxZ);

    tableConvexHull = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    
    lab_pooler_set.resize(6);
    binary_models.resize(3);
    multi_models.resize(3);
    model_name = std::vector<std::string>(OBJECT_MAX, "");
    

    if (!useTableSegmentation) {
      std::cerr << "WARNING: not using table segmentation!\n";
    }
    else table_corner_pub = nh.advertise<sensor_msgs::PointCloud2>("table_corner",3);

    if(loadTable && useTableSegmentation)
    {
        std::string load_directory;
        nh.param("saveTable_directory",load_directory,std::string("./data"));
        pcl::PCDReader reader;
        if( reader.read (load_directory+"/table.pcd", *tableConvexHull) == 0){
            std::cerr << "Table load successfully\n";
            haveTable = true;
        }
        else {
            haveTable = false;
            std::cerr << "Failed to load table. Remove all objects, put the ar_tag marker in the center of the table and it will get anew table data\n";
        }
    }
    
    std::cerr << "Node is running with objRecRANSACdetector: " << objRecRANSACdetector << "\n";
    
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);
    
    nh.param("voxelSize", voxelSize, 0.003);

    //double link_width = 0.075;
    //double node_width = 0.05;
    //double sander_width = 0.16;
    double link_width, node_width, sander_width;
    nh.param("link_width", link_width,  0.075);
    nh.param("node_width", node_width,  0.05);
    nh.param("sander_width", sander_width, 0.16);

    if (!useTFinsteadOfPoses) {
        std::cerr << "Node publish pose array.\n";
        pose_pub = nh.advertise<geometry_msgs::PoseArray>(POSES_OUT,1000);
        pc_sub = nh.subscribe(POINTS_IN,1,&semanticSegmentation::callbackPoses,this);
    }
    else {
        std::cerr << "Node publish TF.\n";
        listener = new (tf::TransformListener);
        spSegmenter = nh.advertiseService("SPSegmenter",&semanticSegmentation::serviceCallback,this);
        segmentGripper = nh.advertiseService("segmentInGripper",&semanticSegmentation::serviceCallbackGripper,this);
        pc_sub = nh.subscribe(POINTS_IN,1,&semanticSegmentation::updateCloudData,this);
    }
    
    detected_object_pub = nh.advertise<costar_objrec_msgs::DetectedObjectList>("detected_object_list",1);

    std::string svm_path,shot_path;
    //get path parameter for svm and shot
    nh.param("svm_path", svm_path,std::string("data/UR5_drill_svm/"));
    nh.param("shot_path", shot_path,std::string("data/UW_shot_dict/"));
    
    std::cerr << "Ratio: " << ratio << std::endl;
    std::cerr << "Downsample: " << down_ss << std::endl;
    
    std::string mesh_path;
    //get parameter for mesh path and cur_name
    nh.param("mesh_path", mesh_path,std::string("data/mesh/"));
    std::vector<std::string> cur_name = stringVectorArgsReader(nh, "cur_name", std::string("drill"));
    
    //get symmetry parameter of the objects
    objectDict = fillDictionary(nh, cur_name);

    bool use_cuda;
    nh.param("use_cuda", use_cuda,true);

    double objectVisibility, sceneVisibility;

    nh.param("objectVisibility",objectVisibility,0.1);
    nh.param("sceneVisibility", sceneVisibility,0.1);
    
    
    if (!useMultiClassSVM || cur_name.size() == 1){
        // initialize combinedObjRecRansac
        combinedObjRec = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(pairWidth, voxelSize));
        combinedObjRec->setParams(objectVisibility,sceneVisibility);
        combinedObjRec->setUseCUDA(use_cuda);
    }
    else objrec.resize(cur_name.size());

    for (std::size_t model_id = 0; model_id < cur_name.size(); model_id++)
    {
        // add all models. model_id starts in model_name start from 1.
        std::string temp_cur = cur_name.at(model_id);
        if (useMultiClassSVM && cur_name.size() > 1)
        {
            
            if( temp_cur == "link_uniform")
            {
                objrec[model_id] = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(link_width, voxelSize));
                objrec[model_id]->setParams(objectVisibility,sceneVisibility);
            }
            else if( temp_cur == "node_uniform")
            {
                objrec[model_id] = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(node_width, voxelSize));
                objrec[model_id]->setParams(objectVisibility,sceneVisibility);
            }
            else if( temp_cur == "sander_makita")
            {
                objrec[model_id] = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(sander_width, voxelSize));
                objrec[model_id]->setParams(objectVisibility,sceneVisibility);
            }
            else
            {
                objrec[model_id] = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(pairWidth, voxelSize));
                objrec[model_id]->setParams(objectVisibility,sceneVisibility);
            }

            /// @todo allow different visibility parameters for each object class
            
            objrec[model_id]->setUseCUDA(use_cuda);
            objrec[model_id]->AddModel(mesh_path + temp_cur, temp_cur);
        }
        else combinedObjRec->AddModel(mesh_path + temp_cur, temp_cur);
        model_name[model_id+1] = temp_cur;
        model_name_map[temp_cur] = model_id+1;
        ModelT mesh_buf = LoadMesh(mesh_path + temp_cur, temp_cur);
        
        mesh_set.push_back(mesh_buf);
        objectTFIndex[temp_cur] = 0;
    }

    hie_producer = Hier_Pooler(radius);
    hie_producer.LoadDict_L0(shot_path, "200", "200");
    hie_producer.setRatio(ratio);

    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        lab_pooler_set[i] = cur_pooler;
    }
    
    for( int ll = 0 ; ll < 3 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;
        
        binary_models[ll] = load_model((svm_path+"binary_L"+ss.str()+"_f.model").c_str());
        if (cur_name.size() > 1) // doing more than one object classisfication
            multi_models[ll] = load_model((svm_path+"multi_L"+ss.str()+"_f.model").c_str());
    }
    
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
          std::cout << "Warning: tried to add duplicate model name to tracker" << std::endl;
       }
      }
    }
}

semanticSegmentation::~semanticSegmentation(){
    for( int ll = 0 ; ll < 3 ; ll++ )
    {
        free_and_destroy_model(&binary_models[ll]);
        if (mesh_set.size() > 1)
            free_and_destroy_model(&multi_models[ll]);
    }
}

void semanticSegmentation::cropPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_input, 
  const Eigen::Affine3f& camera_tf_in_table, 
  const Eigen::Vector3f& box_size)
{
  pcl::PointCloud<PointT>::Ptr  cropped_cloud(new pcl::PointCloud<PointT>());
  pcl::CropBox<PointT> crop_box;
  crop_box.setKeepOrganized(true);
  crop_box.setInputCloud(cloud_input);
  crop_box.setMax(box_size.homogeneous());
  crop_box.setMin((-box_size).homogeneous());
  crop_box.setTransform(camera_tf_in_table);
  crop_box.filter(*cropped_cloud);
  cloud_input = cropped_cloud;
}

void semanticSegmentation::callbackPoses(const sensor_msgs::PointCloud2 &inputCloud)
{
    if (!classReady) return;
    if (useTableSegmentation)
    {
        if (!haveTable) haveTable = getAndSaveTable(inputCloud);
        
        if (haveTable) {
             // publish the table corner every 10 frame of inputCloud
            if (table_corner_published == 0)
            {
                sensor_msgs::PointCloud2 output_msg;
                toROSMsg(*tableConvexHull,output_msg);
                output_msg.header.frame_id = inputCloud.header.frame_id;
                // std::cerr << "Published table corner point cloud\n";
                table_corner_pub.publish(output_msg);
            }
            else 
            {
                table_corner_published++;
                if (table_corner_published > 10) table_corner_published = 0;
            }
        }
        else return; // still does not have table
    }
    
    // Service call will run SPSegmenter
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
    
    fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available!\n";
        return;
    }
    
    if (useTableSegmentation) {
      segmentCloudAboveTable(full_cloud, tableConvexHull, aboveTableMin, aboveTableMax);
    }

    if (full_cloud->size() < 1){
        std::cerr << "No cloud available after removing all object outside the table.\nPut some object above the table.\n";
        return;
    }

    if(useCropBox) {
      Eigen::Affine3d cam_tf_in_table;
      tf::transformTFToEigen(table_transform.inverse(), cam_tf_in_table);
      cropPointCloud(full_cloud, cam_tf_in_table.cast<float>(), crop_box_size);
    }
    
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available after cropping.\n";
        return;
    }
    
    // get all poses from spSegmenterCallback
    std::vector<poseT> all_poses = spSegmenterCallback(full_cloud,*final_cloud);
    
    //publishing the segmented point cloud
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*final_cloud,output_msg);
    output_msg.header.frame_id = inputCloud.header.frame_id;
    pc_pub.publish(output_msg);
    
    if (all_poses.size() < 1) {
        std::cerr << "Failed to segment objects on the table.\n";
        return;
    }
    else
    {
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
        pose_pub.publish(msg);
    }
}

std::vector<poseT> semanticSegmentation::spSegmenterCallback(const pcl::PointCloud<PointT>::Ptr full_cloud, pcl::PointCloud<PointLT> & final_cloud)
{
    pcl::PointCloud<PointT>::Ptr scene_f(new pcl::PointCloud<PointT>());
    *scene_f = *full_cloud;
  
    if( viewer )
    {
        std::cerr<<"Visualize table segmented screen"<<std::endl;
        viewer->removeAllPointClouds();
        viewer->addPointCloud(scene_f, "whole_scene");
        viewer->spin();
        viewer->removeAllPointClouds();
    }
    // std::vector< pcl::PointCloud<myPointXYZ>::Ptr > &cloud_set
    spPooler triple_pooler;
    triple_pooler.lightInit(scene_f, hie_producer, radius, down_ss);
    std::cerr << "LAB Pooling!" << std::endl;
    triple_pooler.build_SP_LAB(lab_pooler_set, false);

    // pcl::PointCloud<PointLT>::Ptr foreground_cloud(new pcl::PointCloud<PointLT>());
    // std::vector< std::vector<PR_ELEM> > cur_fore_pr(3);
    
    if(useBinarySVM)
    {
        // ll means order of superpixels for classification
        // right now I only provide ll=0,1 for classification, 
        // the larger order you use will increase the running time of semantic segmentation
        // recommend to use 1 by default for foreground-background classification
        for( int ll = 0 ; ll <= 1 ; ll++ )
        {
            bool reset_flag = ll == 0 ? true : false;
            if( ll >= 0 )
                triple_pooler.extractForeground(false);
            triple_pooler.InputSemantics(binary_models[ll], ll, reset_flag, false);
        }

        triple_pooler.extractForeground(true);
    }

    std::vector<poseT> all_poses1;
    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    // if has useMultiSVM flag and has more than one objects, do multi object classification
    if (useMultiClassSVM && mesh_set.size() > 1)
    {
        // ll means order of superpixels for classification
        // right now I only provide ll=0,1,2 for classification, 
        // specify the starting order by sll and ending order by ell
        // the larger order you use will increase the running time of semantic segmentation
        // recommend to use 1 by default for link-node-sander classification
        // recommend to use 0 by default for link-node classification
        int sll = 1, ell = 1;
        for( int ll = sll ; ll <= ell ; ll++ )
        {
           bool reset_flag = ll == sll ? true : false;
           triple_pooler.InputSemantics(multi_models[ll], ll, reset_flag, false);
        }
        pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
        label_cloud = triple_pooler.getSemanticLabels();
        pcl::copyPointCloud(*label_cloud, *scene_xyz);
        triple_pooler.reset();
        
        if( viewer )
        {
            std::cerr<<"Visualize after segmentation"<<std::endl;
            visualizeLabels(label_cloud, viewer, color_label);
        }
        std::vector< pcl::PointCloud<myPointXYZ>::Ptr > cloud_set(mesh_set.size()+1); // separate the clouds
        for( size_t j = 0 ; j < cloud_set.size() ; j++ )
            cloud_set[j] = pcl::PointCloud<myPointXYZ>::Ptr (new pcl::PointCloud<myPointXYZ>()); // object cloud starts from 1
        std::cerr<<"Split cloud after segmentation"<<std::endl;
        splitCloud(label_cloud, cloud_set);

        std::cerr<<"Calculate poses"<<std::endl;
        if (doingGripperSegmentation)
        {
            std::vector<poseT> tmp_poses;
            int objrec_index = model_name_map[objectClassInGripper];
            if      (objRecRANSACdetector == "StandardBest")      objrec[objrec_index-1]->StandardBest(cloud_set[objrec_index], tmp_poses);
            else if (objRecRANSACdetector == "GreedyRecognize")   objrec[objrec_index-1]->GreedyRecognize(cloud_set[objrec_index], tmp_poses);
            else if (objRecRANSACdetector == "StandardRecognize") objrec[objrec_index-1]->StandardRecognize(cloud_set[objrec_index], tmp_poses, minConfidence);
            else ROS_ERROR("Unsupported objRecRANSACdetector!");
            all_poses1.insert(all_poses1.end(), tmp_poses.begin(), tmp_poses.end());
        }
        else
        {
            #pragma omp parallel for schedule(dynamic, 1)
            for(size_t j = 1 ; j <= mesh_set.size(); j++ ){ // loop over all objects
                if( cloud_set[j]->empty() == false )
                {
                    std::vector<poseT> tmp_poses;
                    if      (objRecRANSACdetector == "StandardBest")      objrec[j-1]->StandardBest(cloud_set[j], tmp_poses);
                    else if (objRecRANSACdetector == "GreedyRecognize")   objrec[j-1]->GreedyRecognize(cloud_set[j], tmp_poses);
                    else if (objRecRANSACdetector == "StandardRecognize") objrec[j-1]->StandardRecognize(cloud_set[j], tmp_poses, minConfidence);
                    else ROS_ERROR("Unsupported objRecRANSACdetector!");


                    #pragma omp critical
                    {
                        all_poses1.insert(all_poses1.end(), tmp_poses.begin(), tmp_poses.end());
                    }
                }
            }
        }
    }
    else
    {
        // just combine all the object together and do combined object ransac
        pcl::copyPointCloud(*scene_f,*scene_xyz);
        std::vector<poseT> tmp_poses;
        if      (objRecRANSACdetector == "StandardBest")      combinedObjRec->StandardBest(scene_xyz, all_poses1);
        else if (objRecRANSACdetector == "GreedyRecognize")   combinedObjRec->GreedyRecognize(scene_xyz, all_poses1);
        else if (objRecRANSACdetector == "StandardRecognize") combinedObjRec->StandardRecognize(scene_xyz, all_poses1, minConfidence);
        else ROS_ERROR("Unsupported objRecRANSACdetector!");
    }

    std::vector<poseT> all_poses = all_poses1;
    // RefinePoses(scene_xyz, mesh_set, all_poses1);
    std::map<std::string, unsigned int> objectTFIndex_no_persistence = objectTFIndex;
    std::map<std::string, unsigned int> &tmpTFIndex = objectTFIndex;
    
    // baseRotation sets the preferred orientation for initial pose detection for every object
    Eigen::Quaternion<double> baseRotation;
    if (setObjectOrientationTarget && 
        listener->waitForTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time::now(),ros::Duration(1.5)) 
       ){
          tf::StampedTransform transform;
          listener->lookupTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time(0),transform);
          tf::quaternionTFToEigen(transform.getRotation(),baseRotation);
        }
    else
    {
        // just use identity if no preferred orientation is being used
        baseRotation.setIdentity();
    }
    
    if (doingGripperSegmentation || !hasTF){
      // normalize symmetric object Orientation
        if (!doingGripperSegmentation)
        {
            std::cerr << "create tree\n";
            // this will create tree and normalize the orientation to the baseRotation
            createTree(segmentedObjectTree, objectDict, all_poses, ros::Time::now().toSec(), tmpTFIndex, baseRotation);
        }
        else
        {
            // for gripper segmentation, only one value will be updated while the other remains (assuming one gripper)
            std::cerr << "update one value on tree\n";
            updateOneValue(segmentedObjectTree, targetTFtoUpdate, objectDict, all_poses, ros::Time::now().toSec(), tmpTFIndex, baseRotation);
        }
    }
    else if (useObjectPersistence)
    {
        std::cerr << "update tree\n";
        updateTree(segmentedObjectTree, objectDict, all_poses, ros::Time::now().toSec(), tmpTFIndex, baseRotation);
        
    }
    else
    {
        // not using object persistance, recreate tree every service call.
        createTree(segmentedObjectTree, objectDict, all_poses, ros::Time::now().toSec(), tmpTFIndex, baseRotation);
    }

    // restore original index if not using object persistance
    if (!useObjectPersistence) tmpTFIndex = objectTFIndex_no_persistence;

    return getAllPoses(segmentedObjectTree);
}

bool semanticSegmentation::getAndSaveTable (const sensor_msgs::PointCloud2 &pc)
{
    std::string tableTFname, tableTFparent;
    nh.param("tableTF", tableTFname,std::string("/tableTF"));
    
    //listener->getParent(tableTFname,ros::Time(0),tableTFparent);
    tableTFparent = pc.header.frame_id;
    if (listener->waitForTransform(tableTFparent,tableTFname,ros::Time::now(),ros::Duration(1.5)))
    {
        std::cerr << "Table TF with name: '" << tableTFname << "' found with parent frame: " << tableTFparent << std::endl;
        listener->lookupTransform(tableTFparent,tableTFname,ros::Time(0),table_transform);
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        
        fromROSMsg(inputCloud,*full_cloud);
        Eigen::Vector3f crop_box_size_table = crop_box_size;
        double tableTF_z_box;
        nh.param("cropBoxTableZ", tableTF_z_box,0.005);
        crop_box_size_table[2] = tableTF_z_box;
        Eigen::Affine3d cam_tf_in_table;
        tf::transformTFToEigen(table_transform.inverse(), cam_tf_in_table);
        cropPointCloud(full_cloud, cam_tf_in_table.cast<float>(), crop_box_size_table);
        
        if( viewer )
        {
            std::cerr<<"Visualize cropbox screen"<<std::endl;
            viewer->removeAllPointClouds();
            viewer->addPointCloud(full_cloud, "cropbox_scene");
            viewer->spin();
            viewer->removeAllPointClouds();
        }

        tableConvexHull = getTableConvexHull(full_cloud, viewer, tableDistanceThreshold, tableAngularThreshold,tableMinimalInliers);
        if (tableConvexHull->size() < 3) {
            std::cerr << "Retrying table segmentation...\n";
            return false;
        }
        
        bool saveTable;
        nh.param("updateTable",saveTable, true);
        if (saveTable){
            std::string saveTable_directory;
            nh.param("saveTable_directory",saveTable_directory,std::string("./data"));
            pcl::PCDWriter writer;
            writer.write<PointT> (saveTable_directory+"/table.pcd", *tableConvexHull, true);
            std::cerr << "Saved table point cloud in : " << saveTable_directory <<"/table.pcd"<<std::endl;
        }
        return true;
    }
    else {
        std::cerr << "Failed to get table TF with name: '" << tableTFname << "'" << std::endl;
        std::cerr << "Parent: " << tableTFparent << std::endl;
        return false;
    }
}

void semanticSegmentation::updateCloudData (const sensor_msgs::PointCloud2 &pc)
{
    if (!classReady) return;
    // The callback from main only update the cloud data
    inputCloud = pc;
    
    if (useTableSegmentation)
    {
        if (!haveTable) haveTable = getAndSaveTable(inputCloud);
        
        if (haveTable) {
             // publish the table corner every 10 frame of inputCloud
            if (table_corner_published == 0)
            {
                sensor_msgs::PointCloud2 output_msg;
                toROSMsg(*tableConvexHull,output_msg);
                output_msg.header.frame_id = inputCloud.header.frame_id;
                // std::cerr << "Published table corner point cloud\n";
                table_corner_pub.publish(output_msg);
            }
            else 
            {
                table_corner_published++;
                if (table_corner_published > 10) table_corner_published = 0;
            }
        }
        else return; // still does not have table
    }

    if (use_median_filter)
    {
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    
        fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
        cloud_vec[cur_frame_idx] = full_cloud;
        cur_frame_idx++;
        // std::cerr << "PointCloud --- " << cur_frame_idx << "----" << full_cloud->size() << std::endl;
        if( cur_frame_idx >= maxframes )
        {
            cloud_ready = true;
            cur_frame_idx = 0; 
        }
    }
    
}

void semanticSegmentation::populateTFMapFromTree()
{
  ros::param::del("/instructor_landmark/objects");
  
  std::vector<value> sp_segmenter_detectedPoses = getAllNodes(segmentedObjectTree);
//  segmentedObjectTFMap.clear();
  segmentedObjectTFV.clear();
  costar_objrec_msgs::DetectedObjectList object_list;
  object_list.header.seq = ++(this->number_of_segmentation_done);
  object_list.header.stamp = ros::Time::now();
  object_list.header.frame_id =  inputCloud.header.frame_id;

  std::cerr << "detected poses: " << sp_segmenter_detectedPoses.size() << "\n";
  for (std::size_t i = 0; i < sp_segmenter_detectedPoses.size(); i++)
  {
    const value &v = sp_segmenter_detectedPoses.at(i);
    const poseT &p = std::get<1>(v).pose;
    const std::string objectTFname = std::get<1>(v).tfName;
    segmentedObjectTF objectTmp(p,objectTFname);
    segmentedObjectTFV.push_back(objectTmp);
//    segmentedObjectTFMap[objectTmp.TFname] = objectTmp;
    std::stringstream ss;
    ss << "/instructor_landmark/objects/" << p.model_name << "/" << std::get<1>(v).index;
    // std::cerr << "frame " << i << " name = " << objectTmp.TFname << "\n";
    
    ros::param::set(ss.str(), objectTmp.TFname);
    std::stringstream ss2;
    ss2 << std::get<1>(v).tfName;
    costar_objrec_msgs::DetectedObject object_tmp;
  	object_tmp.id = ss2.str();

    // std::cerr << object_tmp.id << "; " << ss.str() << "\n";

  	object_tmp.symmetry.x_rotation = objectDict[ p.model_name ].roll;
  	object_tmp.symmetry.y_rotation = objectDict[ p.model_name ].pitch;
  	object_tmp.symmetry.z_rotation = objectDict[ p.model_name ].yaw;
  	object_tmp.symmetry.x_symmetries = std::floor(2 * M_PI / objectDict[ p.model_name ].roll);
  	object_tmp.symmetry.y_symmetries = std::floor(2 * M_PI / objectDict[ p.model_name ].pitch);
  	object_tmp.symmetry.z_symmetries = std::floor(2 * M_PI / objectDict[ p.model_name ].yaw);
  	object_tmp.object_class = p.model_name;
  	object_list.objects.push_back(object_tmp);
  }

  detected_object_pub.publish(object_list);
}

pcl::PointCloud<PointT>::Ptr MedianPointCloud(const std::vector<pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > &cloud_vec)
{
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    if( cloud_vec.empty() == true )
        return full_cloud;
    
    full_cloud->resize(cloud_vec[0]->size());
    full_cloud->width = cloud_vec[0]->width;
    full_cloud->height = cloud_vec[0]->height;

    #pragma omp parallel for
    for( size_t i = 0 ; i < full_cloud->size() ; i++ )
    {
        int count = 0;
        std::vector<float> x_vec, y_vec, z_vec;
        for( size_t j = 0 ; j < cloud_vec.size() ; j++ )
        {
            if( pcl_isfinite(cloud_vec[j]->at(i).x) == true )
            {
                // full_cloud->at(i).x += cloud_vec[j]->at(i).x;
                // full_cloud->at(i).y += cloud_vec[j]->at(i).y;
                // full_cloud->at(i).z += cloud_vec[j]->at(i).z;
                x_vec.push_back(cloud_vec[j]->at(i).x);
                y_vec.push_back(cloud_vec[j]->at(i).y);
                z_vec.push_back(cloud_vec[j]->at(i).z);

                full_cloud->at(i).rgba = cloud_vec[j]->at(i).rgba;
                count++;
            }
        }

        if( count > cloud_vec.size() / 2.0 )
        {
            std::sort(x_vec.begin(), x_vec.end());
            std::sort(y_vec.begin(), y_vec.end());
            std::sort(z_vec.begin(), z_vec.end());
            full_cloud->at(i).x = x_vec[x_vec.size()/2.0];
            full_cloud->at(i).y = y_vec[y_vec.size()/2.0];
            full_cloud->at(i).z = z_vec[z_vec.size()/2.0];
        }
        else
        {
            full_cloud->at(i).x = std::numeric_limits<float>::quiet_NaN();
            full_cloud->at(i).y = std::numeric_limits<float>::quiet_NaN();
            full_cloud->at(i).z = std::numeric_limits<float>::quiet_NaN();
        }
    }
    return full_cloud;
}

bool semanticSegmentation::serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (!classReady) {
      ROS_ERROR("Class is not ready!");
      return false;
    }
    // Service call will run SPSegmenter
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
    
    if (!haveTable && useTableSegmentation)
    {
        ROS_ERROR("Class does not have table data yet!");
        return false; // still does not have table
    }
    
    // fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    // fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (!use_median_filter)  // not using median filter
        fromROSMsg(inputCloud,*full_cloud);
    else if(cloud_ready == true )
    {
        std::cerr << "Averaging point clouds" << std::endl;
        // full_cloud = AveragePointCloud(cloud_vec);
        full_cloud = MedianPointCloud(cloud_vec);
        // cloud_ready = false;
        std::cerr << "Averaging point clouds Done" << std::endl;
    }
    else
    {
        ROS_INFO("Need to accumulate more frames!");
        return false;
    }

    if (full_cloud->size() < 1){
        ROS_ERROR("No cloud available!");
        return false;
    }
    
    if (useTableSegmentation) {
      segmentCloudAboveTable(full_cloud, tableConvexHull, aboveTableMin, aboveTableMax);
    }
    
    if (full_cloud->size() < 1){
        ROS_ERROR("No cloud available after removing all object outside the table. Put objects above the table.");
        return false;
    }

    if(useCropBox) {
      Eigen::Affine3d cam_tf_in_table;
      tf::transformTFToEigen(table_transform.inverse(), cam_tf_in_table);
      cropPointCloud(full_cloud, cam_tf_in_table.cast<float>(), crop_box_size);
    }

    if (full_cloud->size() < 1){
        ROS_ERROR("No cloud available after cropping.");
        return false;
    }
    
    // get all poses from spSegmenterCallback
    std::vector<poseT> all_poses = spSegmenterCallback(full_cloud,*final_cloud);
    ROS_INFO("Found %lu objects",all_poses.size());
    // std::cerr << "found: " << all_poses.size() << "\n";

    if(enableTracking)
    {
      tracker->generateTrackingPoints(inputCloud.header.stamp, all_poses);
    }
    
    //publishing the segmented point cloud
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*final_cloud,output_msg);
    output_msg.header.frame_id = inputCloud.header.frame_id;
    pc_pub.publish(output_msg);
    
    fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (all_poses.size() < 1) {
        ROS_ERROR("Failed to find any objects on the table.");
        return false;
    }
  
    this->populateTFMapFromTree();
  
    std::cerr << "Segmentation done.\n";
    ROS_INFO("Segmentation done.");
    
    hasTF = true;
    return true;
}

bool semanticSegmentation::serviceCallbackGripper (sp_segmenter::segmentInGripper::Request & request, sp_segmenter::segmentInGripper::Response& response)
{
    std::cerr << "Segmenting object on gripper...\n";
    std::string bestPoseOriginal = objRecRANSACdetector;

     // Use the detector for objects in the gripper
    nh.param("objRecRANSACdetectorInGripper",objRecRANSACdetector,std::string("StandardBest"));
    
    targetTFtoUpdate = request.tfToUpdate;
    objectClassInGripper = request.objectClass;

    this->doingGripperSegmentation = true;
  
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
    std::string segmentFail("Object in gripper segmentation fails.");
    
    fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available";
        response.result = segmentFail;
        objRecRANSACdetector = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    if (listener->waitForTransform(inputCloud.header.frame_id,gripperTF,ros::Time::now(),ros::Duration(1.5)))
    {
        tf::StampedTransform transform;
        listener->lookupTransform(inputCloud.header.frame_id,gripperTF,ros::Time(0),transform);

        Eigen::Affine3d gripper_tf;
        tf::transformTFToEigen(transform.inverse(), gripper_tf);
        cropPointCloud(full_cloud, gripper_tf.cast<float>(), crop_box_gripper_size);
    }
    else
    {
        std::cerr << "Fail to get transform between: "<< gripperTF << " and "<< inputCloud.header.frame_id << std::endl;
        response.result = segmentFail;
        objRecRANSACdetector = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available around gripper. Make sure the object can be seen by the camera.\n";
        objRecRANSACdetector = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    // get best poses from spSegmenterCallback
    std::vector<poseT> all_poses = spSegmenterCallback(full_cloud,*final_cloud);
    
    if (all_poses.size() < 1) {
        std::cerr << "Fail to segment the object around gripper.\n";
        response.result = segmentFail;
        objRecRANSACdetector = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    
    //publishing the segmented point cloud
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*final_cloud,output_msg);
    output_msg.header.frame_id = inputCloud.header.frame_id;
    pc_pub.publish(output_msg);
  
    this->populateTFMapFromTree();
  
    std::cerr << "Object In gripper segmentation done.\n";
    objRecRANSACdetector = bestPoseOriginal;
    this->doingGripperSegmentation = false;
    hasTF = true;
    response.result = "Object In gripper segmentation done.\n";
    return true;
}

void semanticSegmentation::publishTF()
{
    if (!useTFinsteadOfPoses) return; // do nothing
    if (hasTF)
    {
        // broadcast all transform
        std::string parent = inputCloud.header.frame_id;
        // int index = 0;
        for (std::size_t i = 0; i < segmentedObjectTFV.size(); i++){
            // std::cerr << "Publishing: " << segmentedObjectTFV.at(i).TFname << "\n";
            br.sendTransform(
                segmentedObjectTFV.at(i).generateStampedTransform(parent)
                );
        }
        // for (std::map<std::string, segmentedObjectTF>::iterator it=segmentedObjectTFMap.begin(); it!=segmentedObjectTFMap.end(); ++it)
        // {
        //     std::cerr << "Publishing: " << it->second.TFname << "\n";
        //     br.sendTransform(
        //                      it->second.generateStampedTransform(parent)
        //                      );
        // }
    }
}
