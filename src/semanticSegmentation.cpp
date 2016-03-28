#include "sp_segmenter/semanticSegmentation.h"

semanticSegmentation::semanticSegmentation(int argc, char** argv)
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
    this->hasTF = false;
    this->radius = 0.02;
    this->pairWidth = 0.05;
    this->voxelSize = 0.003;
    uchar color_label_tmp[11][3] =
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
    std::copy(&color_label_tmp[0][0], &color_label_tmp[0][0]+11*3,&color_label[0][0]);
    
    this->nh.param("useTF",useTFinsteadOfPoses,true);
    this->nh.param("GripperTF",gripperTF,std::string("endpoint_marker"));
    
    //getting subscriber/publisher parameters
    this->nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
    this->nh.param("POINTS_OUT", POINTS_OUT,std::string("points_out"));
    //get only best poses (1 pose output) or multiple poses
    this->nh.param("bestPoseOnly", bestPoseOnly, true);
    this->nh.param("minConfidence", minConfidence, 0.0);
    this->nh.param("aboveTable", aboveTable, 0.01);
    this->haveTable = false;
    this->nh.param("loadTable",loadTable, true);
    this->nh.param("setObjectOrientation",setObjectOrientationTarget,false);
    this->nh.param("preferredOrientation",targetNormalObjectTF,std::string("/world"));
  
    tableConvexHull = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    
    lab_pooler_set.resize(6);
    binary_models.resize(3);
    multi_models.resize(3);
    model_name = std::vector<std::string>(OBJECT_MAX, "");
    
    if(loadTable)
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
            std::cerr << "Fail to load table. Remove all objects, put the ar_tag marker in the center of the table and it will get anew table data\n";
        }
    }
    
    if (bestPoseOnly)
        std::cerr << "Node will only output the best detected poses \n";
    else
        std::cerr << "Node will output all detected poses \n";
    
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);
    nh.param("pairWidth", pairWidth, 0.05);
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

    objrec.resize(cur_name.size());
    for (int model_id = 0; model_id < cur_name.size(); model_id++)
    {
        // add all models. model_id starts in model_name start from 1.
        std::string temp_cur = cur_name.at(model_id);

        objrec[model_id] = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(pairWidth, voxelSize));
        objrec[model_id]->AddModel(mesh_path + temp_cur, temp_cur);
        model_name[model_id+1] = temp_cur;
        model_name_map[temp_cur] = model_id+1;
        ModelT mesh_buf = LoadMesh(mesh_path + temp_cur + ".obj", temp_cur);
        
        mesh_set.push_back(mesh_buf);
        objectTFIndex[temp_cur] = 0;
    }
/***************************
    objrec[1]->AddModel(mesh_path + "link", "link");
    objrec[2]->AddModel(mesh_path + "node", "node");
    // model_name[model_id+1] = temp_cur;
    // model_name_map[temp_cur] = model_id+1;
    ModelT mesh_buf = LoadMesh(mesh_path + "link.obj", "link");
    mesh_set.push_back(mesh_buf);
***************************/

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
}

semanticSegmentation::~semanticSegmentation(){
    for( int ll = 0 ; ll < 3 ; ll++ )
    {
        free_and_destroy_model(&binary_models[ll]);
        if (mesh_set.size() > 1)
            free_and_destroy_model(&multi_models[ll]);
    }
}

void semanticSegmentation::callbackPoses(const sensor_msgs::PointCloud2 &inputCloud)
{
    if (!classReady) return;
    if (!haveTable)
    {
        haveTable = getAndSaveTable(inputCloud);
        if (!haveTable) return; // still does not have table
    }
    
    // Service call will run SPSegmenter
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
    
    fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available";
        return;
    }
    
    segmentCloudAboveTable(full_cloud, tableConvexHull, aboveTable);
    
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available after removing all object outside the table.\nPut some object above the table.\n";
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
        std::cerr << "Fail to segment objects on the table.\n";
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
        std::cerr<<"Visualize whole screen"<<std::endl;
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
    for( int ll = 0 ; ll <= 1 ; ll++ )
    {
        bool reset_flag = ll == 0 ? true : false;
        if( ll >= 1 )
            triple_pooler.extractForeground(false);
        triple_pooler.InputSemantics(binary_models[ll], ll, reset_flag, false);
    }

    triple_pooler.extractForeground(true);
    if (mesh_set.size() > 1) // more than one objects, do multi object classification
    {
        pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
        for( int ll = 1 ; ll <= 1 ; ll++ )
        {
           bool reset_flag = ll == 1 ? true : false;
           triple_pooler.InputSemantics(multi_models[ll], ll, reset_flag, false);
        }
    }
    pcl::PointCloud<PointLT>::Ptr label_cloud(new pcl::PointCloud<PointLT>());
    label_cloud = triple_pooler.getSemanticLabels();
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

    std::vector<poseT> all_poses1;
    std::cerr<<"Calculate poses"<<std::endl;
    #pragma omp parallel for schedule(dynamic, 1)
    for(size_t j = 1 ; j <= mesh_set.size(); j++ ){ // loop over all objects
        if( cloud_set[j]->empty() == false )
        {
            std::vector<poseT> tmp_poses;
            if (bestPoseOnly)
                objrec[j-1]->StandardBest(cloud_set[j], tmp_poses);
            else
                objrec[j-1]->StandardRecognize(cloud_set[j], tmp_poses, minConfidence);

            #pragma omp critical
            {
                all_poses1.insert(all_poses1.end(), tmp_poses.begin(), tmp_poses.end());
            }
        }
    }

    pcl::PointCloud<myPointXYZ>::Ptr scene_xyz(new pcl::PointCloud<myPointXYZ>());
    pcl::copyPointCloud(*label_cloud, *scene_xyz);

    std::vector<poseT> all_poses = all_poses1;
    // RefinePoses(scene_xyz, mesh_set, all_poses1);
  
    if (doingGripperSegmentation || !hasTF){
      // normalize symmetric object Orientation
        Eigen::Quaternion<double> baseRotation;
        if (setObjectOrientationTarget){
            if (listener->waitForTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time::now(),ros::Duration(1.5)))
            {
              tf::StampedTransform transform;
              listener->lookupTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time(0),transform);
              tf::quaternionTFToEigen(transform.getRotation(),baseRotation);
            }
        }
        else
            baseRotation.setIdentity();
    
        if (!doingGripperSegmentation)
        {
            std::cerr << "create tree\n";
            // this will create tree and normalize the orientation to the baseRotation
            createTree(segmentedObjectTree, objectDict, all_poses, ros::Time::now().toSec(), objectTFIndex, baseRotation);
        }
        else
        {
            std::cerr << "update one value on tree\n";
            updateOneValue(segmentedObjectTree, targetTFtoUpdate, objectDict, all_poses, ros::Time::now().toSec(), objectTFIndex, baseRotation);
        }
    }
    else
    {
        std::cerr << "update tree\n";
        updateTree(segmentedObjectTree, objectDict, all_poses, ros::Time::now().toSec(), objectTFIndex);
    }
    return getAllPoses(segmentedObjectTree);
}

bool semanticSegmentation::getAndSaveTable (const sensor_msgs::PointCloud2 &pc)
{
    std::string tableTFname, tableTFparent;
    nh.param("tableTF", tableTFname,std::string("/tableTF"));
    
    listener->getParent(tableTFname,ros::Time(0),tableTFparent);
    if (listener->waitForTransform(tableTFparent,tableTFname,ros::Time::now(),ros::Duration(1.5)))
    {
        std::cerr << "Table TF with name: '" << tableTFname << "' found with parent frame: " << tableTFparent << std::endl;
        tf::StampedTransform transform;
        listener->lookupTransform(tableTFparent,tableTFname,ros::Time(0),transform);
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        
        fromROSMsg(inputCloud,*full_cloud);
        std::cerr << "PCL organized: " << full_cloud->isOrganized() << std::endl;
        volumeSegmentation(full_cloud,transform,0.5);
        tableConvexHull = getTableConvexHull(full_cloud);
        if (tableConvexHull->size() < 10) {
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
        std::cerr << "Fail to get table TF with name: '" << tableTFname << "'" << std::endl;
        std::cerr << "Parent: " << tableTFparent << std::endl;
        return false;
    }
}

void semanticSegmentation::updateCloudData (const sensor_msgs::PointCloud2 &pc)
{
    if (!classReady) return;
    // The callback from main only update the cloud data
    inputCloud = pc;
    if (!haveTable)
    {
        haveTable = getAndSaveTable(pc);
    }
}

void semanticSegmentation::populateTFMapFromTree()
{
  ros::param::del("/instructor_landmark/objects");
  
  std::vector<value> sp_segmenter_detectedPoses = getAllNodes(segmentedObjectTree);
  segmentedObjectTFMap.clear();
  costar_objrec_msgs::DetectedObjectList object_list;
  object_list.header.seq = ++(this->number_of_segmentation_done);
  object_list.header.stamp = ros::Time::now();
  object_list.header.frame_id =  inputCloud.header.frame_id;
  for (std::size_t i = 0; i < sp_segmenter_detectedPoses.size(); i++)
  {
    const value * v = &sp_segmenter_detectedPoses.at(i);
    const poseT * p = &(std::get<1>(*v).pose);
    const std::string * objectTFname = &(std::get<1>(*v).tfName);
    segmentedObjectTF objectTmp(*p,*objectTFname);
    segmentedObjectTFMap[objectTmp.TFnames] = objectTmp;
    std::stringstream ss;
    ss << "/instructor_landmark/objects/" << p->model_name << "/" << &(std::get<1>(*v).index);
    
    ros::param::set(ss.str(), objectTmp.TFnames);
    std::stringstream ss2;
    ss2 << &(std::get<1>(*v).tfName);
    costar_objrec_msgs::DetectedObject object_tmp;
  	object_tmp.id = ss2.str();
  	object_tmp.symmetry.x_rotation = objectDict[ p->model_name ].roll;
  	object_tmp.symmetry.y_rotation = objectDict[ p->model_name ].pitch;
  	object_tmp.symmetry.z_rotation = objectDict[ p->model_name ].yaw;
  	object_tmp.symmetry.x_symmetries = std::floor(360.0 / objectDict[ p->model_name ].roll);
  	object_tmp.symmetry.y_symmetries = std::floor(360.0 / objectDict[ p->model_name ].pitch);
  	object_tmp.symmetry.z_symmetries = std::floor(360.0 / objectDict[ p->model_name ].yaw);
  	object_tmp.object_class = p->model_name;
  	object_list.objects.push_back(object_tmp);
  }

  detected_object_pub.publish(object_list);
}

bool semanticSegmentation::serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (!classReady) return false;
    // Service call will run SPSegmenter
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
    
    if (!haveTable)
    {
        std::cerr << "Error, class does not have table data yet. Return...\n";
        return false; // still does not have table
    }
    
    fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available";
        return false;
    }
    
    segmentCloudAboveTable(full_cloud, tableConvexHull, aboveTable);
    
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available after removing all object outside the table.\nPut some object above the table.\n";
        return false;
    }
    
    // get all poses from spSegmenterCallback
    std::vector<poseT> all_poses = spSegmenterCallback(full_cloud,*final_cloud);
    
    
    //publishing the segmented point cloud
    sensor_msgs::PointCloud2 output_msg;
    toROSMsg(*final_cloud,output_msg);
    output_msg.header.frame_id = inputCloud.header.frame_id;
    pc_pub.publish(output_msg);
    
    if (all_poses.size() < 1) {
        std::cerr << "Fail to segment objects on the table.\n";
        return false;
    }
  
    this->populateTFMapFromTree();
  
    std::cerr << "Segmentation done.\n";
    
    hasTF = true;
    return true;
}

bool semanticSegmentation::serviceCallbackGripper (sp_segmenter::segmentInGripper::Request & request, sp_segmenter::segmentInGripper::Response& response)
{
    std::cerr << "Segmenting object on gripper...\n";
    bool bestPoseOriginal = bestPoseOnly;
    bestPoseOnly = true; // only get best pose when segmenting object in gripper;
    targetTFtoUpdate = request.tfToUpdate;
    this->doingGripperSegmentation = true;
  
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    pcl::PointCloud<PointLT>::Ptr final_cloud(new pcl::PointCloud<PointLT>());
    std::string segmentFail("Object in gripper segmentation fails.");
    
    fromROSMsg(inputCloud,*full_cloud); // convert to PCL format
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available";
        response.result = segmentFail;
        bestPoseOnly = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    
    if (listener->waitForTransform(inputCloud.header.frame_id,gripperTF,ros::Time::now(),ros::Duration(1.5)))
    {
        tf::StampedTransform transform;
        listener->lookupTransform(inputCloud.header.frame_id,gripperTF,ros::Time(0),transform);
        // do a box segmentation around the gripper (50x50x50 cm)
        volumeSegmentation(full_cloud,transform,0.25,false);
        std::cerr << "Volume Segmentation done.\n";
    }
    else
    {
        std::cerr << "Fail to get transform between: "<< gripperTF << " and "<< inputCloud.header.frame_id << std::endl;
        response.result = segmentFail;
        bestPoseOnly = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    
    if (full_cloud->size() < 1){
        std::cerr << "No cloud available around gripper. Make sure the object can be seen by the camera.\n";
        bestPoseOnly = bestPoseOriginal;
        this->doingGripperSegmentation = false;
        return false;
    }
    // get best poses from spSegmenterCallback
    std::vector<poseT> all_poses = spSegmenterCallback(full_cloud,*final_cloud);
    
    if (all_poses.size() < 1) {
        std::cerr << "Fail to segment the object around gripper.\n";
        response.result = segmentFail;
        bestPoseOnly = bestPoseOriginal;
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
    bestPoseOnly = bestPoseOriginal;
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
        for (std::map<std::string, segmentedObjectTF>::iterator it=segmentedObjectTFMap.begin(); it!=segmentedObjectTFMap.end(); ++it)
        {
            br.sendTransform(
                             it->second.generateStampedTransform(parent)
                             );
        }
    }
}