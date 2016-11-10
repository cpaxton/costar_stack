#include "sp_segmenter/semanticSegmentation.h"
#include <pcl/filters/crop_box.h>
#include <tf_conversions/tf_eigen.h>

// Load in parameters for objects from ROS namespace
std::map<std::string, objectSymmetry> fillObjectPropertyDictionary(const ros::NodeHandle &nh, const std::vector<std::string> &cur_name)
{
    std::map<std::string, objectSymmetry> objectDict;
    std::cerr << "LOADING IN OBJECTS\n";
    for (unsigned int i = 0; i < cur_name.size(); i++) {
        std::cerr << "Name of obj: " << cur_name[i] << "\n";

        double r, p, y, step;
        std::string preferred_axis;
        nh.param(cur_name.at(i)+"/x", r, 360.0);
        nh.param(cur_name.at(i)+"/y", p, 360.0);
        nh.param(cur_name.at(i)+"/z", y, 360.0);
        nh.param(cur_name.at(i)+"/preferred_step", step, 360.0);
        nh.param(cur_name.at(i)+"/preferred_axis", preferred_axis, std::string("z"));

        objectDict[cur_name.at(i)] = objectSymmetry(r, p, y, preferred_axis, step);
    }
    return objectDict;
}

// Load in parameters for objects from ROS namespace
std::map<std::string, objectSymmetry> fillObjectPropertyDictionary(std::map<std::string, ModelObjRecRANSACParameter> &modelObjRecRansacParamDict,const ros::NodeHandle &nh, const std::vector<std::string> &cur_name)
{
    std::map<std::string, objectSymmetry> objectDict;
    std::cerr << "LOADING IN OBJECTS\n";

    double default_pair_width, default_voxel_size, default_scene_visibility, default_object_visibility;
    nh.param("default_object_param/pair_width", pair_width, 0.005);
    nh.param("default_object_param/voxel_size", voxel_size, 0.004);
    nh.param("default_object_param/scene_visibility", scene_visibility, 0.1);
    nh.param("default_object_param/object_visibility", object_visibility, 0.1);

    for (unsigned int i = 0; i < cur_name.size(); i++) {
        std::cerr << "Name of obj: " << cur_name[i] << "\n";

        double r, p, y, step;
        std::string preferred_axis;
        nh.param(cur_name.at(i)+"/x", r, 360.0);
        nh.param(cur_name.at(i)+"/y", p, 360.0);
        nh.param(cur_name.at(i)+"/z", y, 360.0);
        nh.param(cur_name.at(i)+"/preferred_step", step, 360.0);
        nh.param(cur_name.at(i)+"/preferred_axis", preferred_axis, std::string("z"));
        objectDict[cur_name.at(i)] = objectSymmetry(r, p, y, preferred_axis, step);

        double pair_width, voxel_size, scene_visibility, object_visibility;
        nh.param(cur_name.at(i)+"/pair_width", pair_width, default_pair_width);
        nh.param(cur_name.at(i)+"/voxel_size", voxel_size, default_voxel_size);
        nh.param(cur_name.at(i)+"/scene_visibility", scene_visibility, default_scene_visibility);
        nh.param(cur_name.at(i)+"/object_visibility", object_visibility, default_object_visibility);
        modelObjRecRansacParamDict[cur_name.at(i)] = ModelObjRecRANSACParameter(pair_width, voxel_size, scene_visibility, object_visibility);
    }
    return objectDict;
}

semanticSegmentation::semanticSegmentation() : hasTF(false)
{}

void semanticSegmentation::setNodeHandle(const ros::NodeHandle &nh)
{
    this->nh = nh;
    this->initializeSemanticSegmentationFromRosParam(); // initialize all variables before doing semantic segmentation
    this->number_of_segmentation_done = 0;
}

semanticSegmentation::semanticSegmentation(const ros::NodeHandle &nh)
{  
    this->setNodeHandle(nh);
}

void semanticSegmentation::initializeSemanticSegmentationFromRosParam()
{
    // ------------------- SETTING UP SEMANTIC SEGMENTATION --------------------
    // Setting up svm and shot
    std::string svm_path, shot_path;
    bool useBinarySVM, useMultiClassSVM;
    this->nh.param("useBinarySVM",useBinarySVM,false);
    this->nh.param("useMultiClassSVM",useMultiClassSVM,true);
    this->nh.param("svm_path", svm_path,std::string("data/UR5_drill_svm/"));
    this->nh.param("shot_path", shot_path,std::string("data/UW_shot_dict/"));
    this->setDirectorySHOT(shot_path);
    this->setUseBinarySVM(useBinarySVM);
    this->setUseMultiClassSVM(useMultiClassSVM);
    this->setDirectorySVM(svm_path);

    // setting up cropbox
    bool useCropBox;
    this->nh.param("useCropBox",useCropBox,true);
    this->setUseCropBox(useCropBox);
    double cropBoxX, cropBoxY, cropBoxZ;
    this->nh.param("cropBoxX",cropBoxX,1.0);
    this->nh.param("cropBoxY",cropBoxY,1.0);
    this->nh.param("cropBoxZ",cropBoxZ,1.0);
    this->crop_box_size = Eigen::Vector3f(cropBoxX,cropBoxY,cropBoxZ);
    this->nh.param("cropBoxGripperX",cropBoxX,1.0);
    this->nh.param("cropBoxGripperY",cropBoxY,1.0);
    this->nh.param("cropBoxGripperZ",cropBoxZ,1.0);
    this->crop_box_gripper_size = Eigen::Vector3f(cropBoxX, cropBoxY, cropBoxZ);

    // setting up table segmentation parameters
    double loadTable,useTableSegmentation,tableDistanceThreshold,tableAngularThreshold, aboveTableMin, aboveTableMax;
    this->nh.param("useTableSegmentation",useTableSegmentation,true);
    this->nh.param("aboveTableMin", aboveTableMin, 0.01);
    this->nh.param("aboveTableMax", aboveTableMax, 0.25);
    this->nh.param("loadTable",loadTable, true);
    this->nh.param("tableDistanceThreshold",tableDistanceThreshold,0.02);
    this->nh.param("tableAngularThreshold",tableAngularThreshold,2.0);
    this->nh.param("tableMinimalInliers",tableMinimalInliers,5000);
    this->setUseTableSegmentation(useTableSegmentation);
    this->setCropAboveTableBoundary(aboveTableMin,aboveTableMax);
    if(loadTable && useTableSegmentation)
    {
        std::string load_directory;
        nh.param("saveTable_directory",load_directory,std::string("./data"));
        this->loadTableFromFile(load_directory+"/table.pcd");
    }
    this->setTableSegmentationParameters(tableDistanceThreshold,tableAngularThreshold,tableMinimalInliers);

    // Setting up ObjRecRANSAC
    bool compute_pose, use_cuda, setObjectOrientationTarget,useObjectPersistence;
    double  minConfidence;
    std::string objRecRANSACdetector;
    this->nh.param("compute_pose",compute_pose,true);
    this->nh.param("use_cuda", use_cuda,true);
    this->nh.param("objRecRANSACdetector", objRecRANSACdetector, std::string("StandardRecognize"));
    else ROS_ERROR("Unsupported objRecRANSACdetector!");
    this->nh.param("minConfidence", minConfidence, 0.0);
    this->nh.param("setObjectOrientation",setObjectOrientationTarget,false);
    this->nh.param("useObjectPersistence",useObjectPersistence,false);
    this->setUseComputePose(compute_pose);
    this->setUseCuda(use_cuda);
    if      (objRecRANSACdetector == "StandardBest")      this->setModeObjRecRANSAC(STANDARD_BEST);
    else if (objRecRANSACdetector == "GreedyRecognize")   this->setModeObjRecRANSAC(GREEDY_RECOGNIZE);
    else if (objRecRANSACdetector == "StandardRecognize") this->setModeObjRecRANSAC(STANDARD_RECOGNIZE);
    this->setMinConfidenceObjRecRANSAC(minConfidence);
    this->setUsePreferredOrientation(setObjectOrientationTarget);
    this->setUseObjectPersistence(useObjectPersistence);

    if (useTableSegmentation) {
        table_corner_pub = nh.advertise<sensor_msgs::PointCloud2>("table_corner",3);
    
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);

    std::string mesh_path;
    //get parameter for mesh path and cur_name
    nh.param("mesh_path", mesh_path,std::string("data/mesh/"));
    std::vector<std::string> cur_name = stringVectorArgsReader(nh, "cur_name", std::string("drill"));
    //get object parameters
    std::map<std::string, ModelObjRecRANSACParameter> model_obj_ransac_parameter;
    objectDict = fillObjectPropertyDictionary(model_obj_ransac_parameter, nh, cur_name);
    // Add model to the semantic segmentation
    for (std::vector<std::string>::const_iterator it = cur_name.begin(); it != cur_name.end(); ++it)
    {
        std::string object_full_path = mesh_path+*it;
        this->addModel(mesh_path,*it,model_obj_ransac_parameter[*it]);
    }

    this->initializeSemanticSegmentation();

    // ---------------------- SETTING UP ROS SPECIFIC PARAMETERS --------------------------------------
    // Ros specific parameters
    this->nh.param("useTF",useTFinsteadOfPoses,true);
    this->nh.param("GripperTF",gripperTF,std::string("endpoint_marker"));
    this->nh.param("useMedianFilter",use_median_filter,true);
    this->nh.param("preferredOrientation",targetNormalObjectTF,std::string("/world"));
    this->nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
    this->nh.param("POINTS_OUT", POINTS_OUT,std::string("points_out"));

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
    this->nh.param("maxFrames",maxframes,15);
    cur_frame_idx = 0;
    cloud_ready = false;
    cloud_vec.clear();
    cloud_vec.resize(maxframes);
    this->hasTF = false;
    table_corner_published = 0;
}

void semanticSegmentation::callbackPoses(const sensor_msgs::PointCloud2 &inputCloud)
{
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    fromROSMsg(inputCloud,*full_cloud);

    this->setCropBoxSize(crop_box_size);

    pcl::PointCloud<PointLT>::Ptr labelled_point_cloud_result, std::vector<objectTransformInformation> object_transform_result;
    // returns true if the segmentation is successful
    if (segmentAndCalculateObjTransform(full_cloud,labelled_point_cloud_result,object_transform_result))
    {
        pcl::PointCloud<PointT>::Ptr segmented_cloud(new pcl::PointCloud<PointT>());
        pcl::copyPointCloud(*labelled_point_cloud,*segmented_cloud);

        //publishing the segmented point cloud
        sensor_msgs::PointCloud2 output_msg;
        toROSMsg(*segmented_cloud,output_msg);
        output_msg.header.frame_id = inputCloud.header.frame_id;
        pc_pub.publish(output_msg);

        geometry_msgs::PoseArray msg;
        msg.header.frame_id = inputCloud.header.frame_id;
        for (std::vector<objectTransformInformation>::const_iterator it = object_transform_result.begin(); it != object_transform_result.end(); ++it)
        {
            const objectTransformInformation &p = *it;
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
