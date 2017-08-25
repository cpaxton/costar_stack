#include "sp_segmenter/ros_semantic_segmentation.h"
#include <pcl/filters/crop_box.h>
#include <tf_conversions/tf_eigen.h>
#include "sp_segmenter/stringVectorArgsReader.h"

segmentedObjectTF::segmentedObjectTF()
{
    this->transform.setIdentity();
    this->TFname = "dummyTF";
}

segmentedObjectTF::segmentedObjectTF(const ObjectTransformInformation &input)
{
    this->transform.setOrigin(tf::Vector3( input.origin_.x(), input.origin_.y(), input.origin_.z() ));
    this->transform.setRotation(tf::Quaternion(
                                             input.rotation_.x(),input.rotation_.y(),input.rotation_.z(),input.rotation_.w()));
    this->TFname = input.transform_name_;
}

tf::StampedTransform segmentedObjectTF::generateStampedTransform(const std::string &parent) const
{
    return tf::StampedTransform(this->transform,ros::Time::now(),parent, this->TFname);
}


// Load in parameters for objects from ROS namespace
std::map<std::string, ObjectSymmetry> fillObjectPropertyDictionary(const ros::NodeHandle &nh, const std::vector<std::string> &cur_name)
{
    std::map<std::string, ObjectSymmetry> objectDict;
    ROS_INFO("Loading object symmetric properties");
    for (unsigned int i = 0; i < cur_name.size(); i++) {
        ROS_INFO("Found symmetric property for object: %s",cur_name[i].c_str());

        double r, p, y, step;
        std::string preferred_axis;
        nh.param(cur_name.at(i)+"/x", r, 360.0);
        nh.param(cur_name.at(i)+"/y", p, 360.0);
        nh.param(cur_name.at(i)+"/z", y, 360.0);
        nh.param(cur_name.at(i)+"/preferred_step", step, 360.0);
        nh.param(cur_name.at(i)+"/preferred_axis", preferred_axis, std::string("z"));

        objectDict[cur_name.at(i)] = ObjectSymmetry(r, p, y, preferred_axis, step);
    }
    return objectDict;
}

// Load in parameters for objects from ROS namespace
std::map<std::string, ObjectSymmetry> fillObjectPropertyDictionary(std::map<std::string, ModelObjRecRANSACParameter> &modelObjRecRansacParamDict,const ros::NodeHandle &nh, const std::vector<std::string> &cur_name)
{
    std::map<std::string, ObjectSymmetry> objectDict;
    ROS_INFO("Loading object symmetric properties");

    double default_pair_width, default_voxel_size, default_scene_visibility, default_object_visibility;
    nh.param("default_object_param/pair_width", default_pair_width, 0.005);
    nh.param("default_object_param/voxel_size", default_voxel_size, 0.004);
    nh.param("default_object_param/scene_visibility", default_scene_visibility, 0.1);
    nh.param("default_object_param/object_visibility", default_object_visibility, 0.1);

    for (unsigned int i = 0; i < cur_name.size(); i++) {
        ROS_INFO("Found symmetric property for object: %s",cur_name[i].c_str());

        double r, p, y, step;
        std::string preferred_axis;
        nh.param(cur_name.at(i)+"/x", r, 360.0);
        nh.param(cur_name.at(i)+"/y", p, 360.0);
        nh.param(cur_name.at(i)+"/z", y, 360.0);
        nh.param(cur_name.at(i)+"/preferred_step", step, 360.0);
        nh.param(cur_name.at(i)+"/preferred_axis", preferred_axis, std::string("z"));
        objectDict[cur_name.at(i)] = ObjectSymmetry(r, p, y, preferred_axis, step);

        double pair_width, voxel_size, scene_visibility, object_visibility;
        nh.param(cur_name.at(i)+"/pair_width", pair_width, default_pair_width);
        nh.param(cur_name.at(i)+"/voxel_size", voxel_size, default_voxel_size);
        nh.param(cur_name.at(i)+"/scene_visibility", scene_visibility, default_scene_visibility);
        nh.param(cur_name.at(i)+"/object_visibility", object_visibility, default_object_visibility);
        modelObjRecRansacParamDict[cur_name.at(i)] = ModelObjRecRANSACParameter(pair_width, voxel_size, scene_visibility, object_visibility);
    }
    return objectDict;
}

RosSemanticSegmentation::RosSemanticSegmentation() : hasTF(false), has_crop_box_pose_table_(false)
{}

void RosSemanticSegmentation::setNodeHandle(const ros::NodeHandle &nh)
{
    this->nh = nh;
    this->initializeSemanticSegmentationFromRosParam(); // initialize all variables before doing semantic segmentation
    this->number_of_segmentation_done = 0;
}

RosSemanticSegmentation::RosSemanticSegmentation(const ros::NodeHandle &nh) : hasTF(false), has_crop_box_pose_table_(false)
{  
    this->setNodeHandle(nh);
}

void RosSemanticSegmentation::initializeSemanticSegmentationFromRosParam()
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

    this->nh.param("useCropBox",this->use_crop_box_,true);
    
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
    bool loadTable, useTableSegmentation;
    double aboveTableMin, aboveTableMax, tableDistanceThreshold,tableAngularThreshold, tableMinimalInliers;
    this->nh.param("useTableSegmentation",useTableSegmentation,true);
    this->nh.param("aboveTableMin", aboveTableMin, 0.01);
    this->nh.param("aboveTableMax", aboveTableMax, 0.25);
    this->nh.param("loadTable",loadTable, false);
    this->nh.param("tableDistanceThreshold",tableDistanceThreshold,0.02);
    this->nh.param("tableAngularThreshold",tableAngularThreshold,2.0);
    this->nh.param("tableMinimalInliers",tableMinimalInliers,5000.0);
    this->setUseTableSegmentation(useTableSegmentation);
    this->setCropAboveTableBoundary(aboveTableMin,aboveTableMax);
    if(loadTable && useTableSegmentation)
    {
        std::string load_directory;
        nh.param("saveTable_directory",load_directory,std::string("./data"));
        this->loadTableFromFile(load_directory+"/table.pcd");
    }
    this->setTableSegmentationParameters(tableDistanceThreshold,tableAngularThreshold,tableMinimalInliers);

    // Setting up visualization
    bool visualization;
    this->nh.param("visualization",visualization,true);
    this->setUseVisualization(visualization);

#ifdef USE_OBJRECRANSAC
    // Setting up ObjRecRANSAC
    bool compute_pose, use_cuda, use_object_persistence, use_external_segmentation;
    double  minConfidence, max_distance_persistence;
    std::string objRecRANSACdetector;
    this->nh.param("compute_pose",compute_pose,true);
    this->nh.param("use_cuda", use_cuda,true);
    this->nh.param("objRecRANSACdetector", objRecRANSACdetector, std::string("StandardRecognize"));
    this->nh.param("minConfidence", minConfidence, 0.0);
    this->nh.param("useObjectPersistence",use_object_persistence,false);
    this->nh.param("max_distance_persistence",max_distance_persistence,0.025);
    this->nh.param("use_external_segmentation", use_external_segmentation, false);

    this->setUseComputePose(compute_pose);
    this->setUseCuda(use_cuda);
    if      (objRecRANSACdetector == "StandardBest")      this->setModeObjRecRANSAC(STANDARD_BEST);
    else if (objRecRANSACdetector == "GreedyRecognize")   this->setModeObjRecRANSAC(GREEDY_RECOGNIZE);
    else if (objRecRANSACdetector == "StandardRecognize") this->setModeObjRecRANSAC(STANDARD_RECOGNIZE);
    else ROS_ERROR("Unsupported objRecRANSACdetector!");
    this->setMinConfidenceObjRecRANSAC(minConfidence);
    this->setUseObjectPersistence(use_object_persistence);
    this->setPoseConsistencyMaximumDistance(max_distance_persistence);
    this->setUseExternalSegmentation(use_external_segmentation);
    if (use_external_segmentation)
    {
        std::string external_segmentation_done_topic;
        this->nh.param("external_segment_done_topic",external_segmentation_done_topic,std::string("/costar_perception/segment_done"));
        external_segmentation_sub = nh.subscribe(external_segmentation_done_topic,1,&RosSemanticSegmentation::processExternalSegmentationResult,this);
    }

    std::string mesh_path;
    //get parameter for mesh path and cur_name
    nh.param("mesh_path", mesh_path,std::string("data/mesh/"));
    std::vector<std::string> cur_name = stringVectorArgsReader(nh, "cur_name", std::string("drill"));
    //get object parameters
    std::map<std::string, ModelObjRecRANSACParameter> model_obj_ransac_parameter;
    std::map<std::string, ObjectSymmetry> objectDict = fillObjectPropertyDictionary(model_obj_ransac_parameter, nh, cur_name);
    // Add model to the semantic segmentation
    for (std::vector<std::string>::const_iterator it = cur_name.begin(); it != cur_name.end(); ++it)
    {
        std::string object_full_path = mesh_path+*it;
        this->addModel(mesh_path,*it,model_obj_ransac_parameter[*it]);
    }
    this->addModelSymmetricProperty(objectDict);
#else
    ROS_WARN("Could not compute pose because the library is not compiled with USE_OBJRECRANSAC = ON.");
    ROS_INFO("This class will only publish the segmented cloud.");
#endif

#ifdef USE_TRACKING
    this->nh.param("enableTracking",use_tracking_,false);
    if (use_tracking_)
    {
        tracker_ = boost::shared_ptr<Tracker>(new Tracker());
        std::vector<ModelT> mesh_set = this->getMeshSet();
        for(ModelT& model : mesh_set)
        {
            if(!tracker_->addTracker(model))
            {
                ROS_WARN("Tried to add duplicate model name to tracker");
            }
        }
    }
#endif

    this->initializeSemanticSegmentation();

    // ---------------------- SETTING UP ROS SPECIFIC PARAMETERS --------------------------------------
    // Ros specific parameters
    bool setObjectOrientation;
    this->nh.param("useTF",useTFinsteadOfPoses,true);
    this->nh.param("GripperTF",gripperTF,std::string("endpoint_marker"));
    this->nh.param("useMedianFilter",use_median_filter,true);
    this->nh.param("setObjectOrientation",setObjectOrientation,false);
    this->nh.param("preferredOrientation",targetNormalObjectTF,std::string("/world"));
    this->nh.param("POINTS_IN", POINTS_IN,std::string("/camera/depth_registered/points"));
    this->nh.param("POINTS_OUT", POINTS_OUT,std::string("points_out"));
    listener = new (tf::TransformListener);

    if (useTableSegmentation)
        table_corner_pub = nh.advertise<sensor_msgs::PointCloud2>("table_corner",3);
    
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(POINTS_OUT,1000);

    if (!useTFinsteadOfPoses)
    {
        ROS_INFO("Node publish pose array.");
        pose_pub = nh.advertise<geometry_msgs::PoseArray>(POSES_OUT,1000);
        pc_sub = nh.subscribe(POINTS_IN,1,&RosSemanticSegmentation::callbackPoses,this);
    }
    else
    {
        ROS_INFO("Node publish TF frames.");
        spSegmenter = this->nh.advertiseService("SPSegmenter",&RosSemanticSegmentation::serviceCallback,this);
#ifdef COSTAR
        segmentGripper = this->nh.advertiseService("segmentInGripper",&RosSemanticSegmentation::serviceCallbackGripper,this);
#endif
        pc_sub = this->nh.subscribe(POINTS_IN,1,&RosSemanticSegmentation::updateCloudData,this);
    }
#ifdef COSTAR
    detected_object_pub = nh.advertise<costar_objrec_msgs::DetectedObjectList>("detected_object_list",1);
#endif


#ifdef SCENE_PARSING
    hypothesis_pub_ = nh.advertise<objrec_hypothesis_msgs::AllModelHypothesis>("object_hypothesis",1);
    last_hypotheses_server_ = this->nh.advertiseService("GetLastHypothesis",&RosSemanticSegmentation::getLastHypotheses,this);
#endif

    this->nh.param("maxFrames",maxframes,15);
    cur_frame_idx = 0;
    cloud_ready = false;
    cloud_vec.clear();
    cloud_vec.resize(maxframes);
    this->hasTF = false;
    table_corner_published = 0;
    this->need_preferred_tf_ = setObjectOrientation;
    this->setUsePreferredOrientation(setObjectOrientation);
}

void RosSemanticSegmentation::callbackPoses(const sensor_msgs::PointCloud2 &inputCloud)
{
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    fromROSMsg(inputCloud,*full_cloud);

    this->setCropBoxSize(crop_box_size);
    this->setCropBoxPose(crop_box_pose_table_);
    if (need_preferred_tf_)
    {
        if (listener->frameExists(targetNormalObjectTF) &&
            listener->waitForTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time::now(),ros::Duration(5.0)))
        {
            listener->lookupTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time(0),preferred_transform);
            Eigen::Affine3d preferred_transform_eigen;
            tf::transformTFToEigen(preferred_transform, preferred_transform_eigen);
            Eigen::Quaterniond q(preferred_transform_eigen.rotation());
            this->setPreferredOrientation(q);
            this->need_preferred_tf_ = false;
        }
        else
        {
            ROS_WARN("Cannot find preferred orientation frame");
            return;
        }
    }

    pcl::PointCloud<PointLT>::Ptr labelled_point_cloud_result;
#ifdef USE_OBJRECRANSAC
    std::vector<ObjectTransformInformation> object_transform_result;
    // returns true if the segmentation is successful
    if (this->segmentAndCalculateObjTransform(full_cloud,labelled_point_cloud_result,object_transform_result))
    {
        pcl::PointCloud<PointT>::Ptr segmented_cloud;
        this->convertPointCloudLabelToRGBA(labelled_point_cloud_result,segmented_cloud);

        //publishing the segmented point cloud
        sensor_msgs::PointCloud2 output_msg;
        toROSMsg(*segmented_cloud,output_msg);
        output_msg.header.frame_id = inputCloud.header.frame_id;
        pc_pub.publish(output_msg);

        geometry_msgs::PoseArray msg;
        msg.header.frame_id = inputCloud.header.frame_id;
        for (std::vector<ObjectTransformInformation>::const_iterator it = object_transform_result.begin(); it != object_transform_result.end(); ++it)
        {
            const ObjectTransformInformation &p = *it;
            geometry_msgs::Pose pmsg;
            pmsg.position.x = p.origin_.x();
            pmsg.position.y = p.origin_.y();
            pmsg.position.z = p.origin_.z();
            pmsg.orientation.x = p.rotation_.x();
            pmsg.orientation.y = p.rotation_.y();
            pmsg.orientation.z = p.rotation_.z();
            pmsg.orientation.w = p.rotation_.w();
            
            msg.poses.push_back(pmsg);
        }
        pose_pub.publish(msg);
    }
#else
    if (this->segmentPointCloud(full_cloud,labelled_point_cloud_result))
    {
        pcl::PointCloud<PointT>::Ptr segmented_cloud;
        this->convertPointCloudLabelToRGBA(labelled_point_cloud_result,segmented_cloud);

        //publishing the segmented point cloud
        sensor_msgs::PointCloud2 output_msg;
        toROSMsg(*segmented_cloud,output_msg);
        output_msg.header.frame_id = inputCloud.header.frame_id;
        pc_pub.publish(output_msg);
    }
#endif
}

void RosSemanticSegmentation::processExternalSegmentationResult(const std_msgs::Empty &input_msgs)
{
    std_srvs::Empty::Request dummy_req;
    std_srvs::Empty::Response dummy_res;
    this->serviceCallback(dummy_req,dummy_res);
}

bool RosSemanticSegmentation::getAndSaveTable (const sensor_msgs::PointCloud2 &pc)
{
    if (!has_crop_box_pose_table_ && use_crop_box_)
    {
        return false;
    }
    else
    {
        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        fromROSMsg(inputCloud,*full_cloud);
        if (use_crop_box_)
        {
            Eigen::Vector3f crop_box_size_table = crop_box_size;
            double tableTF_z_box;
            nh.param("cropBoxTableZ", tableTF_z_box,0.005);
            crop_box_size_table[2] = tableTF_z_box;
            this->setCropBoxSize(crop_box_size_table);
            this->setCropBoxPose(crop_box_pose_table_);
        }
        bool saveTable;
        std::string saveTable_directory;
        nh.param("updateTable",saveTable, true);
        nh.param("saveTable_directory",saveTable_directory,std::string("./data"));
        return this->getTableSurfaceFromPointCloud(full_cloud,saveTable,saveTable_directory);
    }
}

void RosSemanticSegmentation::updateCloudData (const sensor_msgs::PointCloud2 &pc)
{
    if (!this->class_ready_) return;
    // The callback from main only update the cloud data
    inputCloud = pc;

    if (need_preferred_tf_)
    {
        if (listener->frameExists(targetNormalObjectTF) &&
            listener->waitForTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time::now(),ros::Duration(5.0)))
        {
            listener->lookupTransform(inputCloud.header.frame_id,targetNormalObjectTF,ros::Time(0),preferred_transform);
            Eigen::Affine3d preferred_transform_eigen;
            tf::transformTFToEigen(preferred_transform, preferred_transform_eigen);
            Eigen::Quaterniond q(preferred_transform_eigen.rotation());
            this->setPreferredOrientation(q);
            this->need_preferred_tf_ = false;
        }
        else
        {
            ROS_WARN("Cannot find preferred orientation frame");
            return;
        }
    }

    if (!has_crop_box_pose_table_ && use_crop_box_)
    {
        std::string tableTFname, tableTFparent;
        nh.param("tableTF", tableTFname,std::string("/tableTF"));
        
        tableTFparent = pc.header.frame_id;
        ROS_INFO("Looking for Table frame [%s] with parent frame [%s]",(tableTFname.c_str(), tableTFparent.c_str()));
        if (listener->frameExists(tableTFname) &&
            listener->waitForTransform(tableTFparent,tableTFname,ros::Time::now(),ros::Duration(1.5)))
        {
            ROS_INFO("Table frame found");
            listener->lookupTransform(tableTFparent,tableTFname,ros::Time(0),table_transform);
            tf::transformTFToEigen(table_transform, this->crop_box_pose_table_);
            Eigen::Quaterniond q(this->crop_box_pose_table_.rotation());
            Eigen::Vector3d t(this->crop_box_pose_table_.translation());
            printf("Q: %f %f %f %f\tT: %f %f %f\n", q.w(), q.x(), q.y(), q.z(), t.x(), t.y(), t.z());
            this->has_crop_box_pose_table_ = true;
            this->setUseCropBox(true);
            ROS_INFO("Crop Box activated");
        }
        else
            ROS_INFO("Fail to find table frame.");
    }

    if (this->use_table_segmentation_)
    {
        if (!this->have_table_) getAndSaveTable(inputCloud);
        
        if (this->have_table_) {

             // publish the table corner every 15 frame of inputCloud
            if (table_corner_published == 0)
            {
                sensor_msgs::PointCloud2 output_msg;
                toROSMsg(*table_corner_points_,output_msg);
                output_msg.header.frame_id = inputCloud.header.frame_id;
                // ROS_INFO("Published table corner point cloud");
                table_corner_pub.publish(output_msg);
            }
            else 
            {
                table_corner_published++;
                if (table_corner_published > 15) table_corner_published = 0;
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
        if( cur_frame_idx >= maxframes )
        {
            cloud_ready = true;
            cur_frame_idx = 0; 
        }
    }
}

void RosSemanticSegmentation::populateTFMap(std::vector<ObjectTransformInformation> all_poses)
{
    ros::param::del("/instructor_landmark/objects");

    // segmentedObjectTFMap.clear();

#ifdef COSTAR
    costar_objrec_msgs::DetectedObjectList object_list;
    object_list.header.seq = ++(this->number_of_segmentation_done);
    object_list.header.stamp = ros::Time::now();
    object_list.header.frame_id =  inputCloud.header.frame_id;
#endif

    segmentedObjectTFV.clear();
    
    for (std::vector<ObjectTransformInformation>::const_iterator it = all_poses.begin(); it != all_poses.end(); ++it)
    {
        const segmentedObjectTF &segmented_object = *it;
        segmentedObjectTFV.push_back(segmented_object);

#ifdef COSTAR
        std::stringstream ss;
        ss << "/instructor_landmark/objects/" << it->model_name_ << "/" << it->model_index_;

        ros::param::set(ss.str(), segmented_object.TFname);
        costar_objrec_msgs::DetectedObject object_tmp;
    	object_tmp.id = segmented_object.TFname;
    	object_tmp.symmetry.x_rotation = this->object_dict_[ it->model_name_ ].roll;
    	object_tmp.symmetry.y_rotation = this->object_dict_[ it->model_name_ ].pitch;
    	object_tmp.symmetry.z_rotation = this->object_dict_[ it->model_name_ ].yaw;
    	object_tmp.symmetry.x_symmetries = std::floor(2 * M_PI / this->object_dict_[ it->model_name_ ].roll);
    	object_tmp.symmetry.y_symmetries = std::floor(2 * M_PI / this->object_dict_[ it->model_name_ ].pitch);
    	object_tmp.symmetry.z_symmetries = std::floor(2 * M_PI / this->object_dict_[ it->model_name_ ].yaw);
    	object_tmp.object_class = it->model_name_;
    	object_list.objects.push_back(object_tmp);
#endif
    }

#ifdef COSTAR
    this->last_object_list_ = object_list;
    detected_object_pub.publish(object_list);
#endif
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

bool RosSemanticSegmentation::serviceCallback (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    if (!this->class_ready_) {
        ROS_ERROR("Class is not ready!");
        return false;
    }
    else if (this->need_preferred_tf_)
    {
        ROS_ERROR("Does not have preferred frame yet. Cannot do semantic segmentation.");
        return false;
    }
    // Service call will run SPSegmenter
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    
    if (!use_median_filter)  // not using median filter
    {
        fromROSMsg(inputCloud,*full_cloud);
    }
    else if(cloud_ready == true )
    {
        ROS_INFO("Averaging point clouds");
        full_cloud = MedianPointCloud(cloud_vec);
        ROS_INFO("Averaging point clouds Done");
    }
    else
    {
        ROS_INFO("Need to accumulate more frames!");
        return false;
    }
    ROS_INFO("Segmenting object...");
    this->setCropBoxSize(crop_box_size);
    this->setCropBoxPose(crop_box_pose_table_);
    
    pcl::PointCloud<PointLT>::Ptr labelled_point_cloud_result;
#ifdef USE_OBJRECRANSAC
    std::vector<ObjectTransformInformation> object_transform_result;
    bool segmentation_success;
    if (!this->use_external_segmentation_)
    {
        segmentation_success = segmentAndCalculateObjTransform(full_cloud, labelled_point_cloud_result, object_transform_result);
    }
    else
    {
        // R channel of the point cloud is the label
        labelled_point_cloud_result = this->convertRgbChannelToLabelCloud(full_cloud, RED);
        object_transform_result = this->calculateObjTransform(labelled_point_cloud_result);
        segmentation_success = object_transform_result.size() > 0;
    }

    if (segmentation_success)
    {
        pcl::PointCloud<PointT>::Ptr segmented_cloud;
        this->convertPointCloudLabelToRGBA(labelled_point_cloud_result,segmented_cloud);
        //publishing the segmented point cloud
        sensor_msgs::PointCloud2 output_msg;
        toROSMsg(*segmented_cloud,output_msg);
        output_msg.header.frame_id = inputCloud.header.frame_id;
        pc_pub.publish(output_msg);

        ROS_INFO("Found %lu objects",object_transform_result.size());
        ROS_INFO("Segmentation Done.");

#ifdef USE_TRACKING
        if(use_tracking_)
        {
            std::vector<poseT> all_poses;
            for (std::vector<ObjectTransformInformation>::const_iterator it = object_transform_result.begin(); it!=object_transform_result.end(); ++it)
            {
                all_poses.push_back(it->asPoseT());
            }
            tracker_->generateTrackingPoints(inputCloud.header.stamp, all_poses);
        }
#endif

        if (object_transform_result.size() < 1) 
        {
            ROS_ERROR("Failed to find any objects on the table.");
            return false;
        }
        this->populateTFMap(object_transform_result);
        hasTF = true;

#ifdef SCENE_PARSING
        this->last_segmented_cloud_ = output_msg;
        this->hypothesis_pub_.publish(this->generateAllModelHypothesis());
#endif
        return true;
    }
    else
        return false;
#else
    if (this->segmentPointCloud(full_cloud,labelled_point_cloud_result))
    {
        pcl::PointCloud<PointT>::Ptr segmented_cloud;
        this->convertPointCloudLabelToRGBA(labelled_point_cloud_result,segmented_cloud);

        //publishing the segmented point cloud
        sensor_msgs::PointCloud2 output_msg;
        toROSMsg(*segmented_cloud,output_msg);
        output_msg.header.frame_id = inputCloud.header.frame_id;
        pc_pub.publish(output_msg);
        ROS_INFO("Segmentation Done.");
        return true;
    }
    else return false;
#endif
}

#if COSTAR
bool RosSemanticSegmentation::serviceCallbackGripper (sp_segmenter::SegmentInGripper::Request & request, sp_segmenter::SegmentInGripper::Response& response)
{
    if (!this->class_ready_) {
        ROS_ERROR("Class is not ready!");
        return false;
    }
    else if (this->need_preferred_tf_)
    {
        ROS_ERROR("Does not have preferred frame yet. Cannot do semantic segmentation.");
        return false;
    }
    ROS_INFO("Segmenting object on gripper...");
    int objRecRANSAC_mode_original = this->objRecRANSAC_mode_;
    // Use the detector for objects in the gripper
    std::string objRecRANSACdetector;
    nh.param("objRecRANSACdetectorInGripper",objRecRANSACdetector,std::string("StandardBest"));
    if      (objRecRANSACdetector == "StandardBest")      this->setModeObjRecRANSAC(STANDARD_BEST);
    else if (objRecRANSACdetector == "GreedyRecognize")   this->setModeObjRecRANSAC(GREEDY_RECOGNIZE);
    else if (objRecRANSACdetector == "StandardRecognize") this->setModeObjRecRANSAC(STANDARD_RECOGNIZE);
    
    std::string target_tf_to_update = request.tfToUpdate;
    std::string object_class = request.objectClass;

    std::string segmentFail("Object in gripper segmentation fails.");

    if (listener->frameExists(gripperTF) &&
        listener->waitForTransform(inputCloud.header.frame_id,gripperTF,ros::Time::now(),ros::Duration(1.5)))
    {
        tf::StampedTransform transform;
        listener->lookupTransform(inputCloud.header.frame_id,gripperTF,ros::Time(0),transform);

        Eigen::Affine3d gripper_tf;
        tf::transformTFToEigen(transform, gripper_tf);
        this->setUseCropBox(true);
        this->setCropBoxSize(crop_box_gripper_size);
        this->setCropBoxPose(gripper_tf);

        pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
        fromROSMsg(inputCloud,*full_cloud);
        pcl::PointCloud<PointLT>::Ptr labelled_point_cloud;

        if (segmentPointCloud(full_cloud, labelled_point_cloud))
        {
            pcl::PointCloud<PointT>::Ptr segmented_cloud;
            this->convertPointCloudLabelToRGBA(labelled_point_cloud,segmented_cloud);

            //publishing the segmented point cloud
            sensor_msgs::PointCloud2 output_msg;
            toROSMsg(*segmented_cloud,output_msg);
            output_msg.header.frame_id = inputCloud.header.frame_id;
            pc_pub.publish(output_msg);

#ifdef USE_OBJRECRANSAC
            std::vector<ObjectTransformInformation> object_transform_result = getUpdateOnOneObjTransform(labelled_point_cloud, target_tf_to_update, object_class);
            this->populateTFMap(object_transform_result);
            hasTF = true;
#endif
            this->setModeObjRecRANSAC(objRecRANSAC_mode_original);
            this->setUseCropBox(use_crop_box_);
            ROS_INFO("Object In gripper segmentation done.");
            return true;
        }
    }
    else
    {
        ROS_INFO("Fail to get transform between [%s] and [%s]",(gripperTF.c_str(), inputCloud.header.frame_id.c_str()));
        response.result = segmentFail;
    }
    this->setModeObjRecRANSAC(objRecRANSAC_mode_original);
    this->setUseCropBox(use_crop_box_);
    return false;
}
#endif

void RosSemanticSegmentation::publishTF()
{
    if (!useTFinsteadOfPoses) return; // do nothing
    if (hasTF)
    {
        std::string parent = inputCloud.header.frame_id;
        for (std::size_t i = 0; i < segmentedObjectTFV.size(); i++){
            br.sendTransform(
                segmentedObjectTFV.at(i).generateStampedTransform(parent)
                );
        }
    }
}

#ifdef SCENE_PARSING
objrec_hypothesis_msgs::AllModelHypothesis RosSemanticSegmentation::generateAllModelHypothesis() const
{
    std::vector<GreedyHypothesis> vec_greedy_hypo = this->getHypothesisList();
    objrec_hypothesis_msgs::AllModelHypothesis result;
    result.all_hypothesis.reserve(vec_greedy_hypo.size());

    pcl::KdTreeFLANN<pcl::PointXYZ> tf_coordinate_tree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tf_coordinate_points(new pcl::PointCloud<pcl::PointXYZ>());

    for (std::size_t i = 0; i < segmentedObjectTFV.size(); i++){
        const tf::Transform &t = segmentedObjectTFV[i].transform;
        pcl::PointXYZ p(t.getOrigin().x(), 
            t.getOrigin().y(), t.getOrigin().z());
        tf_coordinate_points->points.push_back(p);
    }
    tf_coordinate_tree.setInputCloud(tf_coordinate_points);
    Eigen::Quaternionf base_rotation = this->base_rotation_.template cast<float>  ();
    for (std::vector<GreedyHypothesis>::iterator it = vec_greedy_hypo.begin(); it != vec_greedy_hypo.end(); ++it)
    {
        std::map< std::size_t, std::vector<AcceptedHypothesisWithConfidence> > &object_hypotheses = it->by_object_hypothesis;
        for (std::map< std::size_t, std::vector<AcceptedHypothesisWithConfidence> >::iterator it_2 = object_hypotheses.begin();
            it_2 != object_hypotheses.end(); ++it_2)
        {
            objrec_hypothesis_msgs::ModelHypothesis object_i;
            object_i.model_hypothesis.reserve( it_2->second.size() );
            object_i.model_name = it->model_id;
            const ObjectSymmetry obj_sym = object_dict_.find(it->model_id)->second;
            for (std::vector<AcceptedHypothesisWithConfidence>::iterator it_3 = it_2->second.begin(); 
                it_3 != it_2->second.end(); ++it_3)
            {
                objrec_hypothesis_msgs::Hypothesis tmp;
                tmp.match = it_3->match;
                tmp.model_name = it_3->model_entry->getUserData()->getLabel();
                // transform: 12 double
                double *rigid_transform = it_3->rigid_transform;
                Eigen::Matrix3f rotation;
                rotation <<
                    rigid_transform[0], rigid_transform[1], rigid_transform[2],
                    rigid_transform[3], rigid_transform[4], rigid_transform[5], 
                    rigid_transform[6], rigid_transform[7], rigid_transform[8];
                Eigen::Quaternionf q(rotation);
                tmp.transform.translation.x = rigid_transform[9];
                tmp.transform.translation.y = rigid_transform[10];
                tmp.transform.translation.z = rigid_transform[11];

                q = normalizeModelOrientation<float>(q,base_rotation, obj_sym);
                tmp.transform.rotation.x = q.x();
                tmp.transform.rotation.y = q.y(); 
                tmp.transform.rotation.z = q.z(); 
                tmp.transform.rotation.w = q.w();

                tmp.confidence = it_3->confidence;
                object_i.model_hypothesis.push_back(tmp);

                if (it_3 == it_2->second.begin())
                {
                    pcl::PointXYZ p(rigid_transform[9], rigid_transform[10], rigid_transform[11]);
                    std::vector< int > k_indices(1);
                    std::vector< float > distances(1);
                    tf_coordinate_tree.nearestKSearch(p, 1, k_indices, distances);
                    std::size_t vector_index = k_indices[0];
                    object_i.tf_name = segmentedObjectTFV[vector_index].TFname;
                }
            }
            result.all_hypothesis.push_back(object_i);
        }
    }
    return result;
}

bool RosSemanticSegmentation::getLastHypotheses (std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    this->pc_pub.publish(last_segmented_cloud_);
    ros::Duration(0.5).sleep();
    this->detected_object_pub.publish(this->last_object_list_);
    ros::Duration(0.5).sleep();
    this->hypothesis_pub_.publish(this->generateAllModelHypothesis());
    ros::Duration(0.5).sleep();
    return true;
}

#endif
