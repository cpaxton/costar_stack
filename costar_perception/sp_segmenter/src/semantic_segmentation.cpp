#include "sp_segmenter/semantic_segmentation.h"

void ModelObjRecRANSACParameter::setPairWidth(const double &pair_width)
{
    this->pair_width_ = pair_width;
}

void ModelObjRecRANSACParameter::setVoxelSize(const double &voxel_size)
{
    this->voxel_size_ = voxel_size;
}

void ModelObjRecRANSACParameter::setObjectVisibility(const double &object_visibility)
{
    this->object_visibility_ = object_visibility;
}

void ModelObjRecRANSACParameter::setSceneVisibility(const double &scene_visibility)
{
    this->scene_visibility_ = scene_visibility;
}

ostream& operator<<(ostream& os, const objectTransformInformation &tf_info)
{
    os << "Found object: " << tf_info.model_name_ << " with ID: " << tf_info.transform_name_ 
        << ".\nObjRecRANSAC Confidence: " << tf_info.confidence_ 
        << "\nLocation: " << tf_info.origin_.x()  << ", " << tf_info.origin_.y()  << ", " << tf_info.origin_.z()  << "." 
        << "\nOrientation: "
        << tf_info.rotation_.w() << ", " << tf_info.rotation_.x() << ", " << tf_info.rotation_.y() << ", " << tf_info.rotation_.z() << ".\n";
    return os;
}

bool objectTransformInformation::operator==(const objectTransformInformation& other) const
{
    return this->transform_name_ == other.transform_name_
        && this->model_name_ == other.model_name_
        && this->model_index_ == other.model_index_
        && this->confidence_ == other.confidence_
        && this->origin_  == other.origin_
        && this->rotation_.isApprox(other.rotation_);
}

SemanticSegmentation::SemanticSegmentation() : class_ready_(false), visualizer_flag_(false), use_crop_box_(false), 
    use_binary_svm_(true), use_multi_class_svm_(false), number_of_added_models_(0), use_table_segmentation_(false), 
    pcl_downsample_(0.003), hier_ratio_(0.1), compute_pose_(false), use_combined_objRecRANSAC_(false), use_cuda_(false),
    use_shot_(true), use_fpfh_(false), use_sift_(false)
{
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
    std::copy(&color_label_tmp[0][0], &color_label_tmp[0][0]+11*3,&color_label[0][0]);
    this->hier_radius_ = 0.02;
    this->have_table_ = false;
    this->use_preferred_orientation_ = false;
    table_corner_points_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    this->objRecRANSAC_mode_ = 1;
    this->min_objrecransac_confidence = 0.0;
    this->base_rotation_.setIdentity();
    this->shot_loaded_ = false;
    this->fpfh_loaded_ = false;
    this->sift_loaded_ = false;
    this->svm_loaded_ = false;
    this->table_distance_threshold_ = 0.02;
    this->table_angular_threshold_ =  2.0;
    this->table_minimal_inliers_ =  5000;
}

void SemanticSegmentation::setDirectorySHOT(const std::string &path_to_shot_directory)
{
    bool success = checkFolderExist(path_to_shot_directory);
    if (!success)
    {
        std::cerr << "setDirectorySHOT failed" << std::endl;
        this->shot_loaded_ = false;
        return;
    }
    this->shot_loaded_ = true;
    std::cerr << "Loading SHOT...\n";

    std::string shot_path = path_to_shot_directory;
    if (shot_path.back() != '/')
        shot_path += "/";

    hie_producer = boost::shared_ptr<Hier_Pooler> (new Hier_Pooler(hier_radius_));
    hie_producer->LoadDict_L0(shot_path, "200", "200");
    hie_producer->setRatio(hier_ratio_);
    std::cerr << "Done.\n";
}

void SemanticSegmentation::setDirectoryFPFH(const std::string &path_to_fpfh_directory)
{
    bool success = checkFolderExist(path_to_fpfh_directory);
    if (!success)
    {
        std::cerr << "setDirectoryFPFH failed" << std::endl;
        this->fpfh_loaded_ = false;
        return;
    }
    this->fpfh_loaded_ = true;
    std::cerr << "Loading FPFH...\n";

    std::string fpfh_path = path_to_fpfh_directory;
    if (fpfh_path.back() != '/')
        fpfh_path += "/";

    fpfh_pooler_set.clear();
    fpfh_pooler_set.resize(2);
    fpfh_pooler_set[1] = boost::shared_ptr<Pooler_L0> (new Pooler_L0(-1));
    fpfh_pooler_set[1]->LoadSeedsPool(fpfh_path+"dict_fpfh_L0_400.cvmat");
    std::cerr << "Done.\n";
}

void SemanticSegmentation::setDirectorySIFT(const std::string &path_to_sift_directory)
{
    bool success = checkFolderExist(path_to_sift_directory);
    if (!success)
    {
        std::cerr << "setDirectorySIFT failed" << std::endl;
        this->sift_loaded_ = false;
        return;
    }
    this->sift_loaded_ = true;
    std::cerr << "Loading SIFT...\n";

    std::string sift_path = path_to_sift_directory;
    if (sift_path.back() != '/')
        sift_path += "/";

    sift_pooler_set.clear();
    sift_pooler_set.resize(2);
    sift_pooler_set[1] = boost::shared_ptr<Pooler_L0> (new Pooler_L0(-1));
    sift_pooler_set[1]->LoadSeedsPool(sift_path+"dict_sift_L0_400.cvmat"); 
    
    // sift paramters can be fixed like this, it won't change too much, to 
    // speed up you can reduce the range like [0.7, 1.6] to [0.8, 0.9]
    for( float sigma = 0.7 ; sigma <= 1.61 ; sigma += 0.1 )
    {   
        cv::SiftFeatureDetector *sift_det = new cv::SiftFeatureDetector(
            0, // nFeatures
            4, // nOctaveLayers
            -10000, // contrastThreshold 
            100000, //edgeThreshold
            sigma//sigma
            );
        sift_det_vec.push_back(sift_det);   
    }
    std::cerr << "Done.\n";
}


void SemanticSegmentation::setUseMultiClassSVM(const bool &use_multi_class_svm)
{
    this->use_multi_class_svm_ = use_multi_class_svm;
}

void SemanticSegmentation::setUseBinarySVM(const bool &use_binary_svm)
{
    this->use_binary_svm_ = use_binary_svm;
}

void SemanticSegmentation::setDirectorySVM(const std::string &path_to_svm_directory)
{
    bool success = checkFolderExist(path_to_svm_directory);
    if (!success)
    {
        std::cerr << "setDirectorySVM failed" << std::endl;
        this->svm_loaded_ = false;
        return;
    }
    else if ((use_binary_svm_ || use_multi_class_svm_) == false)
    {
        std::cerr << "Both setUseMultiClassSVM and setUseBinarySVM is false.\nsetDirectorySVM needs at least one of them to be true\n";
        return;
    }
    this->svm_loaded_ = true;

    binary_models_.resize(3);
    multi_models_.resize(3);

    std::cerr << "Loading SVM...\n";
    std::cerr << "Use Multi Class SVM = " << use_multi_class_svm_ << std::endl;
    std::cerr << "Use Background Foreground SVM = " << use_binary_svm_ << std::endl;

    std::string svm_path = path_to_svm_directory;
    if (svm_path.back() != '/')
        svm_path += "/";

    for( int ll = 0 ; ll < 3 ; ll++ )
    {
        std::stringstream ss;
        ss << ll;
        
        if (use_binary_svm_)
        {
            this->binary_models_[ll] = load_model((svm_path+"binary_L"+ss.str()+"_f.model").c_str());
            if (binary_models_[ll] == NULL)
            {
                // null pointer exception
                std::cerr << "Failed to load file: " << (svm_path+"binary_L"+ss.str()+"_f.model").c_str() << std::endl;
                this->svm_loaded_ = false;
            }
        }
        if (use_multi_class_svm_)
        {
            this->multi_models_[ll] = load_model((svm_path+"multi_L"+ss.str()+"_f.model").c_str());
            if (multi_models_[ll] == NULL)
            {
                // null pointer exception
                std::cerr << "Failed to load file: " << (svm_path+"multi_L"+ss.str()+"_f.model").c_str() << std::endl;
                this->svm_loaded_ = false;
            }
        }
    }

    std::cerr << "Done.\n";
}

void SemanticSegmentation::setDirectorySVM(const std::string &path_to_svm_directory, const bool &use_binary_svm, const bool &use_multi_class_svm)
{
    this->setUseBinarySVM(use_binary_svm);
    this->setUseMultiClassSVM(use_multi_class_svm);
    this->setDirectorySVM(path_to_svm_directory);
}

void SemanticSegmentation::setUseVisualization(const bool &visualization_flag)
{
    this->visualizer_flag_ = visualization_flag;
    // Initialize pcl viewer
    if( visualizer_flag_ )
    {
        viewer = pcl::visualization::PCLVisualizer::Ptr (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters();
        viewer->addCoordinateSystem(0.1);
        viewer->setCameraPosition(0, 0, 0.1, 0, 0, 1, 0, -1, 0);
        viewer->setSize(1280, 960);
    }
}
void SemanticSegmentation::setUseCropBox(const bool &use_crop_box)
{
    this->use_crop_box_ = use_crop_box;
}

void SemanticSegmentation::setUseTableSegmentation(const bool &use_table_segmentation)
{
    this->use_table_segmentation_ = use_table_segmentation;
}

void SemanticSegmentation::loadTableFromFile(const std::string &table_pcd_path)
{
    pcl::PCDReader reader;
    if( reader.read (table_pcd_path, *table_corner_points_) == 0){
        std::cerr << "Table load successfully\n";
        this->have_table_ = true;
        this->setUseTableSegmentation(true);
    }
    else {
        this->have_table_ = false;
        std::cerr << "Failed to load table. Remove all objects, put the ar_tag marker in the center of the table and it will get anew table data\n";
    }
}

void SemanticSegmentation::initializeSemanticSegmentation()
{
    if (this->svm_loaded_)
        std::cerr << "SVM loaded\n";
    else
        std::cerr << "Please set the SVM directory\n";

    if (this->shot_loaded_)
        std::cerr << "SHOT loaded\n";
    else
        std::cerr << "Please set the SHOT directory\n";

    if (this->compute_pose_)
    {
        std::cerr << "Number of loaded model = " << this->number_of_added_models_ << std::endl;
        if (this->number_of_added_models_ == 0)
            std::cerr << "No model has been loaded. Please add at least 1 model.\n";
        this->class_ready_ = (this->number_of_added_models_ > 0 && this->svm_loaded_ && this->shot_loaded_);
    }
    else
        this->class_ready_ = (this->svm_loaded_ && this->shot_loaded_);

    if (this->class_ready_)
        std::cerr << "Semantic segmentation has initialized properly\n";
    else
    {
        std::cerr << "Please resolve the problems before initializing semantic segmentation.\n";
        return;
    }

    if (!use_table_segmentation_) {
      std::cerr << "WARNING: not using table segmentation!\n";
    }

    if (this->compute_pose_)
    {
        std::cerr << "Semantic Segmentation is running with objRecRANSACdetector: ";
        switch (objRecRANSAC_mode_)
        {
            case STANDARD_BEST:
                std::cerr << "STANDARD_BEST \n";
                break;
            case STANDARD_RECOGNIZE:
                std::cerr << "STANDARD_RECOGNIZE \n";
                break;
            case GREEDY_RECOGNIZE:
                std::cerr << "GREEDY_RECOGNIZE \n";
                break;
        }
    }

    // Initialize lab pooler
    lab_pooler_set.resize(6);
    for( size_t i = 1 ; i < lab_pooler_set.size() ; i++ )
    {
        boost::shared_ptr<Pooler_L0> cur_pooler(new Pooler_L0);
        cur_pooler->setHSIPoolingParams(i);
        lab_pooler_set[i] = cur_pooler;
    }

    std::cerr << "Hier Feature Ratio = " << hier_ratio_ << std::endl;
    std::cerr << "Hier Feature Downsample = " << pcl_downsample_ << std::endl;
}

SemanticSegmentation::~SemanticSegmentation()
{
    if (this->svm_loaded_)
    {
        for( int ll = 0 ; ll < 3 ; ll++ )
        {
            free_and_destroy_model(&binary_models_[ll]);
            if (use_multi_class_svm_)
                free_and_destroy_model(&multi_models_[ll]);
        }
    }
}

bool SemanticSegmentation::getTableSurfaceFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, const bool &save_table_pcd, const std::string &save_directory_path)
{
    if (!this->use_table_segmentation_)
    {
        std::cerr << "use_table_segmentation is set to false. Please enable it first before trying to do table segmentation.\n";
        return false;
    }

    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    *full_cloud = *input_cloud;


    if(use_crop_box_) {
        cropPointCloud(full_cloud, crop_box_target_pose_.inverse(), crop_box_size_);
    }

    if( viewer )
    {
        viewer->setWindowName("Cropbox Screen");
        viewer->removeAllPointClouds();
        viewer->addPointCloud(full_cloud, "cropbox_scene");
        viewer->spin();
        viewer->removeAllPointClouds();
    }

    table_corner_points_ = getTableConvexHull(full_cloud, viewer, table_distance_threshold_, table_angular_threshold_,table_minimal_inliers_);
    if (table_corner_points_->size() < 3) {
        std::cerr << "Failed segmenting the table. Please check the input point cloud and the table segmentation parameters.\n";
        return false;
    }
    
    if (save_table_pcd){
        std::string save_table_directory = save_directory_path;
        if (checkFolderExist(save_table_directory))
        {
            if (save_table_directory.back() != '/')
                save_table_directory+= '/';
            pcl::PCDWriter writer;
            writer.write<PointT> (save_table_directory+"table.pcd", *table_corner_points_, true);
            std::cerr << "Saved table point cloud in : " << save_table_directory <<"table.pcd\n";
        }
        else
        {
            std::cerr << "Failed saving table corner pointcloud in "<< save_table_directory << std::endl;
        }
    }
    std::cerr << "Sucessfully segment the table.\n";
    this->have_table_ = true;
    return true;
}

bool SemanticSegmentation::segmentPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr &result)
{
    if (!this->class_ready_)
    {
        std::cerr << "Please initialize semantic segmentation first before doing point cloud segmentation\n";
        return false;
    }
    
    pcl::PointCloud<PointT>::Ptr full_cloud(new pcl::PointCloud<PointT>());
    *full_cloud = *input_cloud;

    if(use_crop_box_) {
      cropPointCloud(full_cloud, crop_box_target_pose_.inverse(), crop_box_size_);
    }

    if (full_cloud->size() < 1){
        std::cerr << "No cloud available after using crop box.\n";
        return false;
    }
    
    if( viewer )
    {
        viewer->setWindowName("Cropbox Screen");
        viewer->removeAllPointClouds();
        viewer->addPointCloud(full_cloud, "cropbox_scene");
        viewer->spin();
        viewer->removeAllPointClouds();
    }


    if (use_table_segmentation_)
    {
        if (!have_table_)
        {
            std::cerr << "Error. Does not has any table data yet, but use_table_segmentation_ flag is set to true\n."; 
            std::cerr << "Please do table segmentation first, or disable use_table_segmentation_\n.";
            return false;
        }
        else
        {
            segmentCloudAboveTable(full_cloud, table_corner_points_, above_table_min, above_table_max);

            if (full_cloud->size() < 1)
            {
                std::cerr << "No cloud available after removing all object outside the table. Put some objects above the table. \n";
                return false;
            }

            if( viewer )
            {
                viewer->setWindowName("Table Segmented Screen");
                viewer->removeAllPointClouds();
                viewer->addPointCloud(full_cloud, "whole_scene");
                viewer->spin();
                viewer->removeAllPointClouds();
            }
        }
    }

    spPooler triple_pooler;
    if (sift_loaded_  && use_sift_) triple_pooler.init(full_cloud, *hie_producer, hier_radius_, pcl_downsample_);
    else triple_pooler.lightInit(full_cloud, *hie_producer, hier_radius_, pcl_downsample_);
    
    std::cerr << "LAB Pooling!" << std::endl;
    if (shot_loaded_ && use_shot_) triple_pooler.build_SP_LAB(lab_pooler_set, false);
    if (fpfh_loaded_ && use_fpfh_) triple_pooler.build_SP_FPFH(fpfh_pooler_set, hier_radius_, false);
    if (sift_loaded_ && use_sift_) triple_pooler.build_SP_SIFT(sift_pooler_set, *hie_producer, sift_det_vec, false);

    if(use_binary_svm_)
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
            triple_pooler.InputSemantics(binary_models_[ll], ll, reset_flag, false);
        }

        triple_pooler.extractForeground(true);
    }

    // if has useMultiSVM flag and has more than one objects, do multi object classification
    if (use_multi_class_svm_ && mesh_set_.size() > 1)
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
           triple_pooler.InputSemantics(multi_models_[ll], ll, reset_flag, false);
        }
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud(new pcl::PointCloud<pcl::PointXYZL>());
    label_cloud = triple_pooler.getSemanticLabels();
    triple_pooler.reset();
    
    if( viewer )
    {
        std::cerr<<"Visualize after segmentation"<<std::endl;
        visualizeLabels(label_cloud, viewer, color_label);
        viewer->spin();
        viewer->removePointCloud("label_cloud");
    }
    result = label_cloud;

    return true;
}

void SemanticSegmentation::convertPointCloudLabelToRGBA(const pcl::PointCloud<pcl::PointXYZL>::Ptr &input, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &output) const
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (pcl::PointCloud<pcl::PointXYZL>::const_iterator it = input->begin(); it != input->end(); ++it)
    {
        pcl::PointXYZRGBA point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        point.r = color_label[it->label][0];
        point.b = color_label[it->label][1];
        point.g = color_label[it->label][2];
        point.a = 255;
        result->push_back(point);
    }
    output = result;
}


#ifdef USE_OBJRECRANSAC
void SemanticSegmentation::setUseComputePose(const bool &compute_pose)
{
    this->compute_pose_ = compute_pose;
}

void SemanticSegmentation::setUseCuda(const bool &use_cuda)
{
    this->use_cuda_ = use_cuda;
}

void SemanticSegmentation::setUseCombinedObjRecRANSAC(const bool &use_combined_objRecRANSAC)
{
    this->use_combined_objRecRANSAC_ = use_combined_objRecRANSAC;
}

void SemanticSegmentation::addModel(const std::string &path_to_model_directory, const std::string &model_name, const ModelObjRecRANSACParameter &parameter)
{
    bool success = checkFolderExist(path_to_model_directory);
    if (!success)
    {
        std::cerr << "addModel failed" << std::endl;
        return;
    }
    std::string model_path = path_to_model_directory;
    if (model_path.back() != '/')
        model_path += "/";

    if (use_combined_objRecRANSAC_ || !use_multi_class_svm_)
    {
        std::cerr << "Using combined ObjRecRANSAC.\n";
        if (combined_ObjRecRANSAC_ == NULL)
        {
            combined_ObjRecRANSAC_ = boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(parameter.pair_width_, parameter.voxel_size_));
            combined_ObjRecRANSAC_->setParams(parameter.object_visibility_,parameter.scene_visibility_);
            combined_ObjRecRANSAC_->setUseCUDA(use_cuda_);
        }
        combined_ObjRecRANSAC_->AddModel(model_path + model_name, model_name);
        this->number_of_added_models_++;
    }
    else
    {
        std::cerr << "Using individual ObjRecRANSAC for each model.\n";
        individual_ObjRecRANSAC_.push_back(boost::shared_ptr<greedyObjRansac>(new greedyObjRansac(parameter.pair_width_, parameter.voxel_size_)));
        individual_ObjRecRANSAC_[number_of_added_models_]->setParams(parameter.object_visibility_,parameter.scene_visibility_);
        individual_ObjRecRANSAC_[number_of_added_models_]->setUseCUDA(use_cuda_);
        individual_ObjRecRANSAC_[number_of_added_models_]->AddModel(model_path + model_name, model_name);
        model_name_map_[model_name] = number_of_added_models_;
        this->number_of_added_models_++;
    }
    ModelT mesh_buf = LoadMesh(model_path + model_name, model_name);  
    mesh_set_.push_back(mesh_buf);
    object_class_transform_index_[model_name] = 0;
}


void SemanticSegmentation::addModelSymmetricProperty(const std::map<std::string, objectSymmetry> &object_dict)
{
    this->object_dict_ = object_dict;
}

void SemanticSegmentation::setModeObjRecRANSAC(const int &mode)
{
    this->objRecRANSAC_mode_ = mode;
}

void SemanticSegmentation::setUsePreferredOrientation(const bool &use_preferred_orientation)
{
    this->use_preferred_orientation_ = use_preferred_orientation;
}


// void SemanticSegmentation::setUsePreferredOrientation(const bool &use_preferred_orientation, const Eigen::Quaterniond &input_preferred_orientation)
// {
//     this->use_preferred_orientation_ = use_preferred_orientation;
//     this->base_rotation_ = input_preferred_orientation;
// }

void SemanticSegmentation::setUseObjectPersistence(const bool &use_object_persistence)
{
    this->use_object_persistence_ = use_object_persistence;
}

std::vector<objectTransformInformation> SemanticSegmentation::calculateObjTransform(const pcl::PointCloud<pcl::PointXYZL>::Ptr &labelled_point_cloud)
{
    if (!this->class_ready_ || !this->compute_pose_)
    {
        std::cerr << "Please set compute pose to true and initialize semantic segmentation before calculating obj transform\n";
        return std::vector<objectTransformInformation>();
    }

    std::vector<poseT> all_poses;

    if( viewer )
    {
        std::cerr<<"Visualize after pose computation"<<std::endl;
        viewer->setWindowName("Computed Pose");
        viewer->removeAllPointClouds();
        visualizeLabels(labelled_point_cloud, viewer, color_label);
    }

    if (use_multi_class_svm_ && !use_combined_objRecRANSAC_)
    {
        // cloud_set contains separated clouds based on the labelled cloud
        std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_set(number_of_added_models_+1);
        for( size_t j = 0 ; j < cloud_set.size() ; j++ )
            cloud_set[j] = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>()); // object cloud starts from 1
        std::cerr<<"Split cloud after segmentation"<<std::endl;
        splitCloud(labelled_point_cloud, cloud_set);

        std::cerr<<"Calculate poses"<<std::endl;

        // loop over all segmented object clouds
        #pragma omp parallel for schedule(dynamic, 1)
        for(size_t j = 1 ; j <= number_of_added_models_; j++ )
        {
            if( cloud_set[j]->empty() == false )
            {
                std::cerr << "cloud set " << j << " size: " << cloud_set[j]->size() << std::endl;
                std::vector<poseT> tmp_poses;
                switch (objRecRANSAC_mode_)
                {
                    case STANDARD_BEST:
                        individual_ObjRecRANSAC_[j-1]->StandardBest(cloud_set[j], tmp_poses);
                        break;
                    case STANDARD_RECOGNIZE:
                        individual_ObjRecRANSAC_[j-1]->StandardRecognize(cloud_set[j], tmp_poses, min_objrecransac_confidence);
                        break;
                    case GREEDY_RECOGNIZE:
                        individual_ObjRecRANSAC_[j-1]->GreedyRecognize(cloud_set[j], tmp_poses);
                        break;
                    default:
                        std::cerr << "Unsupported objRecRANSACdetector!\n";
                }

                if (viewer)
                {
                    individual_ObjRecRANSAC_[j-1]->visualize(viewer, tmp_poses, color_label[j]);
                }

                #pragma omp critical
                {
                    all_poses.insert(all_poses.end(), tmp_poses.begin(), tmp_poses.end());
                }
            }
        }

        if (viewer)
        {
            viewer->spin();
            for(size_t j = 1 ; j <= number_of_added_models_; j++ )
                individual_ObjRecRANSAC_[j-1]->clearMesh(viewer, all_poses);
            viewer->removeAllPointClouds();
        }
    }
    else 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*labelled_point_cloud,*scene_xyz);
        switch (objRecRANSAC_mode_)
        {
            case STANDARD_BEST:
                combined_ObjRecRANSAC_->StandardBest(scene_xyz, all_poses);
                break;
            case STANDARD_RECOGNIZE:
                combined_ObjRecRANSAC_->StandardRecognize(scene_xyz, all_poses, min_objrecransac_confidence);
                break;
            case GREEDY_RECOGNIZE:
                combined_ObjRecRANSAC_->GreedyRecognize(scene_xyz, all_poses);
                break;
            default:
                std::cerr << "Unsupported objRecRANSACdetector!\n";
        }

        if (viewer)
        {
            combined_ObjRecRANSAC_->visualize_m(viewer, all_poses, model_name_map_, color_label);
            viewer->spin();
            combined_ObjRecRANSAC_->clearMesh(viewer, all_poses);
            viewer->removeAllPointClouds();
        }
    }

    std::map<std::string, unsigned int> object_class_transform_index_no_persistence = object_class_transform_index_;
    std::map<std::string, unsigned int> &tmpTFIndex = object_class_transform_index_;
    double current_time = time(0);
    if (segmented_object_tree_.size() == 0 || !use_object_persistence_)
    {
        std::cerr << "create tree\n";
        // this will create tree and normalize the orientation to the base_rotation_
        createTree(segmented_object_tree_, object_dict_, all_poses, current_time, tmpTFIndex, base_rotation_);
    }
    else
    {
        std::cerr << "update tree\n";
        updateTree(segmented_object_tree_, object_dict_, all_poses, current_time, tmpTFIndex, base_rotation_);
    }

    std::vector<objectTransformInformation> result = this->getTransformInformationFromTree();

    // restore original index if not using object persistance
    if (!use_object_persistence_) tmpTFIndex = object_class_transform_index_no_persistence;
    return result;
}

std::vector<objectTransformInformation> SemanticSegmentation::getTransformInformationFromTree()  const
{
    std::vector<objectTransformInformation> result;
    std::vector<value> sp_segmenter_detected_poses = getAllNodes(segmented_object_tree_);

    std::cerr << "detected poses: " << sp_segmenter_detected_poses.size() << "\n";
    for (std::size_t i = 0; i < sp_segmenter_detected_poses.size(); i++)
    {
        const value &v = sp_segmenter_detected_poses.at(i);

        const poseT &pose = std::get<1>(v).pose;
        const std::string &transform_name_ = std::get<1>(v).tfName;
        const unsigned int &model_index = std::get<1>(v).index;
        result.push_back( objectTransformInformation(transform_name_, pose, model_index) );
    }

    return result;
}

std::vector<objectTransformInformation> SemanticSegmentation::getUpdateOnOneObjTransform(const pcl::PointCloud<pcl::PointXYZL>::Ptr &labelled_point_cloud, const std::string &transform_name, const std::string &object_type)
{
    if (!this->class_ready_)
    {
        std::cerr << "Please initialize semantic segmentation first before updating one obj transform\n";
        return std::vector<objectTransformInformation>();
    }    
    std::vector<poseT> all_poses;
    if (use_multi_class_svm_)
    {
        // cloud_set contains separated clouds based on the labelled cloud
        std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > cloud_set(number_of_added_models_+1);
        for( size_t j = 0 ; j < cloud_set.size() ; j++ )
            cloud_set[j] = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>()); // object cloud starts from 1
        std::cerr<<"Split cloud after segmentation"<<std::endl;
        splitCloud(labelled_point_cloud, cloud_set);

        std::cerr<<"Calculate poses"<<std::endl;
        std::vector<poseT> tmp_poses;
        std::size_t objrec_index = model_name_map_[object_type];

        if( cloud_set[objrec_index + 1]->empty() == false )
        {
            std::vector<poseT> tmp_poses;
            switch (objRecRANSAC_mode_)
            {
                case STANDARD_BEST:
                    individual_ObjRecRANSAC_[objrec_index]->StandardBest(cloud_set[objrec_index + 1], tmp_poses);
                    break;
                case STANDARD_RECOGNIZE:
                    individual_ObjRecRANSAC_[objrec_index]->StandardRecognize(cloud_set[objrec_index + 1], tmp_poses, min_objrecransac_confidence);
                    break;
                case GREEDY_RECOGNIZE:
                    individual_ObjRecRANSAC_[objrec_index]->GreedyRecognize(cloud_set[objrec_index + 1], tmp_poses);
                    break;
                default:
                    std::cerr << "Unsupported objRecRANSACdetector!\n";
            }
            all_poses.insert(all_poses.end(), tmp_poses.begin(), tmp_poses.end());
        }
    }
    else 
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::copyPointCloud(*labelled_point_cloud,*scene_xyz);
        std::vector<poseT> tmp_poses;
        switch (objRecRANSAC_mode_)
        {
            case STANDARD_BEST:
                combined_ObjRecRANSAC_->StandardBest(scene_xyz, all_poses);
                break;
            case STANDARD_RECOGNIZE:
                combined_ObjRecRANSAC_->StandardRecognize(scene_xyz, all_poses, min_objrecransac_confidence);
                break;
            case GREEDY_RECOGNIZE:
                combined_ObjRecRANSAC_->GreedyRecognize(scene_xyz, all_poses);
                break;
            default:
                std::cerr << "Unsupported objRecRANSACdetector!\n";
        }
    }

    std::map<std::string, unsigned int> object_class_transform_index_no_persistence = object_class_transform_index_;
    std::map<std::string, unsigned int> &tmpTFIndex = object_class_transform_index_;
    double current_time = time(0);
    std::cerr << "update one value on tree\n";
    updateOneValue(segmented_object_tree_, transform_name, object_dict_, all_poses, current_time, tmpTFIndex, base_rotation_);

    std::vector<objectTransformInformation> result = this->getTransformInformationFromTree();

    // restore original index if not using object persistance
    if (!use_object_persistence_) tmpTFIndex = object_class_transform_index_no_persistence;
    return result;
}

bool SemanticSegmentation::segmentAndCalculateObjTransform(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &input_cloud, 
    pcl::PointCloud<pcl::PointXYZL>::Ptr &labelled_point_cloud_result, std::vector<objectTransformInformation> &object_transform_result)
{
    bool segmentation_successful = this->segmentPointCloud(input_cloud, labelled_point_cloud_result);
    if (segmentation_successful)
        object_transform_result = this->calculateObjTransform(labelled_point_cloud_result);
    
    return (segmentation_successful && object_transform_result.size() > 0);
}

#endif

void SemanticSegmentation::cropPointCloud(pcl::PointCloud<PointT>::Ptr &cloud_input, 
  const Eigen::Affine3f &camera_transform_in_target, 
  const Eigen::Vector3f &box_size) const
{
  pcl::PointCloud<PointT>::Ptr  cropped_cloud(new pcl::PointCloud<PointT>());
  pcl::CropBox<PointT> crop_box;
  crop_box.setKeepOrganized(true);
  crop_box.setInputCloud(cloud_input);
  crop_box.setMax(box_size.homogeneous());
  crop_box.setMin((-box_size).homogeneous());
  crop_box.setTransform(camera_transform_in_target);
  crop_box.filter(*cropped_cloud);
  cloud_input = cropped_cloud;
}

bool SemanticSegmentation::checkFolderExist(const std::string &directory_path) const
{
    bool result = boost::filesystem::is_directory(directory_path);
    if (!result)
    {
        std::cerr << "Folder: " << directory_path << " does not exist.\n";
    }
    return result;
}