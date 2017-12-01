#ifdef USE_OBJRECRANSAC
template <typename NumericType>
void SemanticSegmentation::addModelSymmetricProperty(const std::string &model_name, const NumericType &roll, const NumericType &pitch, const NumericType &yaw, const NumericType &step, const std::string &preferred_axis)
{
    ObjectSymmetry symmetric_property;
    object_dict_[model_name] = ObjectSymmetry(roll, pitch, yaw, preferred_axis, step);
}
template <typename NumericType>
void SemanticSegmentation::setMinConfidenceObjRecRANSAC(const NumericType &min_confidence)
{
    this->min_objrecransac_confidence = double(min_confidence);
}

template <typename NumericType>
void SemanticSegmentation::setPreferredOrientation(const Eigen::Quaternion<NumericType> &base_rotation)
{
    if (!this->use_preferred_orientation_) std::cerr << "WARNING: setUsePreferredOrientation is false. No orientation preference will be used\n";
    else
    {
        std::cerr << "Preferred orientation has been set.\n";
        this->base_rotation_ = base_rotation.template cast<double>  ();
    }
}
#endif

template <typename NumericType>
void SemanticSegmentation::setPointCloudDownsampleValue(const NumericType &down_ss)
{
    this->pcl_downsample_ = float(down_ss);
}

template <typename NumericType>
void SemanticSegmentation::setHierFeaRatio(const NumericType &ratio)
{
    this->hier_ratio_ = float(ratio);
}


template <typename NumericType>
void SemanticSegmentation::setPoseConsistencyMaximumDistance(const NumericType &distance)
{
    double max_distance = distance;
    segmented_object_tree_.setMaxDistance(max_distance);
}

template <typename NumericType>
void SemanticSegmentation::setCropBoxSize(const NumericType &x, const NumericType &y, const NumericType &z)
{
    this->crop_box_size_ = Eigen::Vector3f(x, y, z);
    this->crop_box_target_pose_.setIdentity();
}

template <typename NumericType>
void SemanticSegmentation::setCropBoxSize(const Eigen::Matrix<NumericType, 3, 1> &crop_box_size)
{
    this->crop_box_size_ = Eigen::Vector3f(crop_box_size[0], crop_box_size[1], crop_box_size[2]);
}

template <typename NumericType>
void SemanticSegmentation::setCropBoxPose(const Eigen::Transform< NumericType, 3, Eigen::Affine> &target_pose_relative_to_camera_frame)
{
    this->crop_box_target_pose_ = target_pose_relative_to_camera_frame.template cast<float> ();
}

template <typename NumericType>
void SemanticSegmentation::setCropAboveTableBoundary(const NumericType &min, const NumericType &max)
{
    this->above_table_min = double(min);
    this->above_table_max = double(max);
}

template <typename NumericType1, typename NumericType2, typename NumericType3>
void SemanticSegmentation::setTableSegmentationParameters(const NumericType1 &table_distance_threshold,const NumericType2 &table_angular_threshold,const NumericType3 &table_minimal_inliers)
{
    this->table_distance_threshold_ = double(table_distance_threshold);
    this->table_angular_threshold_ = table_angular_threshold;
    this->table_minimal_inliers_ = table_minimal_inliers;
}