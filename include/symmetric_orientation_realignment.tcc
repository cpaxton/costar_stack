template <typename numericStandard>
void realignOrientation (Eigen::Matrix<numericStandard, 3, 3> &rot_matrix, const ObjectSymmetry &object, 
    const int axis_to_align, const bool with_rotate_specific_axis, const int rotate_around_specific_axis)
{
    Eigen::Vector3f obj_axes[3];
    obj_axes[0] = Eigen::Vector3f(rot_matrix(0,0),rot_matrix(1,0),rot_matrix(2,0));
    obj_axes[1] = Eigen::Vector3f(rot_matrix(0,1),rot_matrix(1,1),rot_matrix(2,1));
    obj_axes[2] = Eigen::Vector3f(rot_matrix(0,2),rot_matrix(1,2),rot_matrix(2,2));

    Eigen::Vector3f axis(0,0,0);
    axis[axis_to_align] = 1;

    double dot_product = axis.dot(obj_axes[axis_to_align]);
    Eigen::Vector3f cross_product = obj_axes[axis_to_align].cross(axis);

    double angle = std::atan2(cross_product.norm(),dot_product); //since the vector is unit vector
    if (angle < 0) angle += 2 * pi;
    
    Eigen::Vector3f best_axis[3];
    best_axis[0] = Eigen::Vector3f(1,0,0);
    best_axis[1] = Eigen::Vector3f(0,1,0);
    best_axis[2] = Eigen::Vector3f(0,0,1);

    int axis_to_rotate = 0;
    double object_limit = std::numeric_limits<double>::max();
    
    if (!with_rotate_specific_axis)
    {
         // pick smallest object Limit that correspond to the object symmetry
        for (int i = 1; i < 3; i++) {
            double tmp;
            // set the rotation step that correspond to object symmetry
            switch ((i + axis_to_align)%3) {
                case 0:
                    tmp = object.roll;
                    break;
                case 1:
                    tmp = object.pitch;
                    break;
                default:
                    tmp = object.yaw;
                    break;
            }
            if (tmp < object_limit){
                object_limit = tmp;
                axis_to_rotate = (i + axis_to_align)%3;
            }
        }
    }
    else
    {
        axis_to_rotate = rotate_around_specific_axis;
        double tmp;
        switch (axis_to_rotate) {
            case 0:
                tmp = object.roll;
                break;
            case 1:
                tmp = object.pitch;
                break;
            default:
                tmp = object.yaw;
                break;
        }
        object_limit = tmp;
    }

    if (std::floor(std::abs(angle+0.5236)/object_limit) < 1) {
        // min angle = within 30 degree to the object_limit to be aligned
        return;
    }

    // rotate target axis to align as close as possible with its original axis by 
    // rotating the best axis to align the target axis. Use -angle because we want to undo the rotation
    rot_matrix = rot_matrix * Eigen::AngleAxisf(boost::math::round(angle/object_limit) * 
        object_limit,best_axis[axis_to_rotate]);
}

template <typename numericStandard>
Eigen::Matrix<numericStandard, 3, 1> extractRPYfromRotMatrix(const Eigen::Matrix<numericStandard, 3, 3> &input, 
    bool reverse_pitch)
{
    Eigen::Matrix<numericStandard, 3, 1> result;
    numericStandard x_component = std::sqrt(input(0,0) * input(0,0) + input(1,0) *input(1,0));
    // since using sqrt, there are 2 solution for x_component
    x_component = reverse_pitch ? -x_component : x_component;
    result[1] = std::atan2(-input(2,0), x_component);
    result[2] = std::atan2(input(1,0)/std::cos(result[1]),input(0,0)/std::cos(result[1]));
    result[0] = std::atan2(input(2,1)/std::cos(result[1]),input(2,2)/std::cos(result[1]));
    return result;
}

template <typename numericStandard>
Eigen::Quaternion<numericStandard> normalizeModelOrientation(const Eigen::Quaternion<numericStandard> &q_from_pose,
    const ObjectSymmetry &object)
{
    const double pi = boost::math::constants::pi<double>();
    Eigen::Matrix3f symmetric_offset;
    Eigen::Quaternion<numericStandard> min_quaternion;
    unsigned int num_roll_steps = std::ceil(2*pi / object.roll);
    unsigned int num_pitch_steps = std::ceil(2*pi / object.pitch);
    unsigned int num_yaw_steps = std::ceil(2*pi / object.yaw);
    
    double minAngle = std::numeric_limits<double>::max();
    for (unsigned int i = 0; i < num_roll_steps; i++)
    {
        for (unsigned int j = 0; j < num_pitch_steps; j++)
        {
            for (unsigned int k = 0; k < num_yaw_steps; k++) 
            {
                symmetric_offset =  Eigen::Matrix3f::Identity()
                    * Eigen::AngleAxisf(i * object.yaw, Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(j * object.pitch, Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(k * object.roll, Eigen::Vector3f::UnitX());
                Eigen::Quaternion<numericStandard> rotated_input_quaternion = q_from_pose * 
                    Eigen::Quaternion<numericStandard>(symmetric_offset);
                
                if (minAngle > rotated_input_quaternion.angularDistance(Eigen::Quaternion<numericStandard>::Identity())) 
                {
                    minAngle = rotated_input_quaternion.angularDistance(Eigen::Quaternion<numericStandard>::Identity ());
                    min_quaternion = rotated_input_quaternion;
                }
            }
        }
    }

    Eigen::Matrix<numericStandard, 3, 3> normalize_orientation = min_quaternion.matrix();
    // realignOrientation(normalize_orientation,object,2);
    // realignOrientation(normalize_orientation,object,0,true,2); 

    return Eigen::Quaternion<numericStandard>(normalize_orientation);
}

template <typename numericStandard>
Eigen::Quaternion<numericStandard> normalizeModelOrientation(const Eigen::Quaternion<numericStandard> &q_new, 
    const Eigen::Quaternion<numericStandard>  &q_previous, const ObjectSymmetry &object)
{
  Eigen::Quaternion<numericStandard> rotation_change = q_previous.inverse() * q_new;
  // Since the rotation_change should be close to identity, 
  // realign the rotation_change as close as identity based on symmetric property of the object
  rotation_change = normalizeModelOrientation(rotation_change, object);
  
  Eigen::Quaternion<numericStandard> result = q_previous * rotation_change;
  // fix the orientation of new pose
  return (result);
}
