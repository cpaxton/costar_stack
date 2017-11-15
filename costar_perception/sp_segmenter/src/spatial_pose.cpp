#include "sp_segmenter/spatial_pose.h"

void SpatialPose::setMaxDistance(const double &distance)
{
    this->max_distance_ = distance;
}

std::size_t SpatialPose::size() const
{
    return rtree_.size();
}

std::string SpatialPose::getFrameName (const value &Node) const
{
    return std::get<1>(Node).frame_name;
}

point3d SpatialPose::generatePoint(const poseT &pose) const
{
    point3d point(pose.shift.x(),pose.shift.y(),pose.shift.z());
    return point;
}

box SpatialPose::generateBox(const poseT &pose, const double &distance) const
{
    point3d pointA(pose.shift.x()-distance,pose.shift.y()-distance,pose.shift.z()-distance),
        pointB(pose.shift.x()+distance,pose.shift.y()+distance,pose.shift.z()+distance);
    return box(pointA,pointB);
}

void SpatialPose::updateOneValue(std::string frame_name, std::vector<poseT> &all_poses, const double &timestamp, 
    std::map<std::string, unsigned int> &tf_index_map)
{
    if (all_poses.size() == 0) {
        std::cerr << "No poses to use for updating TF data\n";
        return; //Do nothing
    }

    std::cerr << "Previous rtree size: " << rtree_.size() << std::endl;

    std::vector<value> result_nn;
    rtree_.query(bgi::satisfies([frame_name](value const& node)
            { return (frame_name==std::get<1>(node).frame_name); }),
            std::back_inserter(result_nn)
        );

    std::string object_type;

    Eigen::Quaternion<float> base_rotation;
    if (result_nn.size() > 0) 
    {
        // use old orientation
        base_rotation = std::get<1>(result_nn.at(0)).pose.rotation;
        object_type = std::get<1>(result_nn.at(0)).pose.model_name;
        // remove old value
        rtree_.remove(result_nn.at(0));
    }
    else
    {
        base_rotation = base_rotation_;
    }

    // look for the target pose update value
    for (const poseT &p: all_poses)
    {
        // Do nothing if the pose match has a different object type
        if (p.model_name != object_type && result_nn.size() != 0) continue;
        else
        {
            ObjectPose tmp;
            tmp.pose = p;
            tmp.timestamp = timestamp;
            
            tmp.pose.rotation = normalizeModelOrientation<float>(p.rotation,base_rotation,object_dict_[p.model_name]);
            tmp.frame_name = frame_name;
            if (result_nn.size() == 0) tmp.index = ++tf_index_map[p.model_name];
            else tmp.index = std::get<1>(result_nn.at(0)).index;
            rtree_.insert(value(generatePoint(p),tmp)); // add new value
            break;
        }
    }
    std::cerr << "New rtree_ size: " << rtree_.size() << std::endl;
}

void SpatialPose::createNewTree(const std::map<std::string, ObjectSymmetry> &object_dict, std::vector<poseT> &all_poses, 
    const double &timestamp, std::map<std::string, unsigned int> &tf_index_map, 
    const Eigen::Quaternion<double> &base_rotation)
{
    rtree_.clear();

    base_rotation_ = base_rotation.template cast<float>();
    object_dict_ = object_dict;
    normalizeAllModelOrientation<float>(all_poses, base_rotation_, object_dict_);
    for (const poseT &p: all_poses)
    {
        ObjectPose tmp;
        tmp.pose = p;
        tmp.timestamp = timestamp;
        std::stringstream child;
        tmp.index = ++tf_index_map[p.model_name];
        // Does not have tracking yet, can not keep the label on object.
        child << "obj_" << p.model_name << "_" << tmp.index;
        tmp.frame_name = child.str();
        rtree_.insert(value(generatePoint(p),tmp));
    }
}

void SpatialPose::updateTree(const std::vector<poseT> &all_poses,
    const double &timestamp, std::map<std::string, unsigned int> &tf_index_map)
{
    std::cerr << "Previous rtree size: " << rtree_.size() << std::endl;

    // get the closest point that belongs to the same object
    int k_points = 1;

    for (const poseT &p: all_poses)
    {
        ObjectPose tmp;
        tmp.timestamp = timestamp;
        std::vector<value> result_nn;
        box boundary = generateBox(p, max_distance_);
        tmp.pose = p;
        rtree_.query(
                bgi::nearest(generatePoint(p),k_points) && bgi::within(boundary) && 
                bgi::satisfies([&](value const& v) {return  std::get<1>(v).pose.model_name == p.model_name;}),
                std::back_inserter(result_nn)
            );
        
        // the pose exists in the current rtree
        if (result_nn.size() > 0) 
        {
            tmp.pose.rotation = normalizeModelOrientation<float>(
                                                                p, std::get<1>(result_nn.at(0)).pose,
                                                                object_dict_[std::get<1>(result_nn.at(0)).pose.model_name]
                                                                );
            tmp.frame_name = std::get<1>(result_nn.at(0)).frame_name;
            tmp.index = std::get<1>(result_nn.at(0)).index;
            rtree_.remove(result_nn.at(0));
        }
        else // new pose
        {
            tmp.pose.rotation = normalizeModelOrientation<float>(p.rotation, base_rotation_, object_dict_[p.model_name]);
            std::stringstream child;
            tmp.index = ++tf_index_map[p.model_name];
            // Does not have tracking yet, can not keep the label of object.
            child << "obj_" << p.model_name << "_" << tmp.index;
            tmp.frame_name = child.str();
        }
        
        rtree_.insert(value(generatePoint(tmp.pose),tmp));
    }

    std::cerr << "Current rtree size: " << rtree_.size() << std::endl;
}

std::vector<value> SpatialPose::getAllNodes () const
{
    std::vector<value> nodes;
    rtree_.query(bgi::satisfies([](value const&){ return true; }),std::back_inserter(nodes));
    return nodes;
}


std::vector<value> SpatialPose::getRecentNodes(const double &cur_timestamp, const double &max_delta_time) const
{
    std::vector<value> nodes;
    rtree_.query(bgi::satisfies([cur_timestamp,max_delta_time](value const& node){
            return (cur_timestamp - std::get<1>(node).timestamp) < max_delta_time; }
        ),std::back_inserter(nodes));
    return nodes;
}

std::vector<poseT> SpatialPose::getAllPoses () const
{
    std::vector<value> nodes = this->getAllNodes();
    std::vector<poseT> poses;
    for (size_t i = 0; i < nodes.size(); i++) {
        poses.push_back(std::get<1>(nodes.at(i)).pose);
    }
    return poses;
}