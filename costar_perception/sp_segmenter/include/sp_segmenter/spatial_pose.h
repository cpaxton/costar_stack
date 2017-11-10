//
//  spatial_pose.h
//  
//
//  Created by Felix Jo on 1/28/16.
//
//

#ifndef spatial_pose_h
#define spatial_pose_h

// for building spatial data structure
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>

// for orientation fixer
#include "sp_segmenter/symmetricOrientationRealignment.h"

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

struct ObjectPose {
    poseT pose;
    double timestamp;
    std::string frame_name;
    unsigned int index;
    // define node equality for rtree_ remove function
    bool operator==(const ObjectPose& value_to_compare) const
    {
        return (
                pose.model_name == value_to_compare.pose.model_name &&
                pose.shift == value_to_compare.pose.shift &&
                pose.rotation.isApprox(value_to_compare.pose.rotation)
                );
    }
};

typedef bg::model::point<double, 3, bg::cs::cartesian> point3d; // boost point (x,y,z) double
typedef bg::model::box<point3d> box;
typedef std::pair<point3d,ObjectPose> value;

class SpatialPose
{
public:
    SpatialPose() : max_distance_(0.025) {};
    void createNewTree(const std::map<std::string, ObjectSymmetry> &object_dict, std::vector<poseT> &all_poses, 
        const double &timestamp, std::map<std::string, unsigned int> &tf_index_map, 
        const Eigen::Quaternion<double> &base_rotation = Eigen::Quaternion<double>(1,0,0,0));
    void updateTree(const std::vector<poseT> &all_poses,
        const double &timestamp, std::map<std::string, unsigned int> &tf_index_map);
    void updateOneValue(std::string frame_name, std::vector<poseT> &all_poses, const double &timestamp, 
        std::map<std::string, unsigned int> &tf_index_map);
    void setMaxDistance(const double &distance);
    std::size_t size() const;
    std::vector<value> getAllNodes() const;
    std::vector<poseT> getAllPoses() const;
    std::vector<value> getRecentNodes(const double &cur_timestamp, const double &max_delta_time = 0.20) const;
    std::string getFrameName (const value &Node) const;

private:
    point3d generatePoint(const poseT &pose) const;
    box generateBox(const poseT &pose, const double &distance) const;

    std::map<std::string, ObjectSymmetry> object_dict_;
    Eigen::Quaternion<float> base_rotation_;
    bgi::rtree<value, bgi::linear<16> > rtree_;
    double max_distance_;
};

#endif /* spatial_pose_h */
