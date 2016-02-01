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

double maxDeltaTime = 60; //1 minute delta time max

struct objectPose {
    poseT pose;
//    std::string name;
    double timestamp;
    std::string tfName;
    
    // define node equality for rtree remove function
    bool operator==(const objectPose& valueToCompare) const
    {
        return (
//                tfName == valueToCompare.tfName && timestamp == valueToCompare.timestamp &&
                pose.model_name == valueToCompare.pose.model_name &&
                pose.shift == valueToCompare.pose.shift &&
                pose.rotation.isApprox(valueToCompare.pose.rotation)
                );
    }
};

typedef bg::model::point<double, 3, bg::cs::cartesian> point3d; // boost point (x,y,z) double
typedef bg::model::box<point3d> box;
typedef std::pair<point3d,objectPose> value;
typedef bgi::rtree <value, bgi::linear<16> > objectRtree;

struct is_correct_model
{
    is_correct_model(std::string model_name)
    :model_name_(model_name)
    {}
    
    template <typename Value>
    bool operator()(Value const& v)
    {
        return std::get<1>(v).pose.model_name == model_name_;
    }
    
    std::string model_name_;
};

std::string getTFname (const value &Node)
{
    return std::get<1>(Node).tfName;
}

point3d generatePoint(const poseT &pose)
{
    point3d point(pose.shift.x(),pose.shift.y(),pose.shift.z());
    return point;
}

box generateBox(const poseT &pose, const double &distance)
{
    point3d pointA(pose.shift.x()-distance,pose.shift.y()-distance,pose.shift.z()-distance),
        pointB(pose.shift.x()+distance,pose.shift.y()+distance,pose.shift.z()+distance);
    return box(pointA,pointB);
}

void treeInsert(objectRtree &rtree, const std::vector<poseT> &all_poses, const double &timestamp)
{
    int index = 0;
    for (const poseT &p: all_poses) {
        objectPose tmpObject;
        tmpObject.pose = p;
//        tmpObject.name = p.model_name;
        tmpObject.timestamp = timestamp;
        std::stringstream child;
        // Does not have tracking yet, can not keep the label on object.
        child << "Obj::" << p.model_name << "::" << ++index;
        tmpObject.tfName = child.str();
        rtree.insert(std::make_pair(generatePoint(p),tmpObject));
    }
}


void createTree(objectRtree &rtree, const std::map<std::string, objectSymmetry> &objectDict, std::vector<poseT> &all_poses, const double &timestamp)
{
    normalizeAllModelOrientation(all_poses, objectDict);
    treeInsert(rtree, all_poses, timestamp);
}

void updateTree(objectRtree &rtree, const std::map<std::string, objectSymmetry> &objectDict, const std::vector<poseT> &all_poses, const double &timestamp, const bool removeNode = true)
{
    objectRtree tmpRtree;
    size_t treeSize = rtree.size();
    std::cerr << "Old rtree size: " << rtree.size() << std::endl;
    if (!removeNode) tmpRtree = rtree; //transfer rtree to tmpRtree
    
    //get closest point that belongs to same object
    int knnNumber = 2;
    double distance = 0.025; // Limit search area to 2.5 cm around the point
    for (const poseT &p: all_poses) {
        objectPose tmpObject;
        //        tmpObject.name = p.model_name;
        tmpObject.timestamp = timestamp;
        std::vector<value> result_nn;
        box boundary = generateBox(p, distance);
        tmpObject.pose = p;
        rtree.query(bgi::nearest(generatePoint(p),knnNumber)
                    && bgi::within(boundary)
                    && bgi::satisfies(
                    [&](value const& v) {return  std::get<1>(v).pose.model_name == p.model_name;}
                    )
                    ,
                    std::back_inserter(result_nn)
                    );
        
        
        if (result_nn.size() > 0) { // the pose exists in the prior rTree
            std::cerr << "Found old value\n";
            tmpObject.pose.rotation = normalizeModelOrientation(
                                                                p, std::get<1>(result_nn.at(0)).pose,
                                                                objectDict.find(std::get<1>(result_nn.at(0)).pose.model_name)->second
                                                                );
            tmpObject.tfName = std::get<1>(result_nn.at(0)).tfName;
//            if (removeNode) rtree.remove(result_nn[0]);
        
        }
        else // new pose
        {
            tmpObject.pose.rotation = normalizeModelOrientation(p.rotation, objectDict.find(p.model_name)->second);
            std::stringstream child;
            // Does not have tracking yet, can not keep the label on object.
            child << "Obj::" << p.model_name << "::" << ++treeSize;
            tmpObject.tfName = child.str();
        }
        
        tmpRtree.insert(std::make_pair(generatePoint(tmpObject.pose),tmpObject));
    }

    rtree.swap(tmpRtree); //get the rtree value back
    std::cerr << "New rtree size: " << rtree.size() << std::endl;
}

std::vector<value> getAllNodes (const objectRtree &rtree)
{
    std::vector<value> nodes;
    rtree.query(bgi::satisfies([](value const&){ return true; }),std::back_inserter(nodes));
    return nodes;
}

std::vector<poseT> getAllPoses (const objectRtree &rtree)
{
    std::vector<value> nodes = getAllNodes(rtree);
    std::vector<poseT> poses;
    for (size_t i = 0; i < nodes.size(); i++) {
        poses.push_back(std::get<1>(nodes.at(i)).pose);
    }
    return poses;
}

#endif /* spatial_pose_h */
