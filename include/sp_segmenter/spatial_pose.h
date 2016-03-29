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
    unsigned int index;
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

void treeInsert(objectRtree &rtree, const std::vector<poseT> &all_poses, const double &timestamp, 
    std::map<std::string, unsigned int> objectTFindex)
{
    for (const poseT &p: all_poses) {
        objectPose tmpObject;
        tmpObject.pose = p;
//        tmpObject.name = p.model_name;
        tmpObject.timestamp = timestamp;
        std::stringstream child;
        tmpObject.index = ++objectTFindex[p.model_name];
        // Does not have tracking yet, can not keep the label on object.
        child << "Obj::" << p.model_name << "::" << tmpObject.index;
        tmpObject.tfName = child.str();
        rtree.insert(value(generatePoint(p),tmpObject));
    }
}

void updateOneValue(objectRtree &rtree, std::string tfToUpdate, const std::map<std::string, objectSymmetry> &objectDict, 
    std::vector<poseT> &all_poses, const double &timestamp, 
    std::map<std::string, unsigned int> &objectTFindex, 
    const Eigen::Quaternion<double> baseRotationInput = Eigen::Quaternion<double>(1,0,0,0))
{
  if (all_poses.size() < 1) {
    std::cerr << "No poses to use for updating TF data\n";
    return; //Do nothing
  }
  std::cerr << "Old rtree size: " << rtree.size() << std::endl;

  std::vector<value> result_nn;
  rtree.query(bgi::satisfies([tfToUpdate](value const& node){ 
    // std::cerr << tfToUpdate << " " << std::get<1>(node).tfName << " match: "<< (tfToUpdate==std::get<1>(node).tfName) <<std::endl;
    return (tfToUpdate==std::get<1>(node).tfName); 
    }),std::back_inserter(result_nn));
  objectPose tmpObject;
  std::string object_type;

  Eigen::Quaternion<float> baseRotation;
  if (result_nn.size() > 0) 
  {
    std::cerr << "Found old pose: " << result_nn.size() << std::endl;
    baseRotation = std::get<1>(result_nn.at(0)).pose.rotation; // use old orientation
    object_type = std::get<1>(result_nn.at(0)).pose.model_name; // get old object type
    rtree.remove(result_nn.at(0)); // remove old value
  }
  else
  {
    baseRotation = Eigen::Quaternion<float>(baseRotationInput.w(),baseRotationInput.x(),baseRotationInput.y(),baseRotationInput.z());
  }

  for (const poseT &p: all_poses) {
    if (p.model_name!=object_type && result_nn.size() != 0) continue; //Do nothing to pose from different object type
    else
    {
        objectPose tmpObject;
        tmpObject.pose = p;
        //        tmpObject.name = p.model_name;
        tmpObject.timestamp = timestamp;
        std::stringstream child;
        
        tmpObject.pose.rotation = normalizeModelOrientation<float>(p.rotation,baseRotation,objectDict.find(p.model_name)->second);
        tmpObject.tfName = tfToUpdate;
        if (result_nn.size() == 0)
        {
            tmpObject.index = ++objectTFindex[p.model_name];
        }
        else tmpObject.index = std::get<1>(result_nn.at(0)).index;
        rtree.insert(value(generatePoint(p),tmpObject)); // add new value
        break;
    }
  }
  std::cerr << "New rtree size: " << rtree.size() << std::endl;
}

void createTree(objectRtree &rtree, const std::map<std::string, objectSymmetry> &objectDict, std::vector<poseT> &all_poses, 
    const double &timestamp, std::map<std::string, unsigned int> &objectTFindex, 
    const Eigen::Quaternion<double> baseRotationInput = Eigen::Quaternion<double>(1,0,0,0))
{
    Eigen::Quaternion<float> baseRotation(baseRotationInput.w(),baseRotationInput.x(),baseRotationInput.y(),baseRotationInput.z());
    normalizeAllModelOrientation<float>(all_poses, baseRotation, objectDict);
    treeInsert(rtree, all_poses, timestamp, objectTFindex);
}

void updateTree(objectRtree &rtree, const std::map<std::string, objectSymmetry> &objectDict, const std::vector<poseT> &all_poses,
    const double &timestamp, std::map<std::string, unsigned int> &objectTFindex)
{
    objectRtree tmpRtree;
    std::cerr << "Old rtree size: " << rtree.size() << std::endl;
    
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
            tmpObject.pose.rotation = normalizeModelOrientation<float>(
                                                                p, std::get<1>(result_nn.at(0)).pose,
                                                                objectDict.find(std::get<1>(result_nn.at(0)).pose.model_name)->second
                                                                );
            // tmpObject.tfName = std::get<1>(result_nn.at(0)).tfName; // Disabled keeping old TF name
            tmpObject.index = ++objectTFindex[p.model_name];
            // Does not have tracking yet, can not keep the label on object.
            std::stringstream child;
            child << "Obj::" << p.model_name << "::" << tmpObject.index;
            tmpObject.index = std::get<1>(result_nn.at(0)).index;
            tmpObject.tfName = child.str();
            rtree.remove(result_nn.at(0)); // each nearest neighboor only can be assigned to one match
        
        }
        else // new pose
        {
            tmpObject.pose.rotation = normalizeModelOrientation<float>(p.rotation, objectDict.find(p.model_name)->second);
            std::stringstream child;
            tmpObject.index = ++objectTFindex[p.model_name];
            // Does not have tracking yet, can not keep the label on object.
            child << "Obj::" << p.model_name << "::" << tmpObject.index;
            tmpObject.tfName = child.str();
        }
        
        tmpRtree.insert(value(generatePoint(tmpObject.pose),tmpObject));
    }

    rtree.swap(tmpRtree); //get new rtree value
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
