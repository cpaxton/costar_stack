//
//  symmetricOrientationRealignment.h
//  
//
//  Created by Felix Jo on 1/27/16.
//
//

#ifndef symmetricOrientationRealignment_h
#define symmetricOrientationRealignment_h

// ros stuff
#include <ros/ros.h>
#include <Eigen/Geometry>
#include "sp_segmenter/utility/typedef.h"
#include "sp_segmenter/utility/utility.h"
#include <math.h>
#include <boost/math/constants/constants.hpp>

// Pass current ROS node handle
// Load in parameters for objects from ROS namespace
std::map<std::string, objectSymmetry> fillDictionary(const ros::NodeHandle &nh, const std::vector<std::string> &cur_name)
{
    const double degToRad = boost::math::constants::pi<double>() / 180;
    std::map<std::string, objectSymmetry> objectDict;
    std::cerr << "LOADING IN OBJECTS\n";
    for (unsigned int i = 0; i < cur_name.size(); i++) {
        std::cerr << "Name of obj: " << cur_name[i] << "\n";
        objectSymmetry tmp;
        nh.param(cur_name.at(i)+"/x", tmp.roll,360.0);
        nh.param(cur_name.at(i)+"/y", tmp.pitch,360.0);
        nh.param(cur_name.at(i)+"/z", tmp.yaw,360.0);
        
        //convert degree to radians
        tmp.roll = tmp.roll * degToRad ;
        tmp.pitch = tmp.pitch * degToRad;
        tmp.yaw = tmp.yaw * degToRad;
        
        objectDict[cur_name.at(i)] = tmp;
    }
    return objectDict;
}

template <typename numericStandard>
Eigen::Matrix<numericStandard, 3, 1> extractRPYfromRotMatrix(const Eigen::Matrix<numericStandard, 3, 3> &input, bool reversePitch = false)
{
    Eigen::Matrix<numericStandard, 3, 1> result;
    numericStandard x_component = std::sqrt(input(0,0) * input(0,0) + input(1,0) *input(1,0));
    // since using sqrt, there are 2 solution for x_component
    x_component = reversePitch ? -x_component : x_component;
    result[1] = std::atan2(-input(2,0), x_component);
    result[2] = std::atan2(input(1,0)/std::cos(result[1]),input(0,0)/std::cos(result[1]));
    result[0] = std::atan2(input(2,1)/std::cos(result[1]),input(2,2)/std::cos(result[1]));
    return result;
}

template <typename numericType>
Eigen::Quaternion<numericType> normalizeModelOrientation(const Eigen::Quaternion<numericType> &q_from_pose, const objectSymmetry &object)
{
    const double pi = boost::math::constants::pi<double>();
    Eigen::Matrix3f symmetricOffset;
    Eigen::Quaternion<numericType> minQuaternion;
    // std::cout << ceil(2*pi / object.roll) << ", " << 
    //     ceil(2*pi / object.yaw) << ", " << 
    //     ceil(2*pi / object.roll) << ", ";
    double minAngle = 100;
    for (unsigned int i = 0; i < ceil(2*pi / object.roll); i++)
        for (unsigned int j = 0; j < ceil(2*pi / object.pitch); j++)
            for (unsigned int k = 0; k < ceil(2*pi / object.yaw); k++)
            {
                symmetricOffset =  Eigen::Matrix3f::Identity()
                    * Eigen::AngleAxisf(i * object.yaw, Eigen::Vector3f::UnitZ())
                    * Eigen::AngleAxisf(j * object.pitch, Eigen::Vector3f::UnitY())
                    * Eigen::AngleAxisf(k * object.roll, Eigen::Vector3f::UnitX());
                Eigen::Quaternion<numericType> rotatedInputQuaternion = q_from_pose * Eigen::Quaternion<float>(symmetricOffset);
                if (minAngle > rotatedInputQuaternion.angularDistance(Eigen::Quaternion<float>::Identity())) 
                {
                    minAngle = rotatedInputQuaternion.angularDistance(Eigen::Quaternion<float>::Identity ());
                    minQuaternion = rotatedInputQuaternion;
                    // std::cout << "Found better matrix with angular distance: " << minAngle * 180 / pi << std::endl;
                }
            }
    std::cout << "Best rotation matrix: \n" << minQuaternion.matrix() << std::endl;
    // std::cout << "Best angular Distance:" << minQuaternion.angularDistance(Eigen::Quaternion<float>::Identity()) << std::endl;
    
    return minQuaternion;
}

template <typename numericType>
void printQuaternion(const Eigen::Quaternion<numericType> &input)
{
  std::cerr << "Q: " << input.w() << ", " << input.x() << ", " << input.y() << ", " << input.z() << std::endl;
}

template <typename numericType>
Eigen::Quaternion<numericType> normalizeModelOrientation(const Eigen::Quaternion<numericType> &q_new, const Eigen::Quaternion<numericType>  &q_previous, const objectSymmetry &object)
{
  std::cerr << "Input Qnew: "; printQuaternion(q_new);
  std::cerr << "Input Qold: "; printQuaternion(q_previous);
  Eigen::Quaternion<float> rotationChange = q_previous.inverse() * q_new;
  // Since the rotationChange should be close to identity, realign the rotationChange as close as identity based on symmetric property of the object
  rotationChange = normalizeModelOrientation(rotationChange, object);
  
  std::cerr << "Output Q: "; printQuaternion(q_previous * rotationChange);
  // fix the orientation of new pose
  return (q_previous * rotationChange);
}

template <typename numericType>
Eigen::Quaternion<numericType> normalizeModelOrientation(const poseT &newPose,const poseT &previousPose, const objectSymmetry &object)
{
    return (normalizeModelOrientation(newPose.rotation,previousPose.rotation,object));
}


template <typename numericType>
void normalizeAllModelOrientation (std::vector<poseT> &all_poses, const Eigen::Quaternion<numericType> &normalOrientation, const std::map<std::string, objectSymmetry> &objectDict)
{
  for (unsigned int i = 0; i < all_poses.size(); i++)
  {
    // std::cerr << "Rot"<< i <<" before normalization: \n" << all_poses[i].rotation.toRotationMatrix() << std::endl;
    std::cerr << "Object name: " << all_poses[i].model_name << std::endl;
    all_poses[i].rotation = normalizeModelOrientation(all_poses[i].rotation, normalOrientation, objectDict.find(all_poses[i].model_name)->second);
    // std::cerr << "Rot"<< i <<" after normalization: \n" << all_poses[i].rotation.toRotationMatrix() << std::endl;
  }
}

void normalizeAllModelOrientation (std::vector<poseT> &all_poses, const std::map<std::string, objectSymmetry> &objectDict)
{
    for (unsigned int i = 0; i < all_poses.size(); i++)
    {
        //        std::cerr << "Object: " << all_poses[i].model_name << std::endl;
        all_poses[i].rotation = normalizeModelOrientation(all_poses[i].rotation, objectDict.find(all_poses[i].model_name)->second);
    }
}

#endif /* symmetricOrientationRealignment_h */
