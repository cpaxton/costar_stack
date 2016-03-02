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
#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include "sp_segmenter/utility/typedef.h"
#include <math.h>

//for normalizing object rotation
struct objectSymmetry
{
    double roll;
    double pitch;
    double yaw;
};


std::map<std::string, objectSymmetry> fillDictionary(ros::NodeHandle nh, std::vector<std::string> cur_name)
{
    const double pi = 3.1415926535897;
    
    std::map<std::string, objectSymmetry> objectDict;
    for (unsigned int i = 0; i < cur_name.size(); i++) {
        objectSymmetry tmp;
        nh.param(cur_name.at(i)+"/x", tmp.roll,360.0);
        nh.param(cur_name.at(i)+"/y", tmp.pitch,360.0);
        nh.param(cur_name.at(i)+"/z", tmp.yaw,360.0);
        
        //convert degree to radians
        tmp.roll = tmp.roll / 180 * pi;
        tmp.pitch = tmp.pitch / 180 * pi;
        tmp.yaw = tmp.yaw / 180 * pi;
        
        objectDict[cur_name.at(i)] = tmp;
    }
    return objectDict;
}

void realignOrientation (Eigen::Matrix3f &rotMatrix, objectSymmetry object, int axisToAlign)
{
//    const float pi = 3.1415926535897;
    
    Eigen::Vector3f objAxes[3];
    objAxes[0] = Eigen::Vector3f(rotMatrix(0,0),rotMatrix(1,0),rotMatrix(2,0));
    objAxes[1] = Eigen::Vector3f(rotMatrix(0,1),rotMatrix(1,1),rotMatrix(2,1));
    objAxes[2] = Eigen::Vector3f(rotMatrix(0,2),rotMatrix(1,2),rotMatrix(2,2));
    
    
    Eigen::Vector3f axis(0,0,0);
    axis[axisToAlign] = 1;
    double dotProduct = axis.dot(objAxes[axisToAlign]);
    double angle = std::acos(dotProduct); //since the vector is unit vector
    
//    Eigen::Vector3f rotAxis = axis.cross(objAxes[axisToAlign]);
    Eigen::Vector3f bestAxis[3];
    bestAxis[0] = Eigen::Vector3f(1,0,0);
    bestAxis[1] = Eigen::Vector3f(0,1,0);
    bestAxis[2] = Eigen::Vector3f(0,0,1);
    
    // get best axis to rotate for aligning target axis
    //    ; double axisAngle = 0;
    //    for (int i = 0; i < 3; i++) {
    //        float tmp = rotAxis.dot(bestAxis[i]);
    //        if (i != axisToAlign && abs(tmp) > axisAngle) {
    //            closestAxisIndex = i;
    ////            sign = std::copysign(1.0, tmp); //get the direction of the rotation
    //            axisAngle = abs(tmp);
    //        }
    //    }
    
    //    angle *= sign;
    int axisToRotate = 0;
    double objectLimit = 999;
    // pick smallest object Limit that correspond to the object symmetry
    for (int i = 1; i < 3; i++) {
        double tmp;
        // set the rotation step that correspond to object symmetry
        switch ((i+axisToAlign)%3) {
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
        if (tmp < objectLimit){
            objectLimit = tmp;
            axisToRotate = (i + axisToAlign)%3;
        }
    }
    
    //    std::cerr << "Number of step: " << std::floor(abs(angle)/objectLimit + 0.3333) << " " << angle * 180 / pi << " " << objectLimit * 180 / pi <<std::endl;
    
    if (std::floor(abs(angle)/objectLimit + 0.3333) < 1) {
        //min angle = 66% of the objectLimit to be aligned
        return;
    }
    
    // rotate target axis to align as close as possible with its original axis by rotating the best axis to align the target axis. Use -angle because we want to undo the rotation
    rotMatrix = rotMatrix * Eigen::AngleAxisf(std::round(angle/objectLimit) * objectLimit,bestAxis[axisToRotate]);
    //    std::cerr << "Rotate axis" << axisToRotate << " : " << angle * 180 / pi << " -> " << std::round(angle/objectLimit) * objectLimit * 180 / pi <<std::endl;
}

void normalizeModelOrientation(poseT &p, const objectSymmetry object)
{
//    const float pi = 3.1415926535897;
    float yaw,pitch,roll;
    Eigen::Matrix3f rotMatrix = p.rotation.toRotationMatrix();
    //    std::cerr << "Initial Rot Matrix: \n" << rotMatrix << std::endl;
    
    //convert to euler angle
    Eigen::Vector3f ea = rotMatrix.eulerAngles(2,1,0);
    //    std::cerr << "RPY: "<< ea * 180/pi << std::endl;
    roll = std::round(ea[2]/object.roll) * object.roll;
    pitch = std::round(ea[1]/object.pitch) * object.pitch;
    yaw = std::round(ea[0]/object.yaw) * object.yaw;
    
    rotMatrix = rotMatrix * Eigen::AngleAxisf(- roll, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(- pitch, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(- yaw, Eigen::Vector3f::UnitZ());
    //    std::cerr << "Undo RPY Rot Matrix: \n" << rotMatrix << std::endl;
    
    ea = rotMatrix.eulerAngles(2,1,0);
    //    std::cerr << "Final RPY: "<< ea * 180/pi << std::endl;
    
    //  realign again with different method to remove the effect of euler angle singularity
    realignOrientation(rotMatrix, object, 2);
    //    std::cerr << "Realign z Rot Matrix: \n" << rotMatrix << std::endl;
    realignOrientation(rotMatrix, object, 1);
    //    std::cerr << "Realign y Rot Matrix: \n" << rotMatrix << std::endl;
    p.rotation = Eigen::Quaternion<float>(rotMatrix);
}

void normalizeAllModelOrientation (std::vector<poseT> &all_poses, std::map<std::string, objectSymmetry> objectDict)
{
    for (unsigned int i = 0; i < all_poses.size(); i++)
    {
        //        std::cerr << "Object: " << all_poses[i].model_name << std::endl;
        normalizeModelOrientation(all_poses[i],objectDict[all_poses[i].model_name]);
    }
}



#endif /* symmetricOrientationRealignment_h */
