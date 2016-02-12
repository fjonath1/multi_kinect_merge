//
//  stringToArrayLoader.h
//  
//
//  Created by Felix Jo on 2/8/16.
//
//

#ifndef stringToArrayLoader_h
#define stringToArrayLoader_h
#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include "multi_kinect_merge/stringVectorArgsReader.h"
#include <sstream>

template <typename T>
Eigen::Transform<T,3,Eigen::Affine> getTransformFromParam(const ros::NodeHandle &nh, const std::string &param_name)
{
    Eigen::Quaternion<T> rotation;
    std::vector<T> tmp = stringVectorArgsReader<T>(nh,param_name + "/quaternion","");
    if (tmp.size() == 4) {
        std::vector<T> quaternion = tmp;
//        stringVectorToNumber(tmp,quaternion);
        rotation = Eigen::Quaternion<T> (quaternion[3],quaternion[0],quaternion[1],quaternion[2]);
    }
    else rotation.setIdentity();
    
    Eigen::Translation<T,3> translation(0,0,0);
    tmp = stringVectorArgsReader<T>(nh,param_name + "/translation","");
    if (tmp.size() == 3) {
        std::vector<T> xyz = tmp;
//        stringVectorToNumber(tmp,xyz);
        translation = Eigen::Translation<T,3> (xyz[0],xyz[1],xyz[2]);
    }
    
    Eigen::Transform<T,3,Eigen::Affine> result(translation * rotation);
    return result;
}

template <typename T>
void publishICPparams(const ros::NodeHandle &nh, const Eigen::Transform<T,3,Eigen::Affine> &inputTransform, const std::string transformName)
{
    const Eigen::Quaternion<T> rotation(inputTransform.rotation());
    std::stringstream ss;
    ss << rotation.x() << "," << rotation.y() << "," << rotation.z() << "," << rotation.w();
    nh.setParam(transformName+"/quaternion", ss.str());
    
    const Eigen::Translation<T,3> translation(inputTransform.translation());
    ss.str(""); ss.clear();
    ss << translation.x() << "," << translation.y() << "," << translation.z();
    nh.setParam(transformName+"/translation", ss.str());
}

#endif /* stringToArrayLoader_h */
